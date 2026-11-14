"""Teste SIL: compila o C GERADO e compara com o modelo de referencia (float64 e emulacao float32)."""
import sys, os, subprocess, ctypes, json; sys.path.insert(0, '../src'); sys.path.insert(0, '.')
import numpy as np
import design as D, refmodel, supervisor as S
from params import *
import scenario

GEN = '../generated'
subprocess.check_call(['gcc', '-std=c99', '-O2', '-ffp-contract=off', '-fno-fast-math', '-fPIC', '-shared', '-Wall', '-Wextra',
                       f'{GEN}/aurora_ctrl.c', f'{GEN}/aurora_sup.c', '-I', GEN, '-o', 'libaurora.so'])
lib = ctypes.CDLL('./libaurora.so')
class CS(ctypes.Structure): _fields_ = [('z', ctypes.c_float), ('rprev', ctypes.c_float), ('rcomp', ctypes.c_float)]
class CO(ctypes.Structure): _fields_ = [('u', ctypes.c_float), ('u_cmd', ctypes.c_float)]
class SS(ctypes.Structure): _fields_ = [('mode', ctypes.c_uint8), ('cnt', ctypes.c_uint8), ('prev_sensed', ctypes.c_uint8), ('prev_rearm', ctypes.c_uint8), ('drv', ctypes.c_uint8)]
lib.aurora_ctrl_step.argtypes = [ctypes.POINTER(CS)] + [ctypes.c_float]*4 + [ctypes.c_uint8]*2 + [ctypes.POINTER(CO)]
lib.aurora_sup_step.argtypes = [ctypes.POINTER(SS), ctypes.c_uint8, ctypes.c_uint8]

class CCtrl:
    def __init__(self): self.s = CS(0, 0, 0); self.o = CO()
    def __call__(self, i, w, th, r, collab, drv):
        lib.aurora_ctrl_step(ctypes.byref(self.s), i, w, th, r, int(collab), int(drv), ctypes.byref(self.o)); return float(self.o.u)
class CSup:
    def __init__(self): self.s = SS(S.MOVING, 0, 0, 0, 1)
    def __call__(self, sensed, rearm):
        lib.aurora_sup_step(ctypes.byref(self.s), int(sensed), int(rearm)); return int(self.s.mode), int(self.s.drv)
class PySup:
    def __init__(self): self.s = S.INIT
    def __call__(self, sensed, rearm): self.s, _ = S.step(self.s, (sensed, rearm)); return self.s[0], self.s[4]
def refctrl(dt):
    c = refmodel.consts(D.lqr_gains((0,0,1,1000), 1e-2)[0].ravel(), 1.0, 0.0, D.Ad[0,0], D.Ad[0,1], D.Bd[0,0], TS, VMAX, D.IMAX_LIM, V_TCP_DESIGN/R_TCP*N*TS, dt)
    st = dict(z=dt(0), rprev=dt(0), rcomp=dt(0))
    return (lambda i, w, th, r, collab, drv: float(refmodel.step(c, st, i, w, th, r, collab, drv)[0])), st

out = {}
# ---- 1) supervisor C vs Python: suite gerada + 300k passos aleatorios ----
T = json.load(open('../tests/generated_tests.json'))
bad = 0; steps = 0
for seq, exp in zip(T['tests'], T['expected']):
    cs = CSup()
    for inp, e in zip(seq, exp):
        cs(*inp); steps += 1
        if (cs.s.mode, cs.s.drv, cs.s.cnt) != tuple(e): bad += 1
rng = np.random.default_rng(0); cs = CSup(); ps = S.INIT; badr = 0; N_RAND = 300000
for _ in range(N_RAND):
    inp = (int(rng.random() < 0.3), int(rng.random() < 0.3))
    cs(*inp); ps, _ = S.step(ps, inp)
    if (cs.s.mode, cs.s.cnt, cs.s.drv, cs.s.prev_rearm) != (ps[0], ps[1], ps[4], ps[3]): badr += 1
out['sup'] = dict(suite_steps=steps, suite_mismatch=bad, random_steps=N_RAND, random_mismatch=badr)

# ---- 2) controlador: entradas gravadas do cenario integrado (open-loop feed) ----
ev = scenario.make_events()
ref64, st64 = refctrl(np.float64)
L, inp = scenario.run(ev, ref64, PySup())               # cenario conduzido pela referencia double
ref64b, st64b = refctrl(np.float64); ref32, _ = refctrl(np.float32); cc = CCtrl()
u64 = []; u32 = []; uc = []; Ssum = []
Kg = D.lqr_gains((0,0,1,1000), 1e-2)[0].ravel()
for (i, w, th, r, collab, drv) in inp:
    Ssum.append(abs(Kg[0]*i) + abs(Kg[1]*w) + abs(Kg[2])*(abs(th) + abs(float(st64b['rprev']))) + abs(Kg[3]*float(st64b['z'])))
    u64.append(ref64b(i, w, th, r, collab, drv)); u32.append(ref32(i, w, th, r, collab, drv)); uc.append(cc(i, w, th, r, collab, drv))
u64, u32, uc = map(np.array, (u64, u32, uc))
EPS32 = float(np.finfo(np.float32).eps); EPS64 = float(np.finfo(np.float64).eps)
e64 = np.abs(uc - u64); e32 = np.abs(uc - u32)
out['ctrl_feed'] = dict(n=len(uc), umax=float(np.abs(u64).max()), max_abs_err_vs_f64=float(e64.max()), max_abs_err_vs_f32=float(e32.max()),
    n_nonzero_vs_f32=int((e32 > 0).sum()), eps32=EPS32, eps64=EPS64, err_over_eps32_umax=float(e64.max()/(EPS32*np.abs(u64).max())),
    max_rel_err_vs_f64=float((e64/np.maximum(np.abs(u64), 1.0)).max()), z_c=float(cc.s.z), z_ref=float(st64['z']))
Ssum = np.array(Ssum)
out['conditioning'] = dict(ratio_max=float((e64/(EPS32*np.maximum(Ssum, 1e-9))).max()), S_at_max=float(Ssum[int(e64.argmax())]), err_max=float(e64.max()))
out['coverage_of_scenario'] = dict(sat_cycles=int((np.abs(u64) >= VMAX-1e-9).sum()), icur_lim_cycles=int((np.abs(L['i']) >= 7.49).sum()),
    collab_cycles=int(inp[:,4].sum()), sto_cycles=int((inp[:,5] == 0).sum()), halts=int(((L['mode'][1:] == 1) & (L['mode'][:-1] != 1)).sum()))

# ---- 3) laco fechado com o C no laco vs referencia double ----
Lref, _ = scenario.run(ev, refctrl(np.float64)[0], PySup())
Lc, _ = scenario.run(ev, CCtrl(), CSup())
out['closed_loop'] = dict(max_dtheta=float(np.abs(Lref['th'] - Lc['th']).max()), max_dw=float(np.abs(Lref['w'] - Lc['w']).max()),
    max_di=float(np.abs(Lref['i'] - Lc['i']).max()), theta_range=float(np.abs(Lref['th']).max()), mode_mismatch=int((Lref['mode'] != Lc['mode']).sum()),
    imax=float(np.abs(Lc['i']).max()), umax=float(np.abs(Lc['u']).max()),
    rel_dtheta=float(np.abs(Lref['th'] - Lc['th']).max()/np.abs(Lref['th']).max()))
json.dump(out, open('../results/sil.json', 'w'), indent=1)
evf = scenario.make_events(variant='fast_entry'); Lf, _ = scenario.run(evf, CCtrl(), CSup())
np.savez('../results/scenario_log_fastentry.npz', **{k: v for k, v in Lf.items()}, r=evf['r'], zone=evf['zone'], hand=evf['hand'], rearm=evf['rearm'])
np.savez('../results/scenario_log.npz', **{k: v for k, v in Lc.items()}, r=ev['r'], zone=ev['zone'], hand=ev['hand'], rearm=ev['rearm'])
print(json.dumps(out, indent=1))
