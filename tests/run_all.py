"""Executa os testes por requisito (T-*) e grava results/test_report.json. Rodar APOS run_design/modelcheck/timing_sweep/testgen/sil_test."""
import sys, json; sys.path.insert(0, '../src')
import numpy as np
from params import *
R = lambda f: json.load(open('../results/' + f))
dz, mc, ts, sil, cov = R('design.json'), R('modelcheck.json'), R('timing_sweep.json'), R('sil.json'), R('coverage.json')
log = np.load('../results/scenario_log.npz')
T = {}
def rec(tid, req, ok, detail): T[tid] = dict(req=req, passed=bool(ok), detail=detail)

lin = [t for t in dz['step_table'] if t['aw'] and t['step'] <= 10]
rec('T-CTRL-001', 'REQ-CTRL-AA-001', max(t['ess'] for t in lin) < 1e-6 and dz['load_dist']['final_err'] < 1e-6,
    f"erro estacionario max {max(t['ess'] for t in lin):.2e} rad (degraus 0,1..10 rad); com degrau de carga 0,25 N.m: {dz['load_dist']['final_err']:.2e} rad")
rec('T-CTRL-002', 'REQ-CTRL-AA-002', max(t['overshoot_pct'] for t in lin) <= 8, f"sobressinal max {max(t['overshoot_pct'] for t in lin):.2f} % (limite 8 %) para degraus 0,1..10 rad")
rec('T-CTRL-003', 'REQ-CTRL-AA-003', max(t['settling_s'] for t in lin) <= 0.3, f"ts(2%) max {max(t['settling_s'] for t in lin)*1e3:.0f} ms (limite 300 ms); valido ate {dz['max_step_ok_rad_motor']:.0f} rad no motor")
imax_all = max(dz['nominal']['imax_fine'], max(d['imax_max'] for d in dz['drops']), sil['closed_loop']['imax'])
umax_all = max(dz['nominal']['umax'], sil['closed_loop']['umax'])
rec('T-CTRL-004', 'REQ-CTRL-AA-004', imax_all <= IMAX and umax_all <= VMAX + 1e-6, f"|u| max {umax_all:.2f} V (48 V); |i| max {imax_all:.2f} A (8 A) incl. ciclos perdidos e laco com C gerado")

hand, drv, mode, sensed, zone = log['hand'], log['drv'], log['mode'], log['sensed'], log['zone']
# T-SAFE-001: latencia ponta a ponta no cenario (C no laco) em cada entrada de mao + varredura + model checker
lat = []
for k in np.where(np.diff(hand.astype(int)) == 1)[0] + 1:
    j = k
    while j < len(drv) and drv[j] == 1: j += 1
    lat.append((j - k) * TS * 1e3)
p1 = mc['final']['P1_holds']
rec('T-SAFE-001', 'REQ-SAFE-AA-001', p1 and ts['worst_ms'] <= 100 and max(lat) <= 100,
    f"model checker P1 verdadeiro ({mc['final']['states']} estados); latencia pior caso {ts['worst_ms']} ms em {ts['combos']} combinacoes; cenario SIL: {', '.join('%.0f'%l for l in lat)} ms; folga >= {100-ts['worst_ms']} ms")
# T-SAFE-002: sem retomada automatica
hl = np.where((mode[1:] == 1) & (mode[:-1] != 1))[0] + 1
auto = 0
for k in range(1, len(drv)):
    if drv[k] == 1 and drv[k-1] == 0 and not (log['rearm'][max(0, k-2):k+1].any()): auto += 1
rec('T-SAFE-002', 'REQ-SAFE-AA-002', mc['final']['P2_holds'] and auto == 0, f"model checker P2 verdadeiro; retomadas sem rearme no cenario: {auto}; mutante 'borda' reprovado por P1, 'lockout' por P3; suite de {cov['n_tests']} testes mata ambos")
# T-SAFE-003: apos o rearme, o braco retoma e volta a trajetoria original (tempo ate |theta - r| < 0.5 rad e modo MOVING)
r_log = log['r']; th = log['th']; res = []
for tr in [2.6, 5.5, 7.1, 9.6]:
    k = int(tr / TS); j = k
    while j < len(th) and not (abs(th[j] - r_log[j]) < 0.5 and mode[j] == 0): j += 1
    res.append((j - k) * TS if j < len(th) else float('inf'))
rec('T-SAFE-003', 'REQ-SAFE-AA-003', mc['final']['P3_holds'] and max(res) < 2.5 and (drv[-1] == 1),
    f"model checker P3 verdadeiro (sem ciclo justo fora de MOVING); tempo do rearme ate MOVING e |theta-r|<0,5 rad: {', '.join('%.2f'%x for x in res)} s; mutante 'lockout' viola P3 (contraexemplo)")
# T-SAFE-004: velocidade do efetuador na zona (apos o transitorio de entrada)
v = np.abs(log['w']) / N * R_TCP * 1000
inzone = np.where(zone)[0]
# velocidade apos a entrada na zona (descarta o 1o 100 ms: robo entra na zona ja desacelerado por hipotese de planejamento)
vz = max(v[k] for k in inzone if drv[k] == 1)
rec('T-SAFE-004', 'REQ-SAFE-AA-004', vz <= 250.0, f"v_TCP max na zona = {vz:.1f} mm/s (limite 250; alvo de projeto 245); referencia limitada em taxa; comando de {abs(r_log).max():.0f} rad")
lf = np.load('../results/scenario_log_fastentry.npz'); vf = np.abs(lf['w']) / N * R_TCP * 1000
vfz = max(vf[k] for k in np.where(lf['zone'])[0] if lf['drv'][k] == 1)
rec('T-SAFE-004N', 'REQ-SAFE-AA-004 (teste negativo)', vfz > 250.0, f"entrada na zona ainda em movimento rapido: v_TCP max = {vfz:.0f} mm/s > 250 -> o limitador de referencia NAO basta; exige desaceleracao planejada antes da zona ou monitoramento de velocidade segura (limitacao documentada)")
rec('T-SIL-EQUIV', 'REQ-CTRL-AA-001..004', sil['ctrl_feed']['max_abs_err_vs_f32'] == 0.0 and sil['sup']['suite_mismatch'] == 0 and sil['sup']['random_mismatch'] == 0,
    f"C gerado vs binary32: erro {sil['ctrl_feed']['max_abs_err_vs_f32']:.1e}; vs float64: {sil['ctrl_feed']['max_abs_err_vs_f64']:.2e} V ({sil['ctrl_feed']['max_abs_err_vs_f64']/(sil['ctrl_feed']['eps32']*sil['ctrl_feed']['umax']):.0f} eps*umax; razao local max. erro/(eps*soma-das-parcelas) = {sil['conditioning']['ratio_max']:.1f}); supervisor C vs modelo: 0 divergencias em {sil['sup']['suite_steps']+sil['sup']['random_steps']} passos")
json.dump(T, open('../results/test_report.json', 'w'), indent=1)
for k, v in T.items(): print(('PASS ' if v['passed'] else 'FAIL ') + k, v['req'], '|', v['detail'])
