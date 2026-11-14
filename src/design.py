"""Projeto do controlador de posicao: realimentacao de estados discreta + acao integral (LQR),
discretizacao ZOH exata, saturacao de tensao, limitador de corrente preditivo e anti-windup
por back-calculation. Todos os numeros do relatorio vem daqui (results/design.json)."""
import numpy as np, json
from scipy.linalg import expm, solve_discrete_are
from params import *

# ---- modelo continuo 3 estados x=[i, omega, theta] ----
A3 = np.array([[-R/L, -KE/L, 0],[KT/J, -B/J, 0],[0, 1, 0]])
B3 = np.array([[1/L],[0],[0]])
E3 = np.array([[0],[-1/J],[0]])        # entrada de disturbio: torque de carga

def zoh(A, Bm, Ts):
    n, m = Bm.shape
    M = np.zeros((n+m, n+m)); M[:n,:n] = A; M[:n,n:] = Bm
    Md = expm(M*Ts)
    return Md[:n,:n], Md[:n,n:]

Ad, Bd = zoh(A3, B3, TS)
_, Ed = zoh(A3, E3, TS)

def lqr_gains(q, rho, Ad=Ad, Bd=Bd, Ts=TS):
    """Estado aumentado xa=[i,w,th,z], z+ = z + Ts*(r-th). Retorna K (1x4)."""
    n = 3
    Aa = np.block([[Ad, np.zeros((n,1))],[np.array([[0,0,-Ts]]), np.eye(1)]])
    Ba = np.vstack([Bd, np.zeros((1,1))])
    Q = np.diag(q); Rm = np.array([[rho]])
    P = solve_discrete_are(Aa, Ba, Q, Rm)
    K = np.linalg.solve(Rm + Ba.T@P@Ba, Ba.T@P@Aa)
    return K, Aa, Ba

# Lei de controle:  u = -Ki*i - Kw*w - Kth*(th - c*r) - Kz*z ,  z+ = z + Ts*(r - th)
def make_ctrl(K, beta=1.0, c=0.0, vdesign=None):
    """c = ponderacao da referencia no termo proporcional de posicao (c=0: estrutura I-P)."""
    return dict(K=np.array(K).ravel(), beta=beta, c=c, vdesign=vdesign)

# Matriz do plano de simulacao em passo fino (dt=TS/20) com atuacao ZOH
NSUB = 20
IMAX_LIM = 7.5   # [A] limite interno do limitador preditivo (margem de 0,5 A p/ pico entre amostras)
Af, Bf = zoh(A3, B3, TS/NSUB)
_, Ef = zoh(A3, E3, TS/NSUB)

import refmodel
_cc = {}
def ctrl_step(ctrl, st, meas, r, use_aw=True, collab=False, drv_en=True):
    """Um ciclo do controlador (REFERENCIA float64) -- delega a refmodel.step (mesma aritmetica do C gerado)."""
    vd = ctrl.get('vdesign') or V_TCP_DESIGN
    key = (tuple(ctrl['K']), ctrl['beta'], ctrl['c'], vd)
    if key not in _cc:
        _cc[key] = refmodel.consts(ctrl['K'], ctrl['beta'], ctrl['c'], Ad[0,0], Ad[0,1], Bd[0,0], TS, VMAX,
                                   IMAX_LIM, vd/R_TCP*N*TS, np.float64)
    i, w, th = meas
    return refmodel.step(_cc[key], st, i, w, th, r, collab, drv_en, use_aw)

def simulate(ctrl, r_fun, T=1.0, use_aw=True, tload=0.0, collab=False, drop_prob=0.0, seed=0,
             tload_t=None, x0=None):
    rng = np.random.default_rng(seed)
    n = int(round(T/TS))
    x = np.zeros(3) if x0 is None else np.array(x0, float)
    st = dict(z=np.float64(0.0), rprev=np.float64(0.0), rcomp=np.float64(0.0))
    log = dict(t=[], th=[], w=[], i=[], u=[], ucmd=[], r=[], imax_fine=0.0, tl=[])
    u_prev = 0.0
    for k in range(n):
        r = r_fun(k*TS)
        u, ucmd = ctrl_step(ctrl, st, x, r, use_aw=use_aw, collab=collab)
        if drop_prob and rng.random() < drop_prob:
            u = u_prev          # ciclo perdido por jitter: atuador mantem comando anterior
        u_prev = u
        tl = tload if (tload_t is None or k*TS >= tload_t) else 0.0
        imaxf = 0
        for _ in range(NSUB):
            x = Af@x + Bf[:,0]*u + Ef[:,0]*tl
            imaxf = max(imaxf, abs(x[0]))
        log['imax_fine'] = max(log['imax_fine'], imaxf)
        for key, val in zip(['t','th','w','i','u','ucmd','r','tl'], [k*TS, x[2], x[1], x[0], u, ucmd, r, tl]):
            log[key].append(val)
    return {k: (np.array(v) if isinstance(v, list) else v) for k, v in log.items()}

def metrics(log, step):
    th = log['th']; t = log['t']
    ss = log['r'][-1]
    os_ = max(0.0, (th.max() - ss)/step*100)
    band = 0.02*step
    out = np.where(np.abs(th - ss) > band)[0]
    ts = (t[out[-1]+1] if out[-1]+1 < len(t) else float('inf')) if len(out) else 0.0
    return dict(overshoot_pct=float(os_), settling_s=float(ts), ess=float(abs(th[-1]-ss)),
                umax=float(np.abs(log['u']).max()), imax=float(np.abs(log['i']).max()),
                imax_fine=float(log['imax_fine']))
