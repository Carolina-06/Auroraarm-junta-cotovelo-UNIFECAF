"""Modelo de REFERENCIA do controlador (um ciclo). A mesma aritmetica, na mesma ordem de operacoes, e
emitida como C pelo gerador (codegen.py). Parametrizado por dtype: float64 (referencia ideal) ou float32
(emulacao bit-a-bit do tipo usado no microcontrolador)."""
import numpy as np

def consts(K, beta, c, Ad00, Ad01, Bd0, ts, vmax, imax_lim, dmax_collab, dt):
    f = dt
    return dict(K0=f(K[0]), K1=f(K[1]), K2=f(K[2]), K3=f(K[3]), CW=f(c), TS=f(ts),
                A00=f(Ad00), A01=f(Ad01), B0=f(Bd0), VMAX=f(vmax), IMAX=f(imax_lim),
                DMAX=f(dmax_collab), CAW=f(beta / (-K[3])), dt=dt)

def step(c, st, i, w, th, r, collab, drv_en, use_aw=True):
    """st = dict(z=..., rprev=..., rcomp=...). Retorna (u, u_cmd) e atualiza st in-place."""
    dt = c['dt']; i, w, th, r = dt(i), dt(w), dt(th), dt(r)
    if not drv_en:                       # HALT (STO): torque zero, integrador congelado, referencia segue a posicao
        st['rprev'] = th; st['rcomp'] = dt(0.0)
        return dt(0.0), dt(0.0)
    rr = r
    if collab:                           # limitador de taxa da referencia (REQ-SAFE-AA-004 / reconvergencia)
        d = r - st['rprev']
        if d > c['DMAX']: d = c['DMAX']
        elif d < -c['DMAX']: d = -c['DMAX']
        y = d - st['rcomp']              # soma compensada (Kahan): elimina o vies de arredondamento do acumulador
        tt = st['rprev'] + y
        st['rcomp'] = (tt - st['rprev']) - y
        rr = tt
    else:
        st['rcomp'] = dt(0.0)
    st['rprev'] = rr
    u_cmd = -(c['K0']*i + c['K1']*w + c['K2']*(th - c['CW']*rr) + c['K3']*st['z'])
    t = c['A00']*i
    t2 = c['A01']*w
    u_hi = (c['IMAX'] - t - t2) / c['B0']
    u_lo = (-c['IMAX'] - t - t2) / c['B0']
    if u_hi > c['VMAX']: u_hi = c['VMAX']
    if u_lo < -c['VMAX']: u_lo = -c['VMAX']
    u = u_cmd
    if u > u_hi: u = u_hi
    if u < u_lo: u = u_lo
    z = st['z'] + c['TS']*(rr - th)
    if use_aw: z = z + (u - u_cmd)*c['CAW']
    st['z'] = z
    return u, u_cmd
