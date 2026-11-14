"""Cenario integrado planta + supervisor + controlador (SIL/co-simulacao em Python). A planta e a MESMA
discretizacao fina usada no projeto; supervisor e controlador sao pluggaveis (referencia Python ou C gerado)."""
import sys; sys.path.insert(0, '../src')
import numpy as np
import design as D, supervisor as S
from params import *

def make_events(T=12.0, seed=3, variant='main'):
    """variant='main': entradas na zona com o braco em baixa velocidade (premissa de planejamento).
       variant='fast_entry': teste NEGATIVO -- entrada na zona ainda em movimento rapido (limite do limitador de referencia)."""
    rng = np.random.default_rng(seed); n = int(T/TS)
    r = np.zeros(n)
    steps = {'main':       [(0,0.5),(0.8,10),(1.6,-12),(2.4,3),(3.2,100),(4.4,40),(5.4,45),(6.0,50),(6.6,75),(7.2,35),(8.0,900),(11.0,0)],
             'fast_entry': [(0,0.5),(0.8,10),(1.6,-12),(2.4,3),(3.2,100),(4.4,-40),(5.8,8),(6.2,0),(6.6,25),(7.2,-5),(8.0,900),(11.0,0)]}[variant]
    for t, a in steps: r[int(t/TS):] = a      # (8.0 s: movimento grande -> cruza 450 rad/s e satura 48 V)
    def seg(*ts):
        m = np.zeros(n, bool)
        for t0, t1 in ts: m[int(t0/TS):int(t1/TS)] = True
        return m
    zone = seg((1.5, 2.5), (6.4, 7.4))
    hand = seg((2.0, 2.3), (5.2, 5.25), (6.5, 6.9), (9.0, 9.2))
    rearm = np.zeros(n, bool)
    for t0 in [2.6, 5.5, 7.1, 9.6]: rearm[int(t0/TS):int(t0/TS)+3] = True
    noise = rng.normal(0, 1e-4, (n, 3))
    return dict(r=r, zone=zone, hand=hand, rearm=rearm, noise=noise, n=n)

def run(ev, ctrl_fn, sup_fn, sensor_delay_cycles=10, lost_cycles=()):
    """ctrl_fn(i,w,th,r,collab,drv_en)->u ; sup_fn(sensed,rearm)->(mode,drv). Retorna logs."""
    n = ev['n']; x = np.zeros(3); u_prev = 0.0
    L = {k: np.zeros(n) for k in ['th','w','i','u','mode','drv','sensed']}
    inp = np.zeros((n, 6)); sensed_hist = [0]*sensor_delay_cycles
    for k in range(n):
        sensed_hist.append(int(ev['hand'][k])); sensed = sensed_hist.pop(0)
        mode, drv = sup_fn(sensed, int(ev['rearm'][k]))
        collab = bool(ev['zone'][k] or mode == S.RESUMING)
        meas = x + ev['noise'][k]*np.array([1,1,1])
        inp[k] = [meas[0], meas[1], meas[2], ev['r'][k], collab, drv]
        u = ctrl_fn(meas[0], meas[1], meas[2], ev['r'][k], collab, drv)
        if k in lost_cycles: u = u_prev
        u_prev = u
        for _ in range(D.NSUB):
            if drv: x = D.Af@x + D.Bf[:,0]*u
            else:   x = D.Af@np.array([0.0, x[1], x[2]]) + 0*D.Bf[:,0]   # STO: corrente = 0 (driver desabilitado)
        if not drv: x[0] = 0.0
        L['th'][k], L['w'][k], L['i'][k], L['u'][k], L['mode'][k], L['drv'][k], L['sensed'][k] = x[2], x[1], x[0], u, mode, drv, sensed
    return L, inp
