"""Item (d) do memorial: varredura exaustiva atraso do sensor x fase do ciclo x ciclos perdidos.
(1) simulacao de eventos discretos com o proprio supervisor.step;  (2) model checker com D e MAXSKIP variaveis."""
import sys, json; sys.path.insert(0, '.')
import supervisor as S
from modelcheck import check
import numpy as np
TS_MS = 2

def latency(D_ms, phase, nlost):
    """Ticks (1 ms) entre a mao entrar na zona (t0) e o torque efetivo = 0. Sensor reporta em t0+D.
    Ciclos do supervisor em ticks multiplos de 2 (deslocados por 'phase'); os 'nlost' primeiros ciclos
    em que a deteccao poderia ser vista sao perdidos (pior caso). Comando e aplicado no inicio do ciclo seguinte."""
    t0 = 1000 + phase                       # t0 com fase arbitraria em relacao ao ciclo
    sup = S.INIT; drv_act = 1; lost = 0
    for t in range(t0 - 40, t0 + 400):
        if t % TS_MS == 0:
            drv_act = sup[4]                # latch do atuador (comando do ciclo anterior)
            sensed = 1 if t >= t0 + D_ms else 0
            if sensed and lost < nlost: lost += 1   # ciclo perdido por jitter
            else: sup, _ = S.step(sup, (sensed, 0))
        if t >= t0 and drv_act == 0: return t - t0
    return None

rows = []
for nlost in (0, 1):
    for phase in (0, 1):
        lat = [latency(D, phase, nlost) for D in range(0, 21)]
        rows.append(dict(nlost=nlost, phase=phase, lat=lat, worst=max(lat)))
worst = max(r['worst'] for r in rows)
out = dict(sweep=rows, worst_ms=worst, combos=len(rows) * 21, margin_ms=100 - worst)

# model checker: D x MAXSKIP
mc = []
for D in (0, 5, 10, 15, 20):
    for k in (0, 1):
        r = check(D=D, MAXSKIP=k, want_liveness=False); mc.append(dict(D=D, MAXSKIP=k, P1=r['P1_holds'], P2=r['P2_holds'], worst=r['worst_exposure_ms'] + 1, states=r['states']))
out['mc_grid'] = mc
# estresse: ate quantos ciclos perdidos CONSECUTIVOS (alem do pedido) o prazo de 100 ms vale?
stress = []
for k in (1, 2, 5, 10, 20, 30, 35, 36, 37, 38, 39, 40):
    r = check(D=20, MAXSKIP=k, want_liveness=False)
    stress.append(dict(MAXSKIP=k, P1=r['P1_holds'], worst=r['worst_exposure_ms'] + 1, states=r['states']))
out['stress_skips'] = stress
# estresse em D (sem jitter extra do enunciado, MAXSKIP=1)
sd = []
for D in (20, 40, 60, 70, 74, 75, 76, 80):
    r = check(D=D, MAXSKIP=1, want_liveness=False); sd.append(dict(D=D, P1=r['P1_holds'], worst=r['worst_exposure_ms'] + 1))
out['stress_D'] = sd
json.dump(out, open('../results/timing_sweep.json', 'w'), indent=1)
print('worst latency (sim):', worst, 'ms over', out['combos'], 'combos; margin', out['margin_ms'])
for r in rows: print(r['nlost'], r['phase'], r['lat'][0], r['lat'][10], r['lat'][20])
for m in mc: print(m)
for m in stress: print(m)
for m in sd: print(m)
