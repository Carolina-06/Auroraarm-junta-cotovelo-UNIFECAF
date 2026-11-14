"""Calibra o limite de velocidade de projeto do limitador de referencia da zona colaborativa:
maior v_design tal que, no PIOR CASO de inversao do sentido da rampa (+v -> -v) em qualquer instante, a velocidade
real do efetuador v_TCP = |w| R_TCP / N permanece <= 0.98 * 250 mm/s (margem de 2% para numerica)."""
import sys, json; sys.path.insert(0, '.')
from design import *
K, _, _ = lqr_gains((0, 0, 1, 1000), 1e-2)
def worst_peak(vd):
    c = make_ctrl(K, 1.0, 0.0, vdesign=vd); peak = 0
    for trev in np.arange(0.04, 0.9, 0.02):
        for sign in (1, -1):
            r = lambda t, trev=trev, sign=sign: sign*(1e3 if t < trev else -1e3)   # alvo distante: rampa +v, inverte em trev
            lg = simulate(c, r, T=trev + 0.6, collab=True)
            peak = max(peak, np.abs(lg['w']).max()*R_TCP/N)
    return peak
LIM = 0.98*V_TCP_MAX
lo, hi = 0.05, V_TCP_MAX
for _ in range(14):
    mid = (lo + hi)/2
    if worst_peak(mid) <= LIM: lo = mid
    else: hi = mid
res = dict(v_design=lo, peak_at_design=worst_peak(lo), limit=LIM, peak_at_old_245=worst_peak(0.98*V_TCP_MAX), ratio_peak_over_design=worst_peak(lo)/lo)
json.dump(res, open('../results/collab_calib.json', 'w'), indent=1); print(res)
