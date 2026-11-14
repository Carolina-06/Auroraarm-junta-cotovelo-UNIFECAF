"""Executa o projeto e a verificacao de desempenho (REQ-CTRL-AA-001..004). Gera results/design.json e figuras."""
import sys, json; sys.path.insert(0,'.')
import numpy as np, matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
from design import *

Q = (0,0,1,1000); RHO = 1e-2; BETA = 1.0; CW = 0.0
K, Aa, Ba = lqr_gains(Q, RHO)
ctrl = make_ctrl(K, BETA, CW)
res = dict(Q=Q, rho=RHO, beta=BETA, c=CW, K=K.ravel().tolist(), Ad=Ad.tolist(), Bd=Bd.ravel().tolist(),
           poles_z=[[z.real, z.imag] for z in np.linalg.eigvals(Aa-Ba@K)],
           poles_s=[[ (np.log(z)/TS).real, (np.log(z)/TS).imag] for z in np.linalg.eigvals(Aa-Ba@K)])

# --- degraus ---
STEPS = [0.1, 1, 5, 10, 20, 30, 40, 60, 100]
tab = []
for s in STEPS:
    for aw in (True, False):
        lg = simulate(ctrl, lambda t, s=s: s, T=2.0, use_aw=aw)
        m = metrics(lg, s); m.update(step=s, aw=aw); tab.append(m)
res['step_table'] = tab
# maior amplitude que ainda atende 002 e 003
ok = [s for s in np.arange(1, 60, 0.5)
      if (lambda m: m['overshoot_pct'] <= 8 and m['settling_s'] <= 0.3)(metrics(simulate(ctrl, lambda t, s=s: s, T=1.5), s))]
res['max_step_ok_rad_motor'] = float(max(ok)); res['max_step_ok_rad_joint'] = float(max(ok))/N
# criterio de projeto: passo nominal de verificacao = 10 rad no motor (0,1 rad = 5,7 graus na junta)
STEP_NOM = 10.0
lg = simulate(ctrl, lambda t: STEP_NOM, T=1.0); res['nominal'] = metrics(lg, STEP_NOM)
lg_lin = simulate(ctrl, lambda t: 0.1, T=1.0); res['linear'] = metrics(lg_lin, 0.1)
lg_sat = simulate(ctrl, lambda t: 100., T=2.0); lg_nosat = simulate(ctrl, lambda t: 100., T=2.0, use_aw=False)
# --- carga ---
lg_load = simulate(ctrl, lambda t: 0.0, T=1.0, tload=0.25, tload_t=0.2)
res['load_dist'] = dict(peak_dev_rad=float(np.abs(lg_load['th']).max()), final_err=float(abs(lg_load['th'][-1])),
                        z_final=float(0))
# --- ciclos perdidos (jitter) ---
dr = []
for p in (0.0, 0.05, 0.1, 0.2):
    ms = [metrics(simulate(ctrl, lambda t: STEP_NOM, T=1.0, drop_prob=p, seed=sd), STEP_NOM) for sd in range(20)]
    dr.append(dict(p=p, os_max=max(m['overshoot_pct'] for m in ms), ts_max=max(m['settling_s'] for m in ms),
                   imax_max=max(m['imax_fine'] for m in ms)))
res['drops'] = dr
# --- margem de atraso: estabilidade com d amostras adicionais de atraso ---
def stable_with_delay(d):
    """Raio espectral do laco com d amostras de atraso extra na atuacao (modelo de ciclos perdidos/jitter)."""
    if d == 0: return float(np.abs(np.linalg.eigvals(Aa - Ba@K)).max())
    m = 4 + d
    Ae = np.zeros((m, m)); Ae[:4, :4] = Aa
    Ae[:4, 4+d-1:4+d] = Ba            # a planta recebe o comando mais antigo da fila
    Ae[4, :4] = -K[0]                 # novo comando entra na fila
    for j in range(1, d): Ae[4+j, 4+j-1] = 1
    return float(np.abs(np.linalg.eigvals(Ae)).max())
res['delay_margin'] = {d: stable_with_delay(d) for d in range(0, 13)}
json.dump(res, open('../results/design.json', 'w'), indent=1, default=float)

# ---------------- figuras ----------------
plt.rcParams.update({'font.size': 9})
fig, ax = plt.subplots(2, 2, figsize=(9, 6))
ax[0,0].plot(lg_lin['t'], lg_lin['th'], label=r'$\theta$'); ax[0,0].plot(lg_lin['t'], lg_lin['r'], 'k--', lw=.8, label='ref')
ax[0,0].axvline(.3, color='r', ls=':'); ax[0,0].set_title('Degrau linear 0,1 rad (sem saturação)')
ax[0,1].plot(lg['t'], lg['th'], label=r'$\theta$'); ax[0,1].plot(lg['t'], lg['r'], 'k--', lw=.8)
ax[0,1].axvline(.3, color='r', ls=':'); ax[0,1].set_title('Degrau nominal 10 rad (corrente limitada)')
ax[1,0].plot(lg['t'], lg['u'], label='u [V]'); ax[1,0].plot(lg['t'], lg['i'], label='i [A]'); ax[1,0].axhline(8, color='r', ls=':'); ax[1,0].legend(); ax[1,0].set_title('Tensão e corrente (10 rad)')
ax[1,1].plot(lg_load['t'], lg_load['th'], label=r'$\theta$ - ref'); ax[1,1].set_title('Rejeição de degrau de carga 0,25 N·m em t=0,2 s')
for a in ax.ravel(): a.set_xlabel('t [s]'); a.grid(alpha=.3)
ax[0,0].legend(); plt.tight_layout(); plt.savefig('../results/fig_step.png', dpi=160); plt.close()

fig, ax = plt.subplots(1, 3, figsize=(10, 3.2))
ax[0].plot(lg_sat['t'], lg_sat['th'], label='com anti-windup'); ax[0].plot(lg_nosat['t'], lg_nosat['th'], label='sem anti-windup'); ax[0].axhline(100, color='k', ls='--', lw=.7); ax[0].legend(); ax[0].set_title(r'$\theta$ (degrau 100 rad)')
ax[1].plot(lg_sat['t'], lg_sat['u'], label='com AW'); ax[1].plot(lg_nosat['t'], lg_nosat['u'], label='sem AW'); ax[1].set_title('u [V] (saturado em ±48 V)'); ax[1].legend()
ax[2].plot(lg_sat['t'], lg_sat['i'], label='com AW'); ax[2].plot(lg_nosat['t'], lg_nosat['i'], label='sem AW'); ax[2].set_title('i [A] (limite 8 A)'); ax[2].legend()
for a in ax: a.set_xlabel('t [s]'); a.grid(alpha=.3)
plt.tight_layout(); plt.savefig('../results/fig_windup.png', dpi=160); plt.close()

# collab: limite de velocidade
lgc = simulate(ctrl, lambda t: 40.0, T=1.0, collab=True)
res['collab'] = dict(wmax=float(np.abs(lgc['w']).max()), wlim=V_TCP_MAX/R_TCP*N, vtcp_max=float(np.abs(lgc['w']).max()/N*R_TCP))
json.dump(res, open('../results/design.json', 'w'), indent=1, default=float)
plt.figure(figsize=(4.5, 3)); plt.plot(lgc['t'], np.abs(lgc['w'])/N*R_TCP*1000); plt.axhline(250, color='r', ls=':'); plt.xlabel('t [s]'); plt.ylabel('v_TCP [mm/s]'); plt.title('Zona colaborativa: |v| ≤ 250 mm/s'); plt.grid(alpha=.3); plt.tight_layout(); plt.savefig('../results/fig_collab.png', dpi=160)
print(json.dumps({k: res[k] for k in ['K','poles_s','nominal','linear','max_step_ok_rad_motor','load_dist','drops','delay_margin','collab']}, indent=1, default=float))
print([ (t['step'], t['aw'], round(t['overshoot_pct'],2), t['settling_s']) for t in tab])
