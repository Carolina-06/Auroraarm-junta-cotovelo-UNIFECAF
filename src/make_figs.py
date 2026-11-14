import sys, json; sys.path.insert(0, '.')
import numpy as np, matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch
plt.rcParams.update({'font.size': 9})
# --- automato ---
fig, ax = plt.subplots(figsize=(8, 3.6)); ax.axis('off'); ax.set_xlim(0, 10); ax.set_ylim(0, 4.4)
def box(x, y, t, c):
    ax.add_patch(FancyBboxPatch((x-1.0, y-0.45), 2.0, 0.9, boxstyle='round,pad=0.05', fc=c, ec='k')); ax.text(x, y, t, ha='center', va='center', fontsize=9, weight='bold')
box(1.5, 3.2, 'MOVING\ndrv=1', '#c8e6c9'); box(8.5, 3.2, 'HALT (STO)\ndrv=0', '#ffcdd2'); box(6.0, 0.9, 'RESUMING\ndrv=1, cnt<20', '#fff9c4')
def arr(p, q, txt, rad=0.0, tx=0, ty=0):
    ax.add_patch(FancyArrowPatch(p, q, arrowstyle='-|>', mutation_scale=12, connectionstyle=f'arc3,rad={rad}', color='k')); ax.text((p[0]+q[0])/2+tx, (p[1]+q[1])/2+ty, txt, ha='center', fontsize=8)
arr((2.55, 3.2), (7.45, 3.2), 'sensed', 0.0, 0, 0.18)
arr((8.6, 2.7), (6.9, 1.3), '¬sensed ∧ borda(rearme)', -0.1, 1.45, 0.25)
arr((5.0, 1.15), (2.0, 2.65), 'cnt ≥ 19', -0.1, -0.85, 0.05)
arr((7.1, 1.3), (9.0, 2.7), 'sensed', 0.35, 1.25, -0.45)
ax.text(5, 4.15, 'Supervisor de segurança (executa a cada Ts = 2 ms)', ha='center', weight='bold')
ax.text(0.1, 0.05, 'REQ-SAFE-001: MOVING/RESUMING --sensed--> HALT\nREQ-SAFE-002: HALT só sai com borda de\n   rearme e ¬sensed\nREQ-SAFE-003: HALT → RESUMING → MOVING', fontsize=7, va='bottom')
plt.tight_layout(); plt.savefig('../results/fig_automato.png', dpi=170); plt.close()
# --- timing sweep ---
t = json.load(open('../results/timing_sweep.json'))
fig, ax = plt.subplots(1, 2, figsize=(9, 3.2))
for r in t['sweep']: ax[0].plot(range(21), r['lat'], marker='o', ms=3, label=f"ciclos perdidos={r['nlost']}, fase={r['phase']} ms")
ax[0].axhline(100, color='r', ls=':'); ax[0].text(1, 92, 'prazo 100 ms', color='r'); ax[0].set_xlabel('atraso do sensor [ms]'); ax[0].set_ylabel('latência até torque zero [ms]'); ax[0].legend(fontsize=6.5); ax[0].set_ylim(0, 110); ax[0].grid(alpha=.3)
ks = [s['MAXSKIP'] for s in t['stress_skips']]; w = [s['worst'] for s in t['stress_skips']]
ax[1].plot(ks, w, 'o-'); ax[1].axhline(100, color='r', ls=':'); ax[1].set_xlabel('ciclos perdidos consecutivos admitidos'); ax[1].set_ylabel('pior latência [ms]'); ax[1].set_title('Estresse (model checker): P1 vale até 38; viola em 39', fontsize=8); ax[1].grid(alpha=.3)
plt.tight_layout(); plt.savefig('../results/fig_timing.png', dpi=170); plt.close()
# --- cenario SIL ---
L = np.load('../results/scenario_log.npz'); tt = np.arange(len(L['th']))*2e-3
fig, ax = plt.subplots(4, 1, figsize=(9, 7.2), sharex=True)
ax[0].plot(tt, L['th'], label='θ'); ax[0].plot(tt, L['r'], 'k--', lw=.7, label='ref'); ax[0].set_ylabel('θ [rad]'); ax[0].legend(fontsize=7)
ax[1].plot(tt, L['w']); ax[1].set_ylabel('ω [rad/s]')
ax[2].plot(tt, L['u'], label='u [V]'); ax[2].plot(tt, L['i'], label='i [A]'); ax[2].legend(fontsize=7)
ax[3].plot(tt, L['mode'], drawstyle='steps-post', label='modo (0=MOV,1=HALT,2=RES)'); ax[3].plot(tt, L['hand']*0.9+0.05, lw=.8, label='mão (verdade)'); ax[3].plot(tt, L['zone']*0.5-0.6, lw=.8, label='zona colab.'); ax[3].legend(fontsize=6.5, ncol=3); ax[3].set_xlabel('t [s]')
for a in ax: a.grid(alpha=.3)
plt.tight_layout(); plt.savefig('../results/fig_scenario.png', dpi=170); plt.close()
print('figs ok')
