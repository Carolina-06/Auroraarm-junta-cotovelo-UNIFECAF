"""Modelo em espaco de estados da junta: x=[i, omega], u=V, y=omega (memorial a,b,c)."""
import numpy as np, json
from numpy.linalg import matrix_rank, eig
from params import *

def matrices():
    A = np.array([[-R/L, -KE/L],
                  [KT/J, -B/J]])
    Bm = np.array([[1/L],[0.0]])
    C = np.array([[0.0, 1.0]])
    D = np.array([[0.0]])
    return A, Bm, C, D

def analysis():
    A, Bm, C, D = matrices()
    ev = eig(A)[0]
    Ctrb = np.hstack([Bm, A@Bm])
    Obsv = np.vstack([C, C@A])
    iss = (B*W_SS + T_LOAD)/KT
    vss = R*iss + KE*W_SS
    return dict(A=A.tolist(), B=Bm.tolist(), C=C.tolist(), eig=sorted(ev.real.tolist()),
        tau_e_ref=L/R, tau_m_ref=J*R/(R*B+KT*KE),
        tau_fast=float(-1/min(ev.real)), tau_slow=float(-1/max(ev.real)),
        det_ctrb=float(np.linalg.det(Ctrb)), rank_ctrb=int(matrix_rank(Ctrb)),
        det_obsv=float(np.linalg.det(Obsv)), rank_obsv=int(matrix_rank(Obsv)),
        Ctrb=Ctrb.tolist(), Obsv=Obsv.tolist(),
        iss=iss, vss=vss, margin_v=VMAX-vss, margin_pct=100*(VMAX-vss)/VMAX,
        margin_i=IMAX-iss,
        omega_max_at_load=(VMAX - R*T_LOAD/KT)/(KE + R*B/KT),
        trace=float(np.trace(A)), det=float(np.linalg.det(A)),
        samples_per_tau_m=float(-1/max(ev.real))/TS,
        samples_per_tau_e=float(-1/min(ev.real))/TS,
        t_payload_joint=2*9.81*0.35, t_payload_reflected=2*9.81*0.35/N,
        omega_motor_collab_limit=V_TCP_MAX/R_TCP*N, v_tcp_at_450=W_SS/N*R_TCP)

if __name__ == "__main__":
    o = analysis()
    json.dump(o, open("../results/plant_analysis.json","w"), indent=1)
    for k,v in o.items(): print(k, v)
