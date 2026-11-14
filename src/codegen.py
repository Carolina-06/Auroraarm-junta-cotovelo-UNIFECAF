"""Gerador automatico de codigo C (float32, C99, sem libm, sem malloc) a partir de:
   (1) results/design.json  -> controlador de posicao (refmodel.step)
   (2) supervisor.SPEC      -> supervisor de seguranca
Saida: generated/aurora_ctrl.{h,c}, generated/aurora_sup.{h,c}.  NAO EDITAR OS ARQUIVOS GERADOS A MAO."""
import sys, json, hashlib, datetime, re; sys.path.insert(0, '.')
import numpy as np
import supervisor as S
from params import *
from design import Ad, Bd, IMAX_LIM

d = json.load(open('../results/design.json'))
K = d['K']; beta = d['beta']; cw = d['c']
def f(x): return ('%.9g' % np.float32(x)) + 'f' if ('.' in '%.9g' % np.float32(x) or 'e' in '%.9g' % np.float32(x)) else ('%.9g' % np.float32(x)) + '.0f'
model_hash = hashlib.sha256((json.dumps(d['K']) + repr(S.SPEC) + str(S.T_RES) + str(Ad[0,0]) + str(Bd[0,0])).encode()).hexdigest()[:16]
stamp = "gerado por src/codegen.py | hash do modelo: %s" % model_hash

TRACE_CTRL = """\
/* ---------------------------------------------------------------------------------------------
 * AuroraArm -- junta do cotovelo -- CONTROLADOR DE POSICAO (arquivo GERADO AUTOMATICAMENTE)
 * %s
 * Origem do modelo : src/design.py (LQR discreto, ZOH, Ts=2 ms) -> results/design.json -> src/refmodel.py
 * Bloco de rastreabilidade (requisito -> elemento do modelo -> trecho deste arquivo -> teste):
 *   REQ-CTRL-AA-001  erro nulo em regime (degrau)  : integrador z (aurora_ctrl_step, 'z = ...')   | T-CTRL-001
 *   REQ-CTRL-AA-002  sobressinal <= 8 %%            : ganhos K0..K3, estrutura I-P (CW=0)         | T-CTRL-002
 *   REQ-CTRL-AA-003  tempo de acomodacao <= 300 ms : polos de malha fechada (design.json)        | T-CTRL-003
 *   REQ-CTRL-AA-004  |V|<=48 V, |i|<=8 A          : saturacao de tensao + limitador de corrente | T-CTRL-004
 *   REQ-SAFE-AA-004  v_TCP <= 250 mm/s (zona)      : limitador de taxa da referencia (DMAX)       | T-SAFE-004
 *   REQ-SAFE-AA-001  torque zero (STO) quando drv_en = 0 : ramo 'if (!drv_en)'                    | T-SAFE-001
 * Aritmetica: float (IEEE-754 binario32). Orcamento por ciclo: estimado < 100 operacoes de ponto flutuante.
 * --------------------------------------------------------------------------------------------- */
""" % stamp

h = TRACE_CTRL + """#ifndef AURORA_CTRL_H
#define AURORA_CTRL_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
typedef struct { float z; float rprev; float rcomp; } aurora_ctrl_state_t;
typedef struct { float u; float u_cmd; } aurora_ctrl_out_t;
void aurora_ctrl_init(aurora_ctrl_state_t *s);
/* i [A], w [rad/s], th [rad] (eixo do motor), r [rad], collab (zona colaborativa ou reconvergencia), drv_en (0 = STO) */
void aurora_ctrl_step(aurora_ctrl_state_t *s, float i, float w, float th, float r,
                      uint8_t collab, uint8_t drv_en, aurora_ctrl_out_t *o);
#ifdef __cplusplus
}
#endif
#endif
"""
c = TRACE_CTRL + """#include "aurora_ctrl.h"
#define K0   %s
#define K1   %s
#define K2   %s
#define K3   %s
#define CW   %s   /* ponderacao da referencia no termo proporcional (0 = estrutura I-P) */
#define TS   %s   /* periodo de amostragem [s] */
#define A00  %s   /* Ad(1,1): discretizacao ZOH exata */
#define A01  %s   /* Ad(1,2) */
#define B0   %s   /* Bd(1)   */
#define VMAX %s   /* [V] */
#define IMAX %s   /* [A] limite interno do limitador preditivo (driver: 8 A) */
#define DMAX %s   /* [rad/ciclo] 233,9 mm/s (calibrado: pior caso de inversao de rampa => pico real <= 245 mm/s) */
#define CAW  %s   /* ganho de back-calculation = beta / (-K3) */

void aurora_ctrl_init(aurora_ctrl_state_t *s) { s->z = 0.0f; s->rprev = 0.0f; s->rcomp = 0.0f; }

void aurora_ctrl_step(aurora_ctrl_state_t *s, float i, float w, float th, float r,
                      uint8_t collab, uint8_t drv_en, aurora_ctrl_out_t *o)
{
    float rr, d, y, tt, u_cmd, t, t2, u_hi, u_lo, u, z;
    if (!drv_en) {                       /* STO: torque zero, integrador congelado, referencia segue a posicao */
        s->rprev = th; s->rcomp = 0.0f; o->u = 0.0f; o->u_cmd = 0.0f; return;
    }
    rr = r;
    if (collab) {                        /* limitador de taxa da referencia */
        d = r - s->rprev;
        if (d > DMAX) { d = DMAX; } else if (d < -DMAX) { d = -DMAX; }
        y = d - s->rcomp;                /* soma compensada (Kahan): requer compilar SEM -ffast-math */
        tt = s->rprev + y;
        s->rcomp = (tt - s->rprev) - y;
        rr = tt;
    } else {
        s->rcomp = 0.0f;
    }
    s->rprev = rr;
    u_cmd = -(K0 * i + K1 * w + K2 * (th - CW * rr) + K3 * s->z);
    t = A00 * i;
    t2 = A01 * w;
    u_hi = (IMAX - t - t2) / B0;         /* limitador de corrente preditivo: |i[k+1]| <= IMAX */
    u_lo = (-IMAX - t - t2) / B0;
    if (u_hi > VMAX) { u_hi = VMAX; }    /* saturacao de tensao */
    if (u_lo < -VMAX) { u_lo = -VMAX; }
    u = u_cmd;
    if (u > u_hi) { u = u_hi; }
    if (u < u_lo) { u = u_lo; }
    z = s->z + TS * (rr - th);
    z = z + (u - u_cmd) * CAW;           /* anti-windup por back-calculation */
    s->z = z;
    o->u = u; o->u_cmd = u_cmd;
}
""" % tuple(f(x) for x in [K[0], K[1], K[2], K[3], cw, TS, Ad[0,0], Ad[0,1], Bd[0,0], VMAX, IMAX_LIM, V_TCP_DESIGN/R_TCP*N*TS, beta/(-K[3])])

# ------------------------------- supervisor -------------------------------
def cexpr(g): return g.replace(' and ', ' && ').replace(' or ', ' || ').replace('not ', '!').replace('T_RES', 'AURORA_T_RES')
TRACE_SUP = """\
/* ---------------------------------------------------------------------------------------------
 * AuroraArm -- SUPERVISOR DE SEGURANCA (arquivo GERADO AUTOMATICAMENTE)
 * %s
 * Origem: src/supervisor.py (SPEC) -- o mesmo modelo verificado por src/modelcheck.py
 * Rastreabilidade:
 *   REQ-SAFE-AA-001  MOVING/RESUMING --sensed--> HALT (drv=0)           | P1 (modelcheck) | T-SAFE-001
 *   REQ-SAFE-AA-002  HALT sai somente com borda de rearme e sensed=0    | P2 (modelcheck) | T-SAFE-002
 *   REQ-SAFE-AA-003  HALT -> RESUMING -> MOVING (trajetoria original)   | P3 (modelcheck) | T-SAFE-003
 * Chamada: 1 vez por ciclo de 2 ms, apos a leitura do sensor de proximidade e do botao de rearme.
 * --------------------------------------------------------------------------------------------- */
""" % stamp
hs = TRACE_SUP + """#ifndef AURORA_SUP_H
#define AURORA_SUP_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
#define AURORA_T_RES %d
enum { SUP_MOVING = 0, SUP_HALT = 1, SUP_RESUMING = 2 };
typedef struct { uint8_t mode; uint8_t cnt; uint8_t prev_sensed; uint8_t prev_rearm; uint8_t drv; } aurora_sup_t;
void aurora_sup_init(aurora_sup_t *s);
void aurora_sup_step(aurora_sup_t *s, uint8_t sensed, uint8_t rearm);
#ifdef __cplusplus
}
#endif
#endif
""" % S.T_RES
body = []
for mode in (S.MOVING, S.HALT, S.RESUMING):
    rows = [r for r in S.SPEC if r[0] == mode]
    nm = {S.MOVING: 'SUP_MOVING', S.HALT: 'SUP_HALT', S.RESUMING: 'SUP_RESUMING'}[mode]
    code = "    case %s:\n" % nm
    for k, (_, g, dst, act) in enumerate(rows):
        code += "        %sif (%s) {\n" % ('' if k == 0 else 'else ', cexpr(g))
        code += "            s->mode = %s; s->cnt = %d; s->drv = %d;\n        }\n" % ({S.MOVING: 'SUP_MOVING', S.HALT: 'SUP_HALT', S.RESUMING: 'SUP_RESUMING'}[dst], act.get('cnt', 0), act['drv'])
    code += "        else {\n" + ("            s->cnt++;\n" if mode == S.RESUMING else "            /* mantem */\n") + "        }\n        break;\n"
    body.append(code)
cs = TRACE_SUP + """#include "aurora_sup.h"
void aurora_sup_init(aurora_sup_t *s) { s->mode = SUP_MOVING; s->cnt = 0; s->prev_sensed = 0; s->prev_rearm = 0; s->drv = 1; }
void aurora_sup_step(aurora_sup_t *s, uint8_t sensed, uint8_t rearm)
{
    uint8_t rearm_edge = (uint8_t)(rearm && !s->prev_rearm);
    uint8_t prev_sensed = s->prev_sensed; (void)prev_sensed;
    uint8_t cnt = s->cnt; (void)cnt;
    switch (s->mode) {
""" + "".join(body) + """    default: s->mode = SUP_HALT; s->drv = 0; s->cnt = 0; break;   /* estado invalido -> estado seguro */
    }
    s->prev_sensed = (uint8_t)(sensed != 0); s->prev_rearm = (uint8_t)(rearm != 0);
}
"""
cs = cs.replace("s->cnt = 0; s->drv = 0;\n        }\n        else {\n            /* mantem */", "s->cnt = 0; s->drv = 0;\n        }\n        else {\n            /* mantem */")
for fn, txt in [('aurora_ctrl.h', h), ('aurora_ctrl.c', c), ('aurora_sup.h', hs), ('aurora_sup.c', cs)]:
    open('../generated/' + fn, 'w').write(txt)
print('ok', model_hash)
