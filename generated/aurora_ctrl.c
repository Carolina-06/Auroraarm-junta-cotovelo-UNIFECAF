/* ---------------------------------------------------------------------------------------------
 * AuroraArm -- junta do cotovelo -- CONTROLADOR DE POSICAO (arquivo GERADO AUTOMATICAMENTE)
 * gerado por src/codegen.py | hash do modelo: 06a1bde69251dd93
 * Origem do modelo : src/design.py (LQR discreto, ZOH, Ts=2 ms) -> results/design.json -> src/refmodel.py
 * Bloco de rastreabilidade (requisito -> elemento do modelo -> trecho deste arquivo -> teste):
 *   REQ-CTRL-AA-001  erro nulo em regime (degrau)  : integrador z (aurora_ctrl_step, 'z = ...')   | T-CTRL-001
 *   REQ-CTRL-AA-002  sobressinal <= 8 %            : ganhos K0..K3, estrutura I-P (CW=0)         | T-CTRL-002
 *   REQ-CTRL-AA-003  tempo de acomodacao <= 300 ms : polos de malha fechada (design.json)        | T-CTRL-003
 *   REQ-CTRL-AA-004  |V|<=48 V, |i|<=8 A          : saturacao de tensao + limitador de corrente | T-CTRL-004
 *   REQ-SAFE-AA-004  v_TCP <= 250 mm/s (zona)      : limitador de taxa da referencia (DMAX)       | T-SAFE-004
 *   REQ-SAFE-AA-001  torque zero (STO) quando drv_en = 0 : ramo 'if (!drv_en)'                    | T-SAFE-001
 * Aritmetica: float (IEEE-754 binario32). Orcamento por ciclo: estimado < 100 operacoes de ponto flutuante.
 * --------------------------------------------------------------------------------------------- */
#include "aurora_ctrl.h"
#define K0   0.0904731601f
#define K1   0.314872801f
#define K2   18.190155f
#define K3   -292.387207f
#define CW   0.0f   /* ponderacao da referencia no termo proporcional (0 = estrutura I-P) */
#define TS   0.00200000009f   /* periodo de amostragem [s] */
#define A00  0.251248449f   /* Ad(1,1): discretizacao ZOH exata */
#define A01  -0.0774846822f   /* Ad(1,2) */
#define B0   0.911982417f   /* Bd(1)   */
#define VMAX 48.0f   /* [V] */
#define IMAX 7.5f   /* [A] limite interno do limitador preditivo (driver: 8 A) */
#define DMAX 0.133657143f   /* [rad/ciclo] 233,9 mm/s (calibrado: pior caso de inversao de rampa => pico real <= 245 mm/s) */
#define CAW  0.0034201222f   /* ganho de back-calculation = beta / (-K3) */

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
