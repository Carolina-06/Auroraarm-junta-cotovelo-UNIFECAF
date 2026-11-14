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
#ifndef AURORA_CTRL_H
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
