/* ---------------------------------------------------------------------------------------------
 * AuroraArm -- SUPERVISOR DE SEGURANCA (arquivo GERADO AUTOMATICAMENTE)
 * gerado por src/codegen.py | hash do modelo: 06a1bde69251dd93
 * Origem: src/supervisor.py (SPEC) -- o mesmo modelo verificado por src/modelcheck.py
 * Rastreabilidade:
 *   REQ-SAFE-AA-001  MOVING/RESUMING --sensed--> HALT (drv=0)           | P1 (modelcheck) | T-SAFE-001
 *   REQ-SAFE-AA-002  HALT sai somente com borda de rearme e sensed=0    | P2 (modelcheck) | T-SAFE-002
 *   REQ-SAFE-AA-003  HALT -> RESUMING -> MOVING (trajetoria original)   | P3 (modelcheck) | T-SAFE-003
 * Chamada: 1 vez por ciclo de 2 ms, apos a leitura do sensor de proximidade e do botao de rearme.
 * --------------------------------------------------------------------------------------------- */
#ifndef AURORA_SUP_H
#define AURORA_SUP_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
#define AURORA_T_RES 20
enum { SUP_MOVING = 0, SUP_HALT = 1, SUP_RESUMING = 2 };
typedef struct { uint8_t mode; uint8_t cnt; uint8_t prev_sensed; uint8_t prev_rearm; uint8_t drv; } aurora_sup_t;
void aurora_sup_init(aurora_sup_t *s);
void aurora_sup_step(aurora_sup_t *s, uint8_t sensed, uint8_t rearm);
#ifdef __cplusplus
}
#endif
#endif
