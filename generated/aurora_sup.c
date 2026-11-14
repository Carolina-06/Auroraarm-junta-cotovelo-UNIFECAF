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
#include "aurora_sup.h"
void aurora_sup_init(aurora_sup_t *s) { s->mode = SUP_MOVING; s->cnt = 0; s->prev_sensed = 0; s->prev_rearm = 0; s->drv = 1; }
void aurora_sup_step(aurora_sup_t *s, uint8_t sensed, uint8_t rearm)
{
    uint8_t rearm_edge = (uint8_t)(rearm && !s->prev_rearm);
    uint8_t prev_sensed = s->prev_sensed; (void)prev_sensed;
    uint8_t cnt = s->cnt; (void)cnt;
    switch (s->mode) {
    case SUP_MOVING:
        if (sensed) {
            s->mode = SUP_HALT; s->cnt = 0; s->drv = 0;
        }
        else {
            /* mantem */
        }
        break;
    case SUP_HALT:
        if ((!sensed) && rearm_edge) {
            s->mode = SUP_RESUMING; s->cnt = 0; s->drv = 1;
        }
        else {
            /* mantem */
        }
        break;
    case SUP_RESUMING:
        if (sensed) {
            s->mode = SUP_HALT; s->cnt = 0; s->drv = 0;
        }
        else if (cnt >= AURORA_T_RES - 1) {
            s->mode = SUP_MOVING; s->cnt = 0; s->drv = 1;
        }
        else {
            s->cnt++;
        }
        break;
    default: s->mode = SUP_HALT; s->drv = 0; s->cnt = 0; break;   /* estado invalido -> estado seguro */
    }
    s->prev_sensed = (uint8_t)(sensed != 0); s->prev_rearm = (uint8_t)(rearm != 0);
}
