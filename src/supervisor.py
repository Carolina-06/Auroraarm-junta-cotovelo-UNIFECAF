"""Modelo do supervisor de seguranca (maquina de estados de ciclo discreto, 1 execucao a cada Ts=2 ms).
UNICA fonte de verdade: usada pelo model checker, pelo gerador de testes e pelo gerador de codigo C.

Estados : MOVING (torque habilitado, trajetoria original), HALT (STO = torque zerado, aguarda rearme),
          RESUMING (torque habilitado, reconvergindo a trajetoria original por T_RES ciclos).
Entradas: sensed (proximidade < 500 mm, ja com atraso do sensor), rearm (nivel do botao de rearme).
Memoria : prev_sensed, prev_rearm, cnt.
Saida   : drv (1 = torque habilitado).
Mutantes (versoes preliminares COM defeito, so para demonstrar contraexemplos):
  'edge'    : parada por BORDA de subida de sensed e RESUMING ignora sensed  (analogo ao quase-acidente)
  'lockout' : rearme com mao presente leva a LOCKOUT sem saida
"""
MOVING, HALT, RESUMING, LOCKOUT = 0, 1, 2, 3
NAMES = {MOVING: 'MOVING', HALT: 'HALT', RESUMING: 'RESUMING', LOCKOUT: 'LOCKOUT'}
T_RES = 20      # ciclos (40 ms) de reconvergencia a trajetoria original

# Especificacao declarativa: (estado_origem, guarda, estado_destino, acoes)  -- ordem = prioridade
# Guardas/acoes usam apenas: sensed, rearm_edge, prev_sensed, cnt, T_RES e operadores and/or/not.
SPEC = [
  (MOVING,   "sensed",                              HALT,     {"drv": 0, "cnt": 0}),
  (HALT,     "(not sensed) and rearm_edge",         RESUMING, {"drv": 1, "cnt": 0}),
  (RESUMING, "sensed",                              HALT,     {"drv": 0, "cnt": 0}),
  (RESUMING, "cnt >= T_RES - 1",                    MOVING,   {"drv": 1, "cnt": 0}),
]
SPEC_MUT = {
  'edge': [
    (MOVING,   "sensed and (not prev_sensed)",      HALT,     {"drv": 0, "cnt": 0}),
    (HALT,     "(not sensed) and rearm_edge",       RESUMING, {"drv": 1, "cnt": 0}),
    (RESUMING, "cnt >= T_RES - 1",                  MOVING,   {"drv": 1, "cnt": 0}),
  ],
  'lockout': SPEC[:1] + [
    (HALT,     "sensed and rearm_edge",             LOCKOUT,  {"drv": 0, "cnt": 0}),
    (HALT,     "(not sensed) and rearm_edge",       RESUMING, {"drv": 1, "cnt": 0}),
  ] + SPEC[2:],
}
_cache = {}
def _compile(spec):
    key = id(spec)
    if key not in _cache:
        _cache[key] = [(s, compile(g, '<g>', 'eval'), d, a) for s, g, d, a in spec]
    return _cache[key]

def step(state, inp, mutant=None):
    """state=(mode, cnt, prev_sensed, prev_rearm, drv); inp=(sensed, rearm). Retorna novo state."""
    mode, cnt, ps, pr, drv = state
    sensed, rearm = bool(inp[0]), bool(inp[1])
    env = dict(sensed=sensed, rearm_edge=(rearm and not pr), prev_sensed=bool(ps), cnt=cnt, T_RES=T_RES)
    spec = SPEC if mutant is None else SPEC_MUT[mutant]
    for s, g, d, a in _compile(spec):
        if s == mode and eval(g, {}, env):
            return (d, a.get("cnt", cnt), int(sensed), int(rearm), a.get("drv", drv)), True
    # nenhuma transicao: permanece (RESUMING incrementa contador); LOCKOUT/HALT/MOVING mantem drv
    if mode == RESUMING: cnt += 1
    return (mode, cnt, int(sensed), int(rearm), drv), False

INIT = (MOVING, 0, 0, 0, 1)
