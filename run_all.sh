#!/usr/bin/env bash
# Reproduz TODA a cadeia de evidencias (modelo -> controle -> verificacao -> codigo -> testes -> rastreabilidade).
set -euo pipefail
cd "$(dirname "$0")/src"
python3 plant.py            > /dev/null   # (a)(b)(c) matrizes, autovalores, posto, ponto de operacao
python3 run_design.py       > /dev/null   # controlador LQR + anti-windup; REQ-CTRL-AA-001..004
python3 calib_collab.py     > /dev/null   # limite de velocidade de projeto da zona colaborativa
python3 run_design.py       > /dev/null   # (reaplica params calibrados)
python3 modelcheck.py       > /dev/null   # P1/P2/P3 + contraexemplos dos mutantes
python3 timing_sweep.py     > /dev/null   # (d) varredura atraso x fase x jitter + estresse
python3 testgen.py          > /dev/null   # suite gerada do modelo + cobertura + mutacao
python3 codegen.py                        # codigo C gerado (controlador + supervisor)
(cd ../sil && python3 sil_test.py > /dev/null)   # (e) SIL: C gerado vs referencia
(cd ../tests && python3 run_all.py)       # testes por requisito
python3 make_trace.py       > /dev/null   # matriz de rastreabilidade
python3 make_figs.py
