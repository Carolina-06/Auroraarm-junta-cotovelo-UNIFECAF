# AuroraArm – junta do cotovelo: pacote de evidências MBD (modelo → controle → verificação formal → código → rastreabilidade)

Projeto da disciplina *Model Based Design for Cyber-Physical Systems* ("Operação Linha Viva").

## Reproduzir
```bash
pip install numpy scipy matplotlib   # + gcc para o SIL
./run_all.sh                         # ~1 min; imprime PASS/FAIL por requisito
```

## Estrutura
| Caminho | Conteúdo |
|---|---|
| `src/params.py` | parâmetros físicos e de segurança (enunciado) |
| `src/plant.py` | **A,B,C,D**, autovalores, posto de controlabilidade/observabilidade, ponto de operação (memorial a, b, c) |
| `src/design.py`, `src/refmodel.py`, `src/run_design.py` | discretização ZOH exata, LQR com ação integral, saturação, limitador de corrente, anti-windup (back-calculation) |
| `src/calib_collab.py` | calibra o limite de velocidade da zona colaborativa (pior caso de inversão de rampa) |
| `src/supervisor.py` | **modelo único** do supervisor de segurança (+ 2 mutantes defeituosos) |
| `src/modelcheck.py` | verificador exaustivo (P1 prazo de 100 ms, P2 sem retomada automática, P3 vivacidade) com contraexemplos |
| `src/timing_sweep.py` | memorial (d): varredura atraso × fase × ciclos perdidos + estresse |
| `src/testgen.py` | testes gerados por percurso do grafo, cobertura de estados/transições, análise de mutação |
| `src/codegen.py` | **gerador** de `generated/*.c|h` (não editar a mão) |
| `sil/` | software-in-the-loop: C compilado (gcc) comparado ao modelo (float64 e binary32) |
| `tests/run_all.py` | testes por requisito → `results/test_report.json` |
| `docs/traceability.md` | matriz requisito → modelo → código → teste |
| `results/` | JSONs e figuras usados no relatório |

## Limites declarados
Verificador próprio (explicit-state, Python), não UPPAAL/nuXmv; SIL em x86 (sem toolchain ARM nem HIL); sem qualificação de ferramenta (ISO 26262 / IEC 61508).
