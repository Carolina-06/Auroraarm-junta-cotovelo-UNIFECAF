# -*- coding: utf-8 -*-
"""Demonstracao para o video: roda AO VIVO o model checker, os mutantes com contraexemplo e a varredura de prazo,
e mostra os resultados ja gravados (testes, cobertura, SIL). So usa a biblioteca padrao do Python (sem pip).
Uso (na raiz do repositorio):  python src/demo_video.py          (pausa entre secoes: ENTER)
                               python src/demo_video.py --sem-pausa"""
import sys, os, json
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
try: sys.stdout.reconfigure(encoding='utf-8', errors='replace')
except Exception: pass
import supervisor as S
from modelcheck import check
RES = os.path.join(HERE, '..', 'results')
PAUSA = '--sem-pausa' not in sys.argv
def banner(t):
    print('\n' + '=' * 78 + '\n  ' + t + '\n' + '=' * 78)
def pausa():
    if PAUSA: input('\n[ENTER para continuar] ')
def load(f): return json.load(open(os.path.join(RES, f), encoding='utf-8'))

banner('1) VERIFICACAO EXAUSTIVA DO SUPERVISOR FINAL (atraso do sensor 0-20 ms, ate 1 ciclo perdido)')
r = check(D=20, MAXSKIP=1)
print(f"Estados alcancaveis explorados : {r['states']}")
print(f"Transicoes exploradas          : {r['transitions']}")
print(f"P1 REQ-SAFE-AA-001 (torque zero em <= 100 ms) : {'VERDADEIRA' if r['P1_holds'] else 'FALSA'}")
print(f"   pior caso: torque zero em {r['worst_exposure_ms'] + 1} ms  (folga de {100 - r['worst_exposure_ms'] - 1} ms)")
print(f"P2 REQ-SAFE-AA-002 (sem retomada automatica)  : {'VERDADEIRA' if r['P2_holds'] else 'FALSA'}")
print(f"P3 REQ-SAFE-AA-003 (vivacidade: volta a MOVING): {'VERDADEIRA' if r['P3_holds'] else 'FALSA'}")
pausa()

banner('2) VERSAO PRELIMINAR COM DEFEITO ("borda"): O VERIFICADOR ACHA O CONTRAEXEMPLO')
m = check(D=20, MAXSKIP=1, mutant='edge', want_liveness=False)
print(f"P1 (prazo de 100 ms) na versao defeituosa: {'VERDADEIRA' if m['P1_holds'] else 'FALSA  <-- violada!'}")
print(f"Contraexemplo com {len(m['cex_P1'])} ticks de 1 ms. Eventos principais:")
print("(coluna torque = torque EFETIVO: so muda no ciclo seguinte ao comando do supervisor)")
prev = None
for row in m['cex_P1']:
    key = (row['mode'], row['drv_act'], row['hand'], row['sensed'], row['rearm'])
    if key != prev:
        print(f"  t={row['tick']:>3} ms | modo={row['mode']:<8} | torque={'LIGADO ' if row['drv_act'] else 'desligado'} | mao={'PRESENTE' if row['hand'] else 'ausente '} | sensor={row['sensed']} | rearme={row['rearm']}")
        prev = key
last = m['cex_P1'][-1]
print(f"  ... no tick {last['tick']}: mao presente ha {last['t_stable']} ms e o torque continua LIGADO -> violou o prazo.")
l = check(D=20, MAXSKIP=1, mutant='lockout')
print(f"\nVersao defeituosa 'lockout': P3 (vivacidade) = {'VERDADEIRA' if l['P3_holds'] else 'FALSA  <-- ficou preso em LOCKOUT'}")
pausa()

banner('3) VARREDURA DO PRAZO DE 100 ms: atraso do sensor x fase do ciclo x ciclos perdidos')
def latency(D_ms, phase, nlost):
    t0 = 1000 + phase; sup = S.INIT; drv_act = 1; lost = 0
    for t in range(t0 - 40, t0 + 400):
        if t % 2 == 0:
            drv_act = sup[4]
            sensed = 1 if t >= t0 + D_ms else 0
            if sensed and lost < nlost: lost += 1
            else: sup, _ = S.step(sup, (sensed, 0))
        if t >= t0 and drv_act == 0: return t - t0
worst = 0; n = 0
for nlost in (0, 1):
    for phase in (0, 1):
        for D in range(0, 21):
            lat = latency(D, phase, nlost); worst = max(worst, lat); n += 1
print(f"Combinacoes testadas: {n}  (atraso 0..20 ms x 2 fases x 0 ou 1 ciclo perdido)")
print(f"Pior latencia ate o torque zerar: {worst} ms   |   prazo: 100 ms   |   folga: {100 - worst} ms")
print("Composicao do pior caso: 20 ms (sensor) + 1 ms (alinhamento) + 2 ms (ciclo perdido) + 2 ms (latch do atuador)")
t = load('timing_sweep.json')
ok = [s for s in t['stress_skips'] if s['P1']]; bad = [s for s in t['stress_skips'] if not s['P1']]
print(f"Estresse (resultado gravado): o prazo vale ate {max(s['MAXSKIP'] for s in ok)} ciclos perdidos consecutivos; "
      f"viola com {min(s['MAXSKIP'] for s in bad)} (o requisito pede apenas 1).")
pausa()

banner('4) TESTES GERADOS DO MODELO: COBERTURA')
c = load('coverage.json')
print(f"Casos de teste gerados : {c['n_tests']}  ({c['total_steps']} passos)")
print(f"Estados cobertos       : {c['nodes_cov']}/{c['nodes']}")
print(f"Transicoes cobertas    : {c['edges_cov']}/{c['edges']}")
print(f"Mutantes detectados    : {c['mutants_killed']}")
pausa()

banner('5) EQUIVALENCIA MODELO x CODIGO C GERADO (software-in-the-loop)')
s = load('sil.json'); cf = s['ctrl_feed']
print(f"Ciclos comparados                      : {cf['n']}")
print(f"Erro C gerado vs emulacao float32      : {cf['max_abs_err_vs_f32']}   (zero = identico bit a bit)")
print(f"Erro C gerado vs referencia float64    : {cf['max_abs_err_vs_f64'] * 1000:.2f} mV   (de uma saida de ate 48 V)")
print(f"Supervisor C vs modelo, divergencias   : {s['sup']['suite_mismatch'] + s['sup']['random_mismatch']} em {s['sup']['suite_steps'] + s['sup']['random_steps']} passos")
pausa()

banner('6) RESULTADO DE CADA TESTE POR REQUISITO')
tr = load('test_report.json')
for k, v in tr.items():
    print(f"  {'PASS' if v['passed'] else 'FAIL'}  {k:<12} {v['req']}")
print(f"\nTotal: {sum(v['passed'] for v in tr.values())}/{len(tr)} aprovados (T-SAFE-004N e um teste NEGATIVO: ele passa quando documenta a limitacao).")
