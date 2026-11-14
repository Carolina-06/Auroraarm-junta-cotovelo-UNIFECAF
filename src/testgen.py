"""Geracao de testes a partir do modelo do supervisor (model-based testing).
Grafo abstrato: nos = (mode, cnt, prev_rearm); arestas = 4 combinacoes de entrada (sensed, rearm).
Algoritmo: cobertura de TODAS as arestas por percursos de caminho mais curto (BFS) a partir do estado inicial;
sequencias quebradas em casos de teste curtos (reset ao estado inicial).  Saida esperada vem do modelo de referencia.
Tambem: cobertura de estados/transicoes, cobertura das linhas da SPEC e analise de mutacao."""
import sys, json, itertools; sys.path.insert(0, '.')
from collections import deque
import supervisor as S

INPUTS = [(0, 0), (0, 1), (1, 0), (1, 1)]
def key(s): return (s[0], s[1], s[3])        # abstracao: (mode, cnt, prev_rearm)

def build_graph(mutant=None):
    init = S.INIT; nodes = {key(init): init}; edges = {}; q = deque([init])
    while q:
        s = q.popleft()
        for inp in INPUTS:
            s2, _ = S.step(s, inp, mutant)
            edges[(key(s), inp)] = key(s2)
            if key(s2) not in nodes: nodes[key(s2)] = s2; q.append(s2)
    return nodes, edges

def gen_tests(max_len=45, mutant=None):
    nodes, edges = build_graph(mutant)
    uncovered = set(edges.keys()); tests = []; cur = []; state = S.INIT
    def path_to_uncovered(s):
        seen = {key(s): None}; q = deque([(s, [])])
        while q:
            st, path = q.popleft()
            for inp in INPUTS:
                if (key(st), inp) in uncovered: return path + [inp]
            for inp in INPUTS:
                s2, _ = S.step(st, inp, mutant)
                if key(s2) not in seen: seen[key(s2)] = 1; q.append((s2, path + [inp]))
        return None
    while uncovered:
        p = path_to_uncovered(state)
        if p is None:
            tests.append(cur); cur = []; state = S.INIT; continue
        for inp in p:
            uncovered.discard((key(state), inp)); state, _ = S.step(state, inp, mutant); cur.append(inp)
        if len(cur) >= max_len:
            tests.append(cur); cur = []; state = S.INIT
    if cur: tests.append(cur)
    return tests, nodes, edges

def run(seq, mutant=None):
    s = S.INIT; out = []
    for inp in seq:
        s, fired = S.step(s, inp, mutant); out.append((s[0], s[4], s[1]))
    return out

def coverage(tests):
    nodes, edges = build_graph()
    vis_n = set(); vis_e = set(); spec_rows = set()
    for t in tests:
        s = S.INIT; vis_n.add(key(s))
        for inp in t:
            vis_e.add((key(s), inp)); s2, fired = S.step(s, inp)
            # qual linha da SPEC disparou
            env = dict(sensed=bool(inp[0]), rearm_edge=bool(inp[1] and not s[3]), prev_sensed=bool(s[2]), cnt=s[1], T_RES=S.T_RES)
            hit = None
            for idx, (m, g, d, a) in enumerate(S.SPEC):
                if m == s[0] and eval(g, {}, env): hit = idx; break
            spec_rows.add(hit if hit is not None else ('stay', s[0]))
            s = s2; vis_n.add(key(s))
    return dict(nodes=len(nodes), nodes_cov=len(vis_n & set(nodes)), edges=len(edges), edges_cov=len(vis_e & set(edges)),
                mode_cov=sorted({S.NAMES[k[0]] for k in vis_n}), spec_rows_cov=sorted(map(str, spec_rows)), n_spec_rows=len(S.SPEC))

def mutation_score(tests):
    res = {}
    for m in ('edge', 'lockout'):
        killed = any(run(t, m) != run(t, None) for t in tests)
        res[m] = killed
    return res

if __name__ == '__main__':
    tests, nodes, edges = gen_tests()
    cov = coverage(tests)
    cov['n_tests'] = len(tests); cov['total_steps'] = sum(map(len, tests)); cov['mutants_killed'] = mutation_score(tests)
    # tambem mutantes manuais adicionais (operadores de mutacao simples na SPEC) para nota de qualidade da suite
    json.dump(dict(tests=tests, expected=[run(t) for t in tests], coverage=cov), open('../tests/generated_tests.json', 'w'))
    json.dump(cov, open('../results/coverage.json', 'w'), indent=1)
    print(cov)
