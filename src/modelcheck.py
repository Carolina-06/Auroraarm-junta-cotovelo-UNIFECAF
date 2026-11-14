"""Verificador exaustivo (explicit-state model checker) do supervisor de seguranca com tempo discreto (1 tick = 1 ms).

Composicao: ambiente (mao, rearme, sensor com atraso <= D) || escalonador (ciclo de 2 ms, ate MAXSKIP ciclos
perdidos consecutivos por jitter) || supervisor (supervisor.step) || atuador (comando aplicado no inicio do ciclo seguinte).

Propriedades:
  P1 (REQ-SAFE-AA-001, seguranca com prazo): hand_true continuamente >= 100 ms  =>  torque efetivo = 0.
  P2 (REQ-SAFE-AA-002, seguranca): todo 0->1 do comando de torque ocorre com borda de rearme e sensed=0.
  P3 (REQ-SAFE-AA-003, vivacidade): sob justica (mao some e rearme e pressionado infinitas vezes),
      o supervisor volta a MOVING.  Verificada por ausencia de ciclo justo fora de MOVING (SCC).
"""
import sys, json, time
from collections import deque
sys.path.insert(0, '.')
import supervisor as S

DEADLINE = 100   # ticks (ms)

def make_checker(D=20, MAXSKIP=1, mutant=None):
    memo = {}
    def sup_step(sup, sensed, rearm):
        k = (sup, sensed, rearm)
        r = memo.get(k)
        if r is None:
            r = S.step(sup, (sensed, rearm), mutant)[0]; memo[k] = r
        return r

    # estado: (sup, drv_act, phase, skipped, hand, stable, sensed, rearm)
    init = (S.INIT, 1, 0, 0, 0, D, 0, 0)

    def successors(st):
        sup, drv_act, phase, skipped, hand, stable, sensed, rearm = st
        out = []
        for h2 in (0, 1):
            if h2 == hand: st2 = min(stable + 1, DEADLINE if h2 else D)
            else: st2 = 0
            sens_opts = (h2,) if st2 >= D else (0, 1)
            for se2 in sens_opts:
                for re2 in (0, 1):
                    if phase == 0:
                        drv_act2 = sup[4]
                        skip_opts = (False, True) if skipped < MAXSKIP else (False,)
                        for sk in skip_opts:
                            if sk: sup2, skipped2 = sup, skipped + 1
                            else:  sup2, skipped2 = sup_step(sup, se2, re2), 0
                            out.append(((sup2, drv_act2, 1, skipped2, h2, st2, se2, re2),
                                        dict(h=h2, se=se2, re=re2, run=not sk, sup0=sup, sup1=sup2, sensed=se2)))
                    else:
                        out.append(((sup, drv_act, 0, skipped, h2, st2, se2, re2),
                                    dict(h=h2, se=se2, re=re2, run=False, sup0=sup, sup1=sup, sensed=se2)))
        return out
    return init, successors

def check(D=20, MAXSKIP=1, mutant=None, tres=None, want_liveness=True, verbose=True):
    if tres is not None: S.T_RES = tres
    init, succ = make_checker(D, MAXSKIP, mutant)
    t0 = time.time()
    parent = {init: None}; q = deque([init]); ntrans = 0
    viol = {'P1': None, 'P2': None}
    worst_exposure = 0
    edges = {}                       # so para estados do grafo de vivacidade
    def trace(s):
        tr = []
        while s is not None:
            p = parent[s]
            tr.append((s, p[1] if p else None)); s = p[0] if p else None
        return tr[::-1]
    live_nodes = set()
    while q:
        s = q.popleft()
        sup, drv_act, phase, skipped, hand, stable, sensed, rearm = s
        if hand and drv_act: worst_exposure = max(worst_exposure, stable)
        if viol['P1'] is None and hand and stable >= DEADLINE and drv_act:
            viol['P1'] = s
        sl = hand == 0 and sup[0] != S.MOVING
        for s2, info in succ(s):
            ntrans += 1
            # P2: transicao 0->1 do comando
            if viol['P2'] is None and info['run'] and info['sup0'][4] == 0 and info['sup1'][4] == 1:
                rearm_edge = info['re'] and not info['sup0'][3]
                if not (rearm_edge and not info['se']):
                    parent.setdefault(s2, (s, info)); viol['P2'] = s2
            if s2 not in parent:
                parent[s2] = (s, info); q.append(s2)
            if want_liveness and sl and s2[4] == 0 and s2[1-1][0] != S.MOVING:
                fair = info['run'] and (not info['se']) and info['re'] and not info['sup0'][3]
                edges.setdefault(s, []).append((s2, fair))
    res = dict(D=D, MAXSKIP=MAXSKIP, mutant=mutant, states=len(parent), transitions=ntrans,
               worst_exposure_ms=worst_exposure, seconds=round(time.time() - t0, 1))
    res['P1_holds'] = viol['P1'] is None
    res['P2_holds'] = viol['P2'] is None
    res['cex_P1'] = fmt_trace(trace(viol['P1'])) if viol['P1'] else None
    res['cex_P2'] = fmt_trace(trace(viol['P2'])) if viol['P2'] else None
    # P3: SCC (Tarjan iterativo) no subgrafo (mao ausente, modo != MOVING)
    if want_liveness:
        bad, nscc = fair_cycle(edges)
        res['P3_holds'] = bad is None
        res['liveness_graph_nodes'] = len(edges)
        res['cex_P3'] = None
        if bad is not None:
            res['cex_P3'] = dict(prefix=fmt_trace(trace(bad[0])), loop_state=fmt_state(bad[0]))
    return res

def fair_cycle(edges):
    index = {}; low = {}; onst = set(); st = []; idx = [0]; bad = [None]; nscc = [0]
    nodes = list(edges.keys())
    for root in nodes:
        if root in index: continue
        work = [(root, 0)]; index[root] = low[root] = idx[0]; idx[0] += 1; st.append(root); onst.add(root)
        while work:
            v, i = work[-1]
            es = edges.get(v, [])
            if i < len(es):
                work[-1] = (v, i + 1); w = es[i][0]
                if w not in edges: continue
                if w not in index:
                    index[w] = low[w] = idx[0]; idx[0] += 1; st.append(w); onst.add(w); work.append((w, 0))
                elif w in onst: low[v] = min(low[v], index[w])
            else:
                work.pop()
                if work: u = work[-1][0]; low[u] = min(low[u], low[v])
                if low[v] == index[v]:
                    comp = []
                    while True:
                        w = st.pop(); onst.discard(w); comp.append(w)
                        if w == v: break
                    nscc[0] += 1
                    cs = set(comp)
                    has_cycle = len(comp) > 1 or any(x[0] == v for x in edges.get(v, []))
                    if has_cycle:
                        for a in comp:
                            for (b, fair) in edges.get(a, []):
                                if fair and b in cs and bad[0] is None: bad[0] = (a,)
    return bad[0], nscc[0]

def fmt_state(s):
    sup, drv_act, phase, skipped, hand, stable, sensed, rearm = s
    return dict(mode=S.NAMES[sup[0]], cnt=sup[1], drv_cmd=sup[4], drv_act=drv_act, hand=hand, t_stable=stable, sensed=sensed, rearm=rearm)

def fmt_trace(tr):
    rows = []
    for k, (s, info) in enumerate(tr):
        d = fmt_state(s); d['tick'] = k; rows.append(d)
    # comprime trechos identicos consecutivos (mesmo estado exceto contadores) para legibilidade
    return rows

if __name__ == '__main__':
    out = {}
    out['final'] = check(D=20, MAXSKIP=1)
    print({k: v for k, v in out['final'].items() if not k.startswith('cex')})
    out['mut_edge'] = check(D=20, MAXSKIP=1, mutant='edge', want_liveness=False)
    print({k: v for k, v in out['mut_edge'].items() if not k.startswith('cex')}, len(out['mut_edge']['cex_P1'] or []))
    out['mut_lockout'] = check(D=20, MAXSKIP=1, mutant='lockout')
    print({k: v for k, v in out['mut_lockout'].items() if not k.startswith('cex')})
    json.dump(out, open('../results/modelcheck.json', 'w'), indent=1)
