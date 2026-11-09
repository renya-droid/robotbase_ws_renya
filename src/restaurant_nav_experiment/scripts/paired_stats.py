#!/usr/bin/env python3
"""Paired comparison of to_table legs (same seed + trip = same table).

Failed or timed-out legs count as the 120 s leg limit (a conservative
censored value). Prints, for each pair of conditions: number of legs where the
first was better / worse / tied (|diff| < 1 s), the mean difference, and a 95%
bootstrap interval over legs. Usage: paired_stats.py <root> [<root2> ...]
"""
import csv
from pathlib import Path
import random
import statistics as st
import sys

LIMIT = 120.0


def load(roots):
    legs = {}
    for root in roots:
        tag = '' if root == roots[0] else '+' + Path(root).name.split('_')[-1]
        for f in Path(root).glob('*/legs.csv'):
            if not (f.parent/'summary.json').exists():
                continue
            for r in csv.DictReader(open(f)):
                if r['leg'] != 'to_table':
                    continue
                cond = r['condition'] + tag
                t = float(r['duration']) if r['result'] == 'SUCCEEDED' else LIMIT
                legs[(r['seed'], int(r['trip']), cond)] = dict(t=t, ok=r['result'] == 'SUCCEEDED',
                                                                stop=float(r['human_stopped_near_robot_s']),
                                                                contact=int(r['contact_samples']))
    return legs


def boot(diffs, n=5000, seed=1):
    rng = random.Random(seed)
    means = sorted(st.mean(rng.choice(diffs) for _ in diffs) for _ in range(n))
    return means[int(.025*n)], means[int(.975*n)]


def main(roots):
    legs = load(roots)
    conds = sorted({k[2] for k in legs})
    for i, a in enumerate(conds):
        for b in conds[i+1:]:
            keys = sorted({k[:2] for k in legs if (k[0], k[1], a) in legs and (k[0], k[1], b) in legs})
            if not keys:
                continue
            for metric in ('t', 'stop'):
                d = [legs[(*k, a)][metric] - legs[(*k, b)][metric] for k in keys]
                better = sum(x < -1 for x in d); worse = sum(x > 1 for x in d)
                lo, hi = boot(d)
                name = 'time to table (s, fail=120)' if metric == 't' else 'people stopped near robot (s)'
                print(f'{a} - {b}  {name}: n={len(d)} {a} better {better}, worse {worse}, '
                      f'tie {len(d)-better-worse}; mean diff {st.mean(d):+.1f} [95% boot {lo:+.1f}, {hi:+.1f}]')
            ok_a = sum(legs[(*k, a)]['ok'] for k in keys); ok_b = sum(legs[(*k, b)]['ok'] for k in keys)
            print(f'{a} vs {b}  succeeded {ok_a}/{len(keys)} vs {ok_b}/{len(keys)}')


if __name__ == '__main__':
    main(sys.argv[1:] or ['.'])
