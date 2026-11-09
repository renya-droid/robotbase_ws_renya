#!/usr/bin/env python3
"""Aggregate run directories (each with legs.csv + summary.json) into
results.csv (one row per run) and results.md (per-condition means)."""
import csv
import json
from pathlib import Path
import statistics as st
import sys

KEYS = ['table_legs', 'table_succeeded', 'mean_table_duration_succeeded', 'legs', 'succeeded',
        'mean_duration', 'mean_path_length', 'mean_robot_stopped_s', 'min_gap_walking',
        'near_miss_events', 'contact_samples', 'human_stopped_near_robot_s', 'seat_occupied_on_arrival',
        'resets', 'people_failures_reported']


def paired(root):
    """Same seed and trip index = same table in every condition: compare those legs only."""
    legs = {}
    for f in root.glob('*/legs.csv'):
        for r in csv.DictReader(open(f)):
            if r['leg'] == 'to_table':
                legs[(r['seed'], int(r['trip']), r['condition'])] = r
    conds = sorted({k[2] for k in legs})
    keys = sorted({k[:2] for k in legs}, key=lambda k: (int(k[0]), k[1]))
    both = [k for k in keys if all((k[0], k[1], c) in legs for c in conds)]
    out = ['Paired to_table legs (same seed and table in every condition): ' + str(len(both)),
           '| condition | succeeded | mean duration all paired (s) | mean duration where all succeeded (s) | people stopped near robot (s/leg) |',
           '|---|---|---|---|---|']
    all_ok = [k for k in both if all(legs[(k[0], k[1], c)]['result'] == 'SUCCEEDED' for c in conds)]
    for c in conds:
        rs = [legs[(k[0], k[1], c)] for k in both]
        ok = [legs[(k[0], k[1], c)] for k in all_ok]
        mean = lambda xs, key: f"{st.mean(float(x[key]) for x in xs):.2f}" if xs else '-'
        out.append(f"| {c} | {sum(r['result'] == 'SUCCEEDED' for r in rs)}/{len(rs)} | {mean(rs, 'duration')} | "
                   f"{mean(ok, 'duration')} (n={len(ok)}) | {mean(rs, 'human_stopped_near_robot_s')} |")
    return '\n'.join(out)


def main(root):
    root = Path(root)
    rows = []
    for summary in sorted(root.glob('*/summary.json')):
        s = json.loads(summary.read_text())
        s['resets'] = len(s.get('resets', []))
        s['run'] = summary.parent.name
        rows.append(s)
    if not rows:
        print('no runs with summary.json under', root); return
    with open(root/'results.csv', 'w', newline='') as f:
        w = csv.DictWriter(f, ['run', 'condition', 'seed'] + KEYS, extrasaction='ignore'); w.writeheader(); w.writerows(rows)
    lines = ['| condition | runs | table legs ok | mean table time (s, ok) | mean leg time (s) | path (m) |'
             ' robot stopped (s/leg) | min gap walking (m) | near-miss | contact samples | people stopped near robot (s) | resets |',
             '|' + '---|'*12]
    for cond in sorted({r['condition'] for r in rows}):
        rs = [r for r in rows if r['condition'] == cond]
        def m(k):
            v = [r[k] for r in rs if r.get(k) is not None]
            return f'{st.mean(v):.2f}' if v else '-'
        ok = sum(r['table_succeeded'] for r in rs); tot = sum(r['table_legs'] for r in rs)
        lines.append(f"| {cond} | {len(rs)} | {ok}/{tot} | {m('mean_table_duration_succeeded')} | {m('mean_duration')} | "
                     f"{m('mean_path_length')} | {m('mean_robot_stopped_s')} | "
                     f"{min((r['min_gap_walking'] for r in rs if r['min_gap_walking'] is not None), default='-')} | "
                     f"{sum(r['near_miss_events'] for r in rs)} | {sum(r['contact_samples'] for r in rs)} | "
                     f"{m('human_stopped_near_robot_s')} | {sum(r['resets'] for r in rs)} |")
    lines += ['', paired(root)]
    (root/'results.md').write_text('\n'.join(lines) + '\n')
    print('\n'.join(lines))


if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) > 1 else '.')
