#!/usr/bin/env bash
# Run conditions x seeds sequentially, then aggregate.
#   run_pilot.sh <out_root> "<conditions>" "<seeds>" [scenario] [extra launch args...]
# e.g. run_pilot.sh artifacts/restaurant_nav/pilot "B0 B1 P" "42 11" high stop_sim:=330.0
set -uo pipefail
root="$1"; conditions="$2"; seeds="$3"; scenario="${4:-high}"; shift $(( $# >= 4 ? 4 : 3 ))
here="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
mkdir -p "$root"
for seed in $seeds; do
    for condition in $conditions; do
        out="$root/${scenario}_seed${seed}_${condition}"
        if [[ -f "$out/summary.json" ]]; then echo "skip $out (done)"; continue; fi
        rm -rf "$out"
        echo "=== $(date +%H:%M:%S) $condition seed $seed"
        bash "$here/run_condition.sh" "$condition" "$scenario" "$seed" "$out" "$@"
    done
done
python3 "$here/summarize.py" "$root"
