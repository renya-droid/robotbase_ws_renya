#!/usr/bin/env bash
# Default is preflight only. --run explicitly starts the loop.
set -eo pipefail
cd "$(dirname "$0")/../.."
python3 - <<'PY'
import json, pathlib, subprocess
root=pathlib.Path.cwd().resolve()
m=json.loads((root/'RALPH_WORKTREE.json').read_text())
assert str(root)==m['worktree'], 'Wrong worktree'
assert str(root)!=m['original'], 'Refusing original workspace'
assert (root/'.git').is_file(), 'Not a linked worktree'
assert pathlib.Path(m['reference_image']).is_file(), 'Reference image missing'
assert subprocess.check_output(['git','branch','--show-current'],text=True).strip()==m['branch']
for p in ['RALPH_PROMPT.md','AGENTS.md','tools/restaurant_ralph/EVALUATION.md']:
    assert (root/p).is_file(), p
print('Worktree and instructions verified:',root)
PY
command -v ralph
command -v codex
ralph --version
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=89 ROS_LOCALHOST_ONLY=1 GZ_PARTITION=restaurant_ralph_research
export ROBOTBASE_PARAMS_DIR="$PWD/params"
if [[ "${1:-}" != --run ]]; then
  printf '%s\n' 'Preflight only. Review RALPH_PROMPT.md and EVALUATION.md. Explicit start: bash tools/restaurant_ralph/run.sh --run'
  exit 0
fi
mkdir -p .ralph
exec 9>.ralph/restaurant.lock
flock -n 9 || { echo 'Another research loop is running'; exit 1; }
# Normal agent sandbox remains enabled; no automatic commits or publication.
exec ralph --agent codex --prompt-file RALPH_PROMPT.md \
  --max-iterations 15 --no-commit --no-allow-all \
  --completion-promise RESTAURANT_RESEARCH_READY \
  --abort-promise RESTAURANT_BLOCKED -- --sandbox workspace-write
