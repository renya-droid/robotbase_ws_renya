#!/usr/bin/env bash
set -euo pipefail
REPLAY_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec env -u ELECTRON_RUN_AS_NODE /usr/bin/foxglove-studio "$REPLAY_ROOT/data/data_0.mcap"
