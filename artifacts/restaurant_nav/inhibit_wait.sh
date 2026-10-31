#!/usr/bin/env bash
# Keep the inhibitor alive while the given queue PIDs run (no pgrep patterns:
# a pattern in this command line once matched the queues' own waits).
for pid in "$@"; do while kill -0 "$pid" 2>/dev/null; do sleep 15; done; done
