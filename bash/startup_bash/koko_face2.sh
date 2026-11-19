#!/usr/bin/env bash
# Transparent desktop companion; the original koko_face retains its black canvas.
set -e
KOKO_FACE2_SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec bash "${KOKO_FACE2_SCRIPT_DIR}/koko_face.sh" --transparent "$@"
