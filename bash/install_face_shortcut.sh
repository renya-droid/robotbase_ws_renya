#!/usr/bin/env bash
# Install commands without changing .bashrc or desktop autostart.
set -e
KOKO_FACE_WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
KOKO_FACE_BIN_DIR="${KOKO_FACE_BIN_DIR:-${HOME}/.local/bin}"
mkdir -p -- "$KOKO_FACE_BIN_DIR"
for name in koko_face koko-face koko_face2; do
  target="${KOKO_FACE_BIN_DIR}/${name}"
  if [[ -e "$target" || -L "$target" ]]; then
    cp -a -- "$target" "${target}.backup.$(date +%Y%m%d_%H%M%S).$$"
  fi
  temporary="$(mktemp "${KOKO_FACE_BIN_DIR}/.koko-face.XXXXXX")"
  launcher=koko_face.sh
  if [[ "$name" == koko_face2 ]]; then launcher=koko_face2.sh; fi
  printf '#!/usr/bin/env bash\nexec bash %q "$@"\n' "${KOKO_FACE_WS}/bash/startup_bash/${launcher}" > "$temporary"
  chmod +x "$temporary"
  mv -f -- "$temporary" "$target"
done
echo "黒背景版: ${KOKO_FACE_BIN_DIR}/koko_face"
echo "背景透明版: ${KOKO_FACE_BIN_DIR}/koko_face2"
