#!/usr/bin/env python3
"""Check finalized MCAP indexes; recover unfinished files without losing originals."""

from pathlib import Path
import os
import shutil
import struct
import subprocess
import sys

MAGIC = b"\x89MCAP0\r\n"
WORKSPACE = Path(__file__).resolve().parents[1]


def is_finalized(path):
    with path.open("rb") as stream:
        size = path.stat().st_size
        if size < 37 or stream.read(8) != MAGIC:
            return False
        stream.seek(-37, 2)
        footer = stream.read(37)
    opcode, length, summary_start, summary_offsets, _ = struct.unpack("<BQQQI", footer[:29])
    return (opcode == 2 and length == 20 and footer[29:] == MAGIC
            and 8 < summary_start < size - 37
            and (summary_offsets == 0 or summary_start <= summary_offsets < size - 37))


def finalize(directory):
    files = sorted(directory.glob("*.mcap"))
    if not files:
        raise RuntimeError(f"MCAPファイルがありません: {directory}")
    repaired = False
    for path in files:
        if is_finalized(path):
            continue
        cli = (os.environ.get("MCAP_BIN") or shutil.which("mcap")
               or str(WORKSPACE / ".runtime/bin/mcap"))
        if not Path(cli).is_file():
            raise RuntimeError(f"{path.name}: 未完了です。修復用mcapコマンドがありません")
        backup = path.with_suffix(path.suffix + ".damaged")
        temporary = path.with_suffix(path.suffix + ".recovering")
        if backup.exists() or temporary.exists():
            raise RuntimeError(f"{path.name}: 以前の修復ファイルがあるため上書きしません")
        print(f"[MCAP] {path.name}: 未完了のファイルを修復します", flush=True)
        result = subprocess.run([cli, "recover", str(path), "--compression", "zstd",
                                 "-o", str(temporary)])
        # CLI status 3 means the readable records were recovered from truncated input.
        if result.returncode not in (0, 3) or not is_finalized(temporary):
            raise RuntimeError(f"修復に失敗しました。元ファイルは保持されています: {path}")
        path.rename(backup)
        temporary.rename(path)
        repaired = True
        print(f"[MCAP] 修復済み。元ファイル: {backup}", flush=True)
        if result.returncode == 3:
            print("[MCAP] 注意: 破損した末尾のデータは復旧できていません", flush=True)
    if repaired:
        subprocess.run(["ros2", "bag", "reindex", "-s", "mcap", str(directory)], check=True)


if __name__ == "__main__":
    try:
        finalize(Path(sys.argv[1]))
    except (OSError, RuntimeError, subprocess.CalledProcessError) as error:
        print(f"[MCAP] {error}", file=sys.stderr)
        sys.exit(1)
