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
    """Check Footer/Summary framing only, not data CRCs or message integrity."""
    with path.open("rb") as stream:
        size = path.stat().st_size
        if size < 37 or stream.read(8) != MAGIC:
            return False
        stream.seek(-37, 2)
        footer = stream.read(37)
        opcode, length, summary_start, summary_offsets, _ = struct.unpack("<BQQQI", footer[:29])
        if not (opcode == 2 and length == 20 and footer[29:] == MAGIC
                and 8 < summary_start < size - 37
                and (summary_offsets == 0 or summary_start <= summary_offsets < size - 37)):
            return False
        # Check Summary record boundaries without reading/decompressing data chunks.
        offset = summary_start
        boundaries = set()
        while offset < size - 37:
            boundaries.add(offset)
            stream.seek(offset)
            header = stream.read(9)
            if len(header) != 9:
                return False
            record, payload_length = struct.unpack("<BQ", header)
            if record not in (3, 4, 8, 10, 11, 13, 14):
                return False
            offset += 9 + payload_length
        return offset == size - 37 and (not summary_offsets or summary_offsets in boundaries)


def find_mcap_cli():
    """Use the reference branch's explicit override, PATH, then local runtime."""
    cli = (os.environ.get("MCAP_BIN") or shutil.which("mcap")
           or str(WORKSPACE / ".runtime/bin/mcap"))
    if not Path(cli).is_file() or not os.access(cli, os.X_OK):
        raise RuntimeError("修復用mcapコマンドがありません（MCAP_BIN / PATH / .runtime/bin/mcap）")
    return cli


def finalize(directory):
    files = sorted(directory.glob("*.mcap"))
    if not files:
        raise RuntimeError(f"MCAPファイルがありません: {directory}")
    # Persist this across partial repairs or a failed reindex; a later run must
    # not report success merely because the repaired MCAP already has a Footer.
    pending = directory / ".mcap-reindex-required"
    for path in files:
        if is_finalized(path):
            continue
        cli = find_mcap_cli()
        backup = path.with_suffix(path.suffix + ".damaged")
        temporary = path.with_suffix(path.suffix + ".recovering")
        if os.path.lexists(backup) or os.path.lexists(temporary):
            raise RuntimeError(f"{path.name}: 以前の修復ファイルがあるため上書きしません")
        # Reserve the output exclusively before the CLI opens it for writing.
        with temporary.open("xb"):
            pass
        print(f"[MCAP] {path.name}: 未完了のファイルを修復します", flush=True)
        result = subprocess.run([cli, "recover", str(path), "--compression", "zstd",
                                 "-o", str(temporary)])
        # CLI status 3 means the readable records were recovered from truncated input.
        if result.returncode not in (0, 3) or not is_finalized(temporary):
            raise RuntimeError(f"修復に失敗しました。元ファイルは保持されています: {path}")
        pending.touch(exist_ok=True)
        # link() fails if a backup appeared meanwhile; never overwrite it.
        os.link(path, backup)
        os.replace(temporary, path)
        print(f"[MCAP] 修復済み。元ファイル: {backup}", flush=True)
        if result.returncode == 3:
            print("[MCAP] 注意: 破損した末尾のデータは復旧できていません", flush=True)
    if pending.exists() or not (directory / "metadata.yaml").is_file():
        subprocess.run(["ros2", "bag", "reindex", "-s", "mcap", str(directory)], check=True)
        if not (directory / "metadata.yaml").is_file():
            raise RuntimeError("reindex後のmetadata.yamlを確認できません")
        pending.unlink(missing_ok=True)


if __name__ == "__main__":
    try:
        finalize(Path(sys.argv[1]))
    except (OSError, RuntimeError, subprocess.CalledProcessError) as error:
        print(f"[MCAP] {error}", file=sys.stderr)
        sys.exit(1)
