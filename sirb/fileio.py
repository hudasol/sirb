"""Bounded file reading for files that may come from someone else (show specs, export bundles)."""

from __future__ import annotations

import stat
from pathlib import Path

MAX_FILE_BYTES = 512 * 1024 * 1024   # a 10,000-drone export is of the order of 150 MB


def read_text_limited(path: str | Path, max_bytes: int | None = None) -> str:
    """Read a UTF-8 text file; refuse non-regular files (devices, FIFOs, directories) and files
    larger than ``max_bytes``. Raises ValueError (or OSError if the file cannot be opened)."""
    limit = MAX_FILE_BYTES if max_bytes is None else max_bytes
    p = Path(path)
    st = p.stat()
    if not stat.S_ISREG(st.st_mode):
        raise ValueError(f"{p} is not a regular file")
    if st.st_size > limit:
        raise ValueError(f"{p} is {st.st_size} bytes, above the {limit} byte limit")
    with p.open("r", encoding="utf-8") as fh:
        text = fh.read(limit + 1)
    if len(text) > limit:           # file grew while reading
        raise ValueError(f"{p} exceeds the {limit} byte limit")
    return text
