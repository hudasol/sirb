"""Numerical environment block recorded inside every results file.

Reports the interpreter, the operating system, the versions of the numerical libraries, the
thread settings that influence them and the compute device. Seeds are passed in by the caller.
"""

from __future__ import annotations

import os
import platform
import sys


def environment(seeds: dict | None = None) -> dict:
    import numpy
    import scipy

    from . import __version__

    threads = {k: os.environ.get(k) for k in
               ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "SIRB_WORKERS")}
    return {
        "python": sys.version.split()[0],
        "implementation": platform.python_implementation(),
        "numpy": numpy.__version__,
        "scipy": scipy.__version__,
        "sirb": __version__,
        "os": f"{platform.system()} {platform.release()}",
        "machine": platform.machine(),
        "cpu_count": os.cpu_count(),
        "thread_environment": threads,
        "device": "cpu",
        "seeds": dict(seeds or {}),
    }
