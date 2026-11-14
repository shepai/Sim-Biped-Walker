"""Training progress bar: tqdm when installed, otherwise a small built-in bar."""

from __future__ import annotations

import sys
import time
from typing import Dict, Optional


def _fmt_time(s: float) -> str:
    if s != s or s == float("inf"):  # nan / inf
        return "--:--"
    s = int(s)
    h, rem = divmod(s, 3600)
    m, sec = divmod(rem, 60)
    return f"{h}:{m:02d}:{sec:02d}" if h else f"{m:02d}:{sec:02d}"


def _fmt_postfix(d: Dict) -> str:
    parts = []
    for k, v in d.items():
        if isinstance(v, float):
            parts.append(f"{k}={v:.3g}")
        else:
            parts.append(f"{k}={v}")
    return ", ".join(parts)


class ProgressBar:
    """Timestep progress bar.

    ``update(n)`` advances by ``n`` steps; ``set_postfix(dict)`` shows the
    latest metrics; ``write(msg)`` prints a line without breaking the bar.
    ``enabled=False`` makes every call a no-op except ``write``.
    """

    def __init__(self, total: int, desc: str = "training", enabled: bool = True, initial: int = 0):
        self.total = int(total)
        self.n = int(initial)
        self.desc = desc
        self.enabled = enabled
        self._tqdm = None
        self._postfix = ""
        self._t0 = time.time()
        self._last_draw = 0.0
        self._closed = False
        if enabled:
            try:
                from tqdm.auto import tqdm
                self._tqdm = tqdm(total=self.total, initial=self.n, desc=desc, unit="step",
                                  unit_scale=True, dynamic_ncols=True, smoothing=0.05)
            except ImportError:
                self._draw(force=True)

    def update(self, n: int) -> None:
        self.n += int(n)
        if not self.enabled:
            return
        if self._tqdm is not None:
            self._tqdm.update(int(n))
        else:
            self._draw()

    def set_postfix(self, values: Dict) -> None:
        if not self.enabled:
            return
        if self._tqdm is not None:
            self._tqdm.set_postfix_str(_fmt_postfix(values), refresh=True)
        else:
            self._postfix = _fmt_postfix(values)
            self._draw(force=True)

    def write(self, msg: str) -> None:
        if self._tqdm is not None:
            self._tqdm.write(msg)
        else:
            live = self.enabled and not self._closed
            if live:
                sys.stdout.write("\r\033[K")
            print(msg, flush=True)
            if live:
                self._draw(force=True)

    def _draw(self, force: bool = False) -> None:
        now = time.time()
        if not force and now - self._last_draw < 0.5:
            return
        self._last_draw = now
        frac = min(1.0, self.n / max(self.total, 1))
        width = 28
        filled = int(round(width * frac))
        elapsed = now - self._t0
        rate = self.n / elapsed if elapsed > 0 else 0.0
        eta = (self.total - self.n) / rate if rate > 0 else float("inf")
        line = (f"\r{self.desc}: {frac * 100:5.1f}% |{'#' * filled}{'-' * (width - filled)}| "
                f"{self.n:,}/{self.total:,} [{_fmt_time(elapsed)}<{_fmt_time(eta)}, {rate:,.0f} step/s]")
        if self._postfix:
            line += f" {self._postfix}"
        sys.stdout.write("\033[K" + line)
        sys.stdout.flush()

    def close(self) -> None:
        if not self.enabled or self._closed:
            return
        self._closed = True
        if self._tqdm is not None:
            self._tqdm.close()
        else:
            self._draw(force=True)
            sys.stdout.write("\n")
            sys.stdout.flush()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
