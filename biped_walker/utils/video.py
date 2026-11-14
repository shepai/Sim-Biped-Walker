"""Video writing helper."""

from __future__ import annotations

from pathlib import Path
from typing import Sequence

import numpy as np


def save_video(frames: Sequence[np.ndarray], path: str | Path, fps: int = 50) -> Path:
    """Write RGB frames to ``path`` (.mp4 needs ``imageio-ffmpeg``; .gif works without it)."""
    import imageio.v2 as imageio

    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.suffix.lower() == ".gif":
        imageio.mimsave(path, list(frames), duration=1000.0 / fps, loop=0)
        return path
    try:
        with imageio.get_writer(path, fps=fps, codec="libx264", quality=8,
                                macro_block_size=1) as writer:
            for f in frames:
                writer.append_data(np.asarray(f))
    except (ImportError, ValueError, RuntimeError) as err:  # no ffmpeg backend
        gif = path.with_suffix(".gif")
        print(f"could not write {path.suffix} ({err}); writing {gif} instead")
        imageio.mimsave(gif, list(frames), duration=1000.0 / fps, loop=0)
        return gif
    return path
