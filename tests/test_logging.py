"""Run logger and progress bar (no MuJoCo needed: a dummy agent stands in)."""

import csv
import json
from pathlib import Path

import numpy as np

from biped_walker.rl.logger import ConsoleLogger, RunLogger
from biped_walker.utils.progress import ProgressBar


class DummyAgent:
    algo = "dummy"
    env_id = "SimBiped-v0"
    env_kwargs = {"history_len": 3}
    seed = 0

    def __init__(self):
        self.saved, self.videos = [], []

    def hyperparameters(self):
        return {"lr": 0.1}

    def make_env(self, **_):
        raise RuntimeError("no simulator in this test")

    def save(self, d):
        Path(d).mkdir(parents=True, exist_ok=True)
        (Path(d) / "agent.json").write_text("{}")
        self.saved.append(Path(d).name)

    def visualize(self, video_path, **kw):
        Path(video_path).write_bytes(b"fake")
        self.videos.append(Path(video_path).name)
        return {"video": str(video_path), "episodes": [{"return": 1.0, "distance": 0.1, "command": [0.05, 0.0]}]}


def _episode(r, fell=0.0):
    return {"return": r, "length": 100, "distance": 0.2, "energy": 3.0, "cost_of_transport": 1.5,
            "fell": fell, "forward_velocity_error": 0.01, "r_lin_vel": r, "r_energy": -0.1}


def test_run_folder_contents(tmp_path):
    agent = DummyAgent()
    log = RunLogger(agent, 1000, log_root=tmp_path, run_name="run", video_every=0.25, progress_bar=False)
    for t in range(100, 1001, 100):
        log.log_episodes(t, [_episode(t / 100), _episode(t / 50, fell=1.0)])
        log.advance(t)
        log.log_metrics(t, {"loss": 1.0 / t})
        log.maybe_save_best(t)
        log.handle_milestones(t)
    log.close(1000)
    d = tmp_path / "run"
    assert {"config.json", "metrics.csv", "metrics.jsonl", "episodes.csv", "train.log", "summary.json"} <= \
        {p.name for p in d.iterdir()}
    rows = list(csv.DictReader(open(d / "metrics.csv")))
    assert len(rows) == 10 and float(rows[-1]["ep_fall_rate"]) == 0.5 and "loss" in rows[0]
    assert len(list(csv.DictReader(open(d / "episodes.csv")))) == 20
    # a video and a checkpoint at 25, 50, 75 and 100 %
    assert agent.videos == ["025pct_step000000300.mp4", "050pct_step000000500.mp4",
                            "075pct_step000000800.mp4", "100pct_step000001000.mp4"]
    assert {"025pct", "050pct", "075pct", "100pct"} <= set(agent.saved) and "best" in agent.saved
    assert json.loads((d / "summary.json").read_text())["episodes"] == 20


def test_csv_grows_new_columns(tmp_path):
    log = RunLogger(DummyAgent(), 10, log_root=tmp_path, run_name="r", video_every=None, progress_bar=False)
    log.log_metrics(1, {"a": 1.0})
    log.log_metrics(2, {"a": 2.0, "b": 3.0})
    log.close(2)
    rows = list(csv.DictReader(open(tmp_path / "r" / "metrics.csv")))
    assert rows[0]["b"] == "" and rows[1]["b"] == "3.0"


def test_console_logger_and_bar(capsys):
    log = ConsoleLogger(DummyAgent(), 100, progress_bar=True)
    log.log_episodes(50, [_episode(2.0)])
    row = log.log_metrics(50, {})
    assert np.isclose(row["ep_return_mean"], 2.0)
    log.advance(100)
    log.close(100)
    bar = ProgressBar(10, enabled=False)
    bar.update(5)
    assert bar.n == 5
