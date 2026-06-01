import json

import pytest

from openpilot.selfdrive.controls.lib.chauffeur_learned import (
  BinParams,
  ChauffeurStopLearner,
  LearnedStore,
  default_store,
  detect_rollback,
  load_store,
  pitch_roll_bin_index,
  save_store,
)


class TestChauffeurLearned:

  def test_pitch_roll_bins(self):
    flat = pitch_roll_bin_index(0.0, 0.0)
    uphill = pitch_roll_bin_index(0.08, 0.0)
    downhill = pitch_roll_bin_index(-0.08, 0.0)
    assert flat != uphill
    assert flat != downhill

  def test_detect_rollback(self):
    assert detect_rollback(-0.1, 0.0, True)
    assert detect_rollback(0.1, -0.1, True)
    assert not detect_rollback(0.1, 0.1, True)
    assert not detect_rollback(-0.1, -0.1, False)

  def test_store_roundtrip(self, tmp_path, monkeypatch):
    path = tmp_path / "params"
    path.mkdir()

    class FakeParams:
      def __init__(self):
        self.data = {}

      def get(self, key):
        return self.data.get(key)

      def put(self, key, val):
        self.data[key] = val

    params = FakeParams()
    store = default_store()
    store.bins["0"] = BinParams(decel_offset=-0.05, count=10)
    save_store(store, params)
    loaded = load_store(params)
    assert loaded.bins["0"].decel_offset == -0.05
    assert loaded.bins["0"].count == 10

  def test_rollback_episode_strengthens_hold(self):
    learner = ChauffeurStopLearner()
    learner.store = default_store()
    idx = pitch_roll_bin_index(0.04, 0.0)
    key = str(idx)

    learner.active = True
    learner.start_t = 0.0
    learner.mean_pitch = 0.04
    learner.mean_roll = 0.0
    learner.sample_count = 10
    learner.samples = [
      type("S", (), {"t": 0.0, "v_ego": 0.5, "a_ego": -0.5, "pitch": 0.04, "roll": 0.0, "v_forward": 0.4})(),
      type("S", (), {"t": 0.1, "v_ego": 0.05, "a_ego": 0.6, "pitch": 0.04, "roll": 0.0, "v_forward": -0.08})(),
    ]
    learner.saw_rollback = True
    before = learner.store.bins[key].decel_offset
    learner._finish_episode()
    assert learner.store.bins[key].decel_offset < before
