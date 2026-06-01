import json
import math
from dataclasses import asdict, dataclass, field

import numpy as np

from openpilot.common.constants import ACCELERATION_DUE_TO_GRAVITY, CV
from openpilot.common.params import Params, UnknownKeyName
from openpilot.common.realtime import DT_CTRL

PARAM_KEY = "ChauffeurStopLearnedParams"
LEARNING_ENABLED_KEY = "ChauffeurStopLearningEnabled"
CHAUFFEUR_ENABLED_KEY = "ChauffeurStopEnabled"
PARAM_VERSION = 1
CHAUFFEUR_MAX_SPEED = 2.0 * CV.MPH_TO_MS
ROLLBACK_V_EGO = -0.03
ROLLBACK_V_FORWARD = -0.05

def hill_hold_offset(pitch: float) -> float:
  return math.sin(pitch) * ACCELERATION_DUE_TO_GRAVITY


def in_chauffeur_zone(v_ego: float) -> bool:
  return 0.0 <= v_ego < CHAUFFEUR_MAX_SPEED


def detect_rollback(v_ego: float, v_forward: float, chauffeur_active: bool) -> bool:
  if not chauffeur_active:
    return False
  return v_ego < ROLLBACK_V_EGO or v_forward < ROLLBACK_V_FORWARD


def _safe_get_bool(params: Params, key: str, default: bool = False) -> bool:
  try:
    return params.get_bool(key)
  except UnknownKeyName:
    return default


def is_chauffeur_stop_enabled(params: Params | None = None) -> bool:
  params = params or Params()
  return _safe_get_bool(params, CHAUFFEUR_ENABLED_KEY)

PITCH_BOUNDS = [-0.06, -0.02, 0.02, 0.06]
ROLL_BOUNDS = [-0.015, 0.015]
NUM_PITCH_BINS = len(PITCH_BOUNDS) + 1
NUM_ROLL_BINS = len(ROLL_BOUNDS) + 1
NUM_BINS = NUM_PITCH_BINS * NUM_ROLL_BINS

JOLT_THRESHOLD = 0.8
CREEP_THRESHOLD = 0.12
EPISODE_TIMEOUT = 10.0
MIN_EPISODES_TO_APPLY = 8
LEARN_STEP_DECEL = 0.015
LEARN_STEP_JERK = 0.08
LEARN_STEP_GOOD = 0.003

DECEL_OFFSET_MIN = -0.45
DECEL_OFFSET_MAX = 0.15
JERK_OFFSET_MIN = -0.8
JERK_OFFSET_MAX = 0.4


@dataclass
class BinParams:
  decel_offset: float = 0.0
  jerk_offset: float = 0.0
  taper_offset: float = 0.0
  count: int = 0


@dataclass
class LearnedStore:
  version: int = PARAM_VERSION
  bins: dict[str, BinParams] = field(default_factory=dict)


@dataclass
class EpisodeSample:
  t: float
  v_ego: float
  a_ego: float
  pitch: float
  roll: float
  v_forward: float


def pitch_roll_bin_index(pitch: float, roll: float) -> int:
  pitch_bin = NUM_PITCH_BINS - 1
  for i, bound in enumerate(PITCH_BOUNDS):
    if pitch < bound:
      pitch_bin = i
      break

  roll_bin = NUM_ROLL_BINS - 1
  for i, bound in enumerate(ROLL_BOUNDS):
    if roll < bound:
      roll_bin = i
      break

  return pitch_bin * NUM_ROLL_BINS + roll_bin


def bin_key(bin_index: int) -> str:
  return str(bin_index)


def default_store() -> LearnedStore:
  store = LearnedStore()
  for i in range(NUM_BINS):
    store.bins[bin_key(i)] = BinParams()
  return store


def load_store(params: Params | None = None) -> LearnedStore:
  params = params or Params()
  try:
    raw = params.get(PARAM_KEY)
  except UnknownKeyName:
    return default_store()
  if raw is None:
    return default_store()

  try:
    data = json.loads(raw)
  except (json.JSONDecodeError, TypeError):
    return default_store()

  if data.get("version") != PARAM_VERSION:
    return default_store()

  store = default_store()
  for key, values in data.get("bins", {}).items():
    if key not in store.bins:
      continue
    store.bins[key] = BinParams(
      decel_offset=float(values.get("decel_offset", 0.0)),
      jerk_offset=float(values.get("jerk_offset", 0.0)),
      taper_offset=float(values.get("taper_offset", 0.0)),
      count=int(values.get("count", 0)),
    )
  return store


def save_store(store: LearnedStore, params: Params | None = None) -> None:
  params = params or Params()
  payload = {
    "version": store.version,
    "bins": {key: asdict(val) for key, val in store.bins.items()},
  }
  try:
    params.put(PARAM_KEY, json.dumps(payload))
  except UnknownKeyName:
    pass


def get_bin_params(store: LearnedStore, pitch: float, roll: float) -> BinParams:
  idx = pitch_roll_bin_index(pitch, roll)
  return store.bins[bin_key(idx)]


def learned_offsets_for_grade(store: LearnedStore, pitch: float, roll: float) -> tuple[float, float, float]:
  params = get_bin_params(store, pitch, roll)
  if params.count < MIN_EPISODES_TO_APPLY:
    return 0.0, 0.0, 0.0
  return params.decel_offset, params.jerk_offset, params.taper_offset


def _clip_offsets(decel_offset: float, jerk_offset: float, taper_offset: float = 0.0) -> tuple[float, float, float]:
  return (
    float(np.clip(decel_offset, DECEL_OFFSET_MIN, DECEL_OFFSET_MAX)),
    float(np.clip(jerk_offset, JERK_OFFSET_MIN, JERK_OFFSET_MAX)),
    float(np.clip(taper_offset, -0.3, 0.3)),
  )


class ChauffeurStopLearner:
  def __init__(self):
    self.params = Params()
    self.store = load_store(self.params)
    self.active = False
    self.invalid = False
    self.start_t = 0.0
    self.mean_pitch = 0.0
    self.mean_roll = 0.0
    self.sample_count = 0
    self.samples: list[EpisodeSample] = []
    self.saw_rollback = False
    self.persist_counter = 0

  def is_learning_enabled(self) -> bool:
    return _safe_get_bool(self.params, LEARNING_ENABLED_KEY)

  def reset_episode(self) -> None:
    self.active = False
    self.invalid = False
    self.start_t = 0.0
    self.mean_pitch = 0.0
    self.mean_roll = 0.0
    self.sample_count = 0
    self.samples = []
    self.saw_rollback = False

  def _episode_invalid(self, long_active: bool, brake_pressed: bool, gas_pressed: bool,
                       fcw: bool, chauffeur_enabled: bool) -> bool:
    return (not long_active or brake_pressed or gas_pressed or fcw or not chauffeur_enabled)

  def update(self, t: float, v_ego: float, a_ego: float, pitch: float, roll: float, v_forward: float,
             long_active: bool, brake_pressed: bool, gas_pressed: bool, standstill: bool,
             should_stop: bool, fcw: bool) -> None:
    if not self.is_learning_enabled() or not is_chauffeur_stop_enabled(self.params):
      if self.active:
        self.reset_episode()
      return

    chauffeur_context = long_active and (should_stop or in_chauffeur_zone(v_ego)) and a_ego < 0.05

    if self.active and self._episode_invalid(long_active, brake_pressed, gas_pressed, fcw, True):
      self.reset_episode()
      return

    if detect_rollback(v_ego, v_forward, chauffeur_context and in_chauffeur_zone(v_ego)):
      self.saw_rollback = True

    if not self.active:
      if chauffeur_context and in_chauffeur_zone(v_ego) and v_ego > 0.05 and a_ego < -0.05:
        self.active = True
        self.start_t = t
        self.mean_pitch = pitch
        self.mean_roll = roll
        self.sample_count = 1
        self.samples = []
      else:
        return
    else:
      self.mean_pitch = (self.mean_pitch * self.sample_count + pitch) / (self.sample_count + 1)
      self.mean_roll = (self.mean_roll * self.sample_count + roll) / (self.sample_count + 1)
      self.sample_count += 1

    self.samples.append(EpisodeSample(t, v_ego, a_ego, pitch, roll, v_forward))

    timed_out = (t - self.start_t) > EPISODE_TIMEOUT
    finished = standstill or (v_ego < 0.08 and (t - self.start_t) > 0.5)
    if not (finished or timed_out):
      return

    if timed_out and not standstill:
      self.reset_episode()
      return

    self._finish_episode()
    self.reset_episode()

  def _finish_episode(self) -> None:
    if len(self.samples) < 5:
      return

    idx = pitch_roll_bin_index(self.mean_pitch, self.mean_roll)
    key = bin_key(idx)
    params = self.store.bins[key]

    jolt = 0.0
    creep = 0.0
    for i in range(1, len(self.samples)):
      dt = max(self.samples[i].t - self.samples[i - 1].t, DT_CTRL)
      da = (self.samples[i].a_ego - self.samples[i - 1].a_ego) / dt
      if self.samples[i].v_ego < CHAUFFEUR_MAX_SPEED:
        jolt = max(jolt, da)

    tail = self.samples[-min(len(self.samples), 40):]
    creep = max(s.v_ego for s in tail)

    hill = hill_hold_offset(self.mean_pitch)
    decel = params.decel_offset
    jerk = params.jerk_offset

    if self.saw_rollback or (hill < -0.15 and creep > CREEP_THRESHOLD):
      decel -= LEARN_STEP_DECEL * 2.0
    elif jolt > JOLT_THRESHOLD:
      decel += LEARN_STEP_DECEL
      jerk -= LEARN_STEP_JERK
    else:
      decel += LEARN_STEP_GOOD
      jerk += LEARN_STEP_JERK * 0.25

    decel, jerk, taper = _clip_offsets(decel, jerk, params.taper_offset)
    params.decel_offset = decel
    params.jerk_offset = jerk
    params.taper_offset = taper
    params.count += 1

    self.persist_counter += 1
    if self.persist_counter >= 3:
      save_store(self.store, self.params)
      self.persist_counter = 0
