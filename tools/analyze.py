#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import math
import re
import struct
import sys
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Optional

import numpy as np

ACC_LSB_PER_G = 8192.0
GYRO_LSB_PER_DPS = 65.5

MARKER_NONE = 0
MARKER_BITE = 1
MARKER_FALSE_TRIGGER = 2

_BINARY_RECORD_FMT = "<Ihhhhhhb"
_BINARY_RECORD_SIZE = struct.calcsize(_BINARY_RECORD_FMT)
assert _BINARY_RECORD_SIZE == 17, _BINARY_RECORD_SIZE

_CSV_HEADER_RE = re.compile(
    r"#\s*FishBite log v1\s+sample_hz=(?P<sample_hz>\d+)\s+"
    r"record_bytes=(?P<record_bytes>\d+)\s+t0_ms=(?P<t0_ms>\d+)"
)


@dataclass
class Marker:
    t_ms: int
    type: int


@dataclass
class Capture:
    t_ms: np.ndarray
    ax: np.ndarray
    ay: np.ndarray
    az: np.ndarray
    gx: np.ndarray
    gy: np.ndarray
    gz: np.ndarray
    marker_raw: np.ndarray
    sample_hz: Optional[int] = None
    t0_ms: Optional[int] = None
    source_path: Optional[str] = None

    @property
    def accel_g(self) -> np.ndarray:
        return np.stack([self.ax, self.ay, self.az], axis=1) / ACC_LSB_PER_G

    @property
    def gyro_dps(self) -> np.ndarray:
        return np.stack([self.gx, self.gy, self.gz], axis=1) / GYRO_LSB_PER_DPS

    @property
    def accel_mag_g(self) -> np.ndarray:
        return np.linalg.norm(self.accel_g, axis=1)

    @property
    def gyro_mag_dps(self) -> np.ndarray:
        return np.linalg.norm(self.gyro_dps, axis=1)

    @property
    def markers(self) -> list[Marker]:
        idx = np.nonzero(self.marker_raw)[0]
        return [Marker(t_ms=int(self.t_ms[i]), type=int(self.marker_raw[i])) for i in idx]

    def __len__(self) -> int:
        return len(self.t_ms)


def parse_csv(path: str) -> Capture:
    sample_hz = None
    t0_ms = None
    rows: list[tuple[int, int, int, int, int, int, int, int]] = []

    with open(path, "r", newline="") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            if line.startswith("#"):
                m = _CSV_HEADER_RE.match(line)
                if m:
                    sample_hz = int(m.group("sample_hz"))
                    t0_ms = int(m.group("t0_ms"))
                continue
            fields = next(csv.reader([line]))
            if len(fields) != 8:
                continue
            t_ms, ax, ay, az, gx, gy, gz, marker = (int(v) for v in fields)
            rows.append((t_ms, ax, ay, az, gx, gy, gz, marker))

    if not rows:
        raise ValueError(f"no data rows found in {path}")

    arr = np.array(rows, dtype=np.int64)
    return Capture(
        t_ms=arr[:, 0],
        ax=arr[:, 1].astype(np.int16), ay=arr[:, 2].astype(np.int16), az=arr[:, 3].astype(np.int16),
        gx=arr[:, 4].astype(np.int16), gy=arr[:, 5].astype(np.int16), gz=arr[:, 6].astype(np.int16),
        marker_raw=arr[:, 7].astype(np.uint8),
        sample_hz=sample_hz,
        t0_ms=t0_ms,
        source_path=str(path),
    )


def parse_binary(path: str, sample_hz: Optional[int] = None) -> Capture:
    with open(path, "rb") as f:
        data = f.read()

    t0_ms = None
    if data[:1] == b"#":
        newline = data.index(b"\n")
        header_line = data[:newline].decode("ascii", errors="replace").strip()
        m = _CSV_HEADER_RE.match(header_line)
        if m:
            sample_hz = int(m.group("sample_hz"))
            t0_ms = int(m.group("t0_ms"))
        data = data[newline + 1:]

    n = len(data) // _BINARY_RECORD_SIZE
    if n * _BINARY_RECORD_SIZE != len(data):
        raise ValueError(
            f"{path}: {len(data)} bytes is not a whole number of "
            f"{_BINARY_RECORD_SIZE}-byte records"
        )

    t_ms = np.empty(n, dtype=np.int64)
    ax = np.empty(n, dtype=np.int16); ay = np.empty(n, dtype=np.int16); az = np.empty(n, dtype=np.int16)
    gx = np.empty(n, dtype=np.int16); gy = np.empty(n, dtype=np.int16); gz = np.empty(n, dtype=np.int16)
    marker = np.empty(n, dtype=np.uint8)

    for i in range(n):
        off = i * _BINARY_RECORD_SIZE
        (t_ms[i], ax[i], ay[i], az[i], gx[i], gy[i], gz[i], m_) = struct.unpack_from(
            _BINARY_RECORD_FMT, data, off
        )
        marker[i] = m_ & 0xFF

    return Capture(
        t_ms=t_ms, ax=ax, ay=ay, az=az, gx=gx, gy=gy, gz=gz, marker_raw=marker,
        sample_hz=sample_hz, t0_ms=t0_ms, source_path=str(path),
    )


def parse_capture(path: str, fmt: str = "auto") -> Capture:
    p = Path(path)
    if fmt == "csv" or (fmt == "auto" and p.suffix.lower() == ".csv"):
        return parse_csv(path)
    if fmt == "binary" or (fmt == "auto" and p.suffix.lower() in (".bin", ".dat")):
        return parse_binary(path)
    if fmt == "auto":
        with open(path, "rb") as f:
            head = f.read(64)
        try:
            head.decode("ascii")
            return parse_csv(path)
        except UnicodeDecodeError:
            return parse_binary(path)
    raise ValueError(f"unknown format: {fmt!r}")


def plot_capture(cap: Capture, out_path: Optional[str] = None, show: bool = True) -> None:
    import matplotlib.pyplot as plt

    t_s = (cap.t_ms - cap.t_ms[0]) / 1000.0
    accel_mag = cap.accel_mag_g
    gyro_mag = cap.gyro_mag_dps
    markers = cap.markers

    fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=(12, 6))

    ax1.plot(t_s, accel_mag, linewidth=0.8, color="tab:blue")
    ax1.set_ylabel("|accel| (g)")
    ax1.set_title(cap.source_path or "capture")

    ax2.plot(t_s, gyro_mag, linewidth=0.8, color="tab:orange")
    ax2.set_ylabel("|gyro| (dps)")
    ax2.set_xlabel("time (s)")

    for m in markers:
        m_t_s = (m.t_ms - cap.t_ms[0]) / 1000.0
        color = "green" if m.type == MARKER_BITE else "red"
        label = "bite" if m.type == MARKER_BITE else "false trigger"
        for ax in (ax1, ax2):
            ax.axvline(m_t_s, color=color, linestyle="--", linewidth=1, alpha=0.8)
        ax1.annotate(label, (m_t_s, ax1.get_ylim()[1]), color=color,
                     fontsize=7, rotation=90, va="top", ha="right")

    fig.tight_layout()
    if out_path:
        fig.savefig(out_path, dpi=150)
    if show:
        plt.show()
    else:
        plt.close(fig)


@dataclass
class Sample:
    t_ms: int
    ax: int; ay: int; az: int
    gx: int; gy: int; gz: int
    accel_g: tuple[float, float, float]
    gyro_dps: tuple[float, float, float]
    accel_mag_g: float
    gyro_mag_dps: float


DetectorFn = Callable[[Sample, object], tuple[bool, object]]


def replay(detector_fn: DetectorFn, cap: Capture) -> list[int]:
    state: object = None
    detections: list[int] = []
    n = len(cap)
    accel_g = cap.accel_g
    gyro_dps = cap.gyro_dps
    accel_mag = cap.accel_mag_g
    gyro_mag = cap.gyro_mag_dps

    for i in range(n):
        sample = Sample(
            t_ms=int(cap.t_ms[i]),
            ax=int(cap.ax[i]), ay=int(cap.ay[i]), az=int(cap.az[i]),
            gx=int(cap.gx[i]), gy=int(cap.gy[i]), gz=int(cap.gz[i]),
            accel_g=tuple(accel_g[i]), gyro_dps=tuple(gyro_dps[i]),
            accel_mag_g=float(accel_mag[i]), gyro_mag_dps=float(gyro_mag[i]),
        )
        fired, state = detector_fn(sample, state)
        if fired:
            detections.append(sample.t_ms)

    return detections


DEFAULT_WINDOW_BEFORE_MS = 15000
DEFAULT_WINDOW_AFTER_MS = 2000


@dataclass
class ScoreResult:
    window_before_ms: int
    window_after_ms: int
    n_bite_markers: int
    n_false_trigger_markers: int
    true_positives: int
    missed: list[int] = field(default_factory=list)
    fp_unexplained: list[int] = field(default_factory=list)
    fp_near_false_trigger: list[int] = field(default_factory=list)


def score(
    detections: list[int],
    markers: list[Marker],
    window_before_ms: int = DEFAULT_WINDOW_BEFORE_MS,
    window_after_ms: int = DEFAULT_WINDOW_AFTER_MS,
) -> ScoreResult:
    bite_markers = sorted((m for m in markers if m.type == MARKER_BITE), key=lambda m: m.t_ms)
    ft_markers = sorted((m for m in markers if m.type == MARKER_FALSE_TRIGGER), key=lambda m: m.t_ms)
    detections_sorted = sorted(detections)

    matched: set[int] = set()
    missed: list[int] = []
    tp = 0

    for m in bite_markers:
        lo, hi = m.t_ms - window_before_ms, m.t_ms + window_after_ms
        hit = next((d for d in detections_sorted if lo <= d <= hi and d not in matched), None)
        if hit is not None:
            matched.add(hit)
            tp += 1
        else:
            missed.append(m.t_ms)

    fp_unexplained: list[int] = []
    fp_near_ft: list[int] = []
    for d in detections_sorted:
        if d in matched:
            continue
        if any(m.t_ms - window_before_ms <= d <= m.t_ms + window_after_ms for m in ft_markers):
            fp_near_ft.append(d)
        else:
            fp_unexplained.append(d)

    return ScoreResult(
        window_before_ms=window_before_ms,
        window_after_ms=window_after_ms,
        n_bite_markers=len(bite_markers),
        n_false_trigger_markers=len(ft_markers),
        true_positives=tp,
        missed=missed,
        fp_unexplained=fp_unexplained,
        fp_near_false_trigger=fp_near_ft,
    )


def print_score_table(result: ScoreResult, detector_name: str = "") -> None:
    label = f" ({detector_name})" if detector_name else ""
    print(f"score{label}  window=-{result.window_before_ms}ms/+{result.window_after_ms}ms")
    print(f"  bite markers          : {result.n_bite_markers}")
    print(f"  true positives        : {result.true_positives}")
    print(f"  missed bites          : {len(result.missed)}  {result.missed}")
    print(f"  false positives (near known false-trigger) : {len(result.fp_near_false_trigger)}")
    print(f"  false positives (unexplained)               : {len(result.fp_unexplained)}")


def detect_amplitude(sample: Sample, state: object) -> tuple[bool, object]:
    JERK_SMOOTH_ALPHA = 0.30
    NOISE_ALPHA = 0.01
    NOISE_UPDATE_MAX = 3.0
    GYR_JERK_FLOOR = 0.30
    ACC_JERK_FLOOR = 0.02
    TRIG_SCORE = 9.0
    PERSIST_N = 3
    MUTE_MS = 3000

    RECAL_MIN_MS = 1500
    RECAL_MAX_MS = 10000
    RECAL_ALPHA = 0.20
    RECAL_STABLE_PCT = 0.05

    ACC_SMOOTH_ALPHA = 0.02
    COS_TILT_LIMIT = math.cos(math.radians(5.0))
    GYR_SETTLE_SCORE = 4.0
    ORIENT_HOLD_MS = 2000

    def reset_channels_and_timers(s, now_ms):
        s["gyr_prev"] = None; s["gyr_jerk"] = 0.0; s["gyr_noise"] = GYR_JERK_FLOOR
        s["acc_prev"] = None; s["acc_jerk"] = 0.0; s["acc_noise"] = ACC_JERK_FLOOR
        s["over_count"] = 0
        s["recal_active"] = True
        s["recal_start_ms"] = now_ms
        s["last_noise_check"] = 0.0
        s["next_noise_check_ms"] = now_ms + 500
        s["orient_off_since_ms"] = 0
        s["alert_until_ms"] = now_ms + RECAL_MAX_MS

    if state is None:
        state = {
            "acc_init": False, "sax": 0.0, "say": 0.0, "saz": 0.0,
            "ref_valid": False, "ref": (0.0, 0.0, 0.0),
        }
        reset_channels_and_timers(state, sample.t_ms)

    now = sample.t_ms
    ax_g, ay_g, az_g = sample.accel_g

    a_alpha = RECAL_ALPHA if state["recal_active"] else ACC_SMOOTH_ALPHA
    if not state["acc_init"]:
        state["sax"], state["say"], state["saz"] = ax_g, ay_g, az_g
        state["acc_init"] = True
    else:
        state["sax"] += a_alpha * (ax_g - state["sax"])
        state["say"] += a_alpha * (ay_g - state["say"])
        state["saz"] += a_alpha * (az_g - state["saz"])

    muted = now < state["alert_until_ms"]

    def update_channel(prev_key, jerk_key, noise_key, floor_val, mag):
        prev = state[prev_key]
        if prev is None:
            state[prev_key] = mag
            state[jerk_key] = 0.0
            return 0.0
        j = abs(mag - prev)
        state[prev_key] = mag
        state[jerk_key] += JERK_SMOOTH_ALPHA * (j - state[jerk_key])

        n_alpha = RECAL_ALPHA if state["recal_active"] else NOISE_ALPHA
        noise = state[noise_key]
        if state["recal_active"] or (not muted and state[jerk_key] < NOISE_UPDATE_MAX * noise):
            noise += n_alpha * (state[jerk_key] - noise)
            if noise < floor_val:
                noise = floor_val
            state[noise_key] = noise
        return state[jerk_key] / state[noise_key]

    gyr_score = update_channel("gyr_prev", "gyr_jerk", "gyr_noise", GYR_JERK_FLOOR, sample.gyro_mag_dps)
    acc_score = update_channel("acc_prev", "acc_jerk", "acc_noise", ACC_JERK_FLOOR, sample.accel_mag_g)

    def capture_orientation_ref():
        m = math.sqrt(state["sax"] ** 2 + state["say"] ** 2 + state["saz"] ** 2)
        if m < 0.5:
            return
        state["ref"] = (state["sax"] / m, state["say"] / m, state["saz"] / m)
        state["ref_valid"] = True

    if state["recal_active"]:
        if now >= state["next_noise_check_ms"]:
            delta = abs(state["gyr_noise"] - state["last_noise_check"])
            stable = (state["last_noise_check"] > 0) and (delta < RECAL_STABLE_PCT * state["last_noise_check"])
            state["last_noise_check"] = state["gyr_noise"]
            state["next_noise_check_ms"] = now + 500
            if stable and (now - state["recal_start_ms"]) >= RECAL_MIN_MS:
                state["recal_active"] = False
                state["alert_until_ms"] = now
                capture_orientation_ref()
        if state["recal_active"] and (now - state["recal_start_ms"]) >= RECAL_MAX_MS:
            state["recal_active"] = False
            state["alert_until_ms"] = now
            capture_orientation_ref()
        return False, state

    if state["ref_valid"]:
        m = math.sqrt(state["sax"] ** 2 + state["say"] ** 2 + state["saz"] ** 2)
        if m > 0.5:
            rx, ry, rz = state["ref"]
            dot = (state["sax"] / m) * rx + (state["say"] / m) * ry + (state["saz"] / m) * rz
            dot = max(-1.0, min(1.0, dot))
            tilted = dot < COS_TILT_LIMIT
            settled = gyr_score < GYR_SETTLE_SCORE
            if tilted and settled:
                if state["orient_off_since_ms"] == 0:
                    state["orient_off_since_ms"] = now
                elif now - state["orient_off_since_ms"] >= ORIENT_HOLD_MS:
                    reset_channels_and_timers(state, now)
                    return False, state
            else:
                state["orient_off_since_ms"] = 0

    if muted:
        state["over_count"] = 0
        return False, state

    combined = gyr_score + acc_score
    state["over_count"] = state["over_count"] + 1 if combined > TRIG_SCORE else 0

    if state["over_count"] >= PERSIST_N:
        state["over_count"] = 0
        state["alert_until_ms"] = now + MUTE_MS
        return True, state

    return False, state


def detect_highpass(sample: Sample, state: object) -> tuple[bool, object]:
    ALPHA = 0.85
    GYR_THRESH_DPS = 40.0
    ACC_THRESH_G = 0.15
    PERSIST_N = 2
    MUTE_MS = 1000

    if state is None:
        state = {
            "gyr_prev": None, "acc_prev": None,
            "gyr_hp": 0.0, "acc_hp": 0.0,
            "over_count": 0, "mute_until_ms": sample.t_ms,
        }

    if state["gyr_prev"] is None:
        state["gyr_prev"] = sample.gyro_mag_dps
        state["acc_prev"] = sample.accel_mag_g
        return False, state

    state["gyr_hp"] = ALPHA * (state["gyr_hp"] + sample.gyro_mag_dps - state["gyr_prev"])
    state["acc_hp"] = ALPHA * (state["acc_hp"] + sample.accel_mag_g - state["acc_prev"])
    state["gyr_prev"] = sample.gyro_mag_dps
    state["acc_prev"] = sample.accel_mag_g

    muted = sample.t_ms < state["mute_until_ms"]
    over = abs(state["gyr_hp"]) > GYR_THRESH_DPS or abs(state["acc_hp"]) > ACC_THRESH_G

    if muted or not over:
        state["over_count"] = 0
        return False, state

    state["over_count"] += 1
    if state["over_count"] >= PERSIST_N:
        state["over_count"] = 0
        state["mute_until_ms"] = sample.t_ms + MUTE_MS
        return True, state

    return False, state


def detect_jerk(sample: Sample, state: object) -> tuple[bool, object]:
    GYR_RATE_THRESH_DPS_PER_S = 4000.0
    ACC_RATE_THRESH_G_PER_S = 6.0
    PERSIST_N = 2
    MUTE_MS = 1000

    if state is None:
        state = {
            "prev_t_ms": None, "prev_gyr": None, "prev_acc": None,
            "over_count": 0, "mute_until_ms": sample.t_ms,
        }

    if state["prev_t_ms"] is None:
        state["prev_t_ms"] = sample.t_ms
        state["prev_gyr"] = sample.gyro_mag_dps
        state["prev_acc"] = sample.accel_mag_g
        return False, state

    dt_s = max(1, sample.t_ms - state["prev_t_ms"]) / 1000.0
    gyr_rate = abs(sample.gyro_mag_dps - state["prev_gyr"]) / dt_s
    acc_rate = abs(sample.accel_mag_g - state["prev_acc"]) / dt_s
    state["prev_t_ms"] = sample.t_ms
    state["prev_gyr"] = sample.gyro_mag_dps
    state["prev_acc"] = sample.accel_mag_g

    muted = sample.t_ms < state["mute_until_ms"]
    over = gyr_rate > GYR_RATE_THRESH_DPS_PER_S or acc_rate > ACC_RATE_THRESH_G_PER_S

    if muted or not over:
        state["over_count"] = 0
        return False, state

    state["over_count"] += 1
    if state["over_count"] >= PERSIST_N:
        state["over_count"] = 0
        state["mute_until_ms"] = sample.t_ms + MUTE_MS
        return True, state

    return False, state


def detect_periodicity_reject(sample: Sample, state: object) -> tuple[bool, object]:
    CANDIDATE_GYR_RATE_THRESH_DPS_PER_S = 4000.0
    PERSIST_N = 2
    MUTE_MS = 1000
    WINDOW_S = 6.0
    MIN_LAG_S = 1.0
    MAX_LAG_S = 5.0
    CORR_THRESH = 0.5
    GRID_HZ = 50.0

    if state is None:
        state = {
            "buf_t": deque(), "buf_v": deque(),
            "prev_t_ms": None, "prev_gyr": None,
            "over_count": 0, "mute_until_ms": sample.t_ms,
        }

    state["buf_t"].append(sample.t_ms)
    state["buf_v"].append(sample.gyro_mag_dps)
    cutoff = sample.t_ms - WINDOW_S * 1000
    while state["buf_t"] and state["buf_t"][0] < cutoff:
        state["buf_t"].popleft()
        state["buf_v"].popleft()

    if state["prev_t_ms"] is None:
        state["prev_t_ms"] = sample.t_ms
        state["prev_gyr"] = sample.gyro_mag_dps
        return False, state

    dt_s = max(1, sample.t_ms - state["prev_t_ms"]) / 1000.0
    gyr_rate = abs(sample.gyro_mag_dps - state["prev_gyr"]) / dt_s
    state["prev_t_ms"] = sample.t_ms
    state["prev_gyr"] = sample.gyro_mag_dps

    muted = sample.t_ms < state["mute_until_ms"]
    candidate = gyr_rate > CANDIDATE_GYR_RATE_THRESH_DPS_PER_S

    if muted or not candidate:
        state["over_count"] = 0
        return False, state

    state["over_count"] += 1
    if state["over_count"] < PERSIST_N:
        return False, state
    state["over_count"] = 0

    if len(state["buf_v"]) < 20:
        state["mute_until_ms"] = sample.t_ms + MUTE_MS
        return True, state

    times = np.asarray(state["buf_t"], dtype=float)
    vals = np.asarray(state["buf_v"], dtype=float)
    span_s = (times[-1] - times[0]) / 1000.0
    if span_s < MIN_LAG_S:
        state["mute_until_ms"] = sample.t_ms + MUTE_MS
        return True, state

    vals = vals - vals.mean()
    uniform_t = np.arange(times[0], times[-1], 1000.0 / GRID_HZ)
    if len(uniform_t) < 20:
        state["mute_until_ms"] = sample.t_ms + MUTE_MS
        return True, state
    uniform_v = np.interp(uniform_t, times, vals)

    lag_lo = int(MIN_LAG_S * GRID_HZ)
    lag_hi = min(int(MAX_LAG_S * GRID_HZ), len(uniform_v) - 1)
    best_corr = 0.0
    for lag in range(lag_lo, lag_hi):
        a = uniform_v[:-lag]
        b = uniform_v[lag:]
        denom = np.sqrt(np.dot(a, a) * np.dot(b, b))
        if denom < 1e-9:
            continue
        corr = np.dot(a, b) / denom
        if corr > best_corr:
            best_corr = corr

    if best_corr > CORR_THRESH:
        return False, state

    state["mute_until_ms"] = sample.t_ms + MUTE_MS
    return True, state


DETECTORS: dict[str, DetectorFn] = {
    "amplitude": detect_amplitude,
    "highpass": detect_highpass,
    "jerk": detect_jerk,
    "periodicity_reject": detect_periodicity_reject,
}


def main(argv: Optional[list[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("capture", help="path to a .csv (or .bin) capture file")
    ap.add_argument("--format", choices=["auto", "csv", "binary"], default="auto")
    ap.add_argument("--plot", action="store_true", help="show accel/gyro/marker plot")
    ap.add_argument("--save-plot", metavar="PATH", help="save plot to PATH instead of/as well as showing it")
    ap.add_argument("--replay", metavar="DETECTOR", choices=sorted(DETECTORS),
                     help="run a detector and print its score table")
    ap.add_argument("--window-before-ms", type=int, default=DEFAULT_WINDOW_BEFORE_MS,
                     help=f"how far before a marker to search for a matching detection "
                          f"(default {DEFAULT_WINDOW_BEFORE_MS})")
    ap.add_argument("--window-after-ms", type=int, default=DEFAULT_WINDOW_AFTER_MS,
                     help=f"how far after a marker to search for a matching detection "
                          f"(default {DEFAULT_WINDOW_AFTER_MS})")
    args = ap.parse_args(argv)

    cap = parse_capture(args.capture, fmt=args.format)
    print(f"loaded {len(cap)} samples from {args.capture} "
          f"({len(cap.markers)} markers, sample_hz={cap.sample_hz})")

    if args.plot or args.save_plot:
        plot_capture(cap, out_path=args.save_plot, show=args.plot)

    if args.replay:
        detector_fn = DETECTORS[args.replay]
        detections = replay(detector_fn, cap)
        result = score(detections, cap.markers,
                        window_before_ms=args.window_before_ms,
                        window_after_ms=args.window_after_ms)
        print_score_table(result, detector_name=args.replay)

    return 0


if __name__ == "__main__":
    sys.exit(main())
