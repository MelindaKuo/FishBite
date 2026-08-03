from __future__ import annotations

import argparse
import math
import random
import sys
from dataclasses import dataclass, field as dc_field
from pathlib import Path
from typing import List, Tuple

sys.path.insert(0, str(Path(__file__).parent))
import analyze as az

SAMPLE_HZ = 100
DT_MS = 1000 // SAMPLE_HZ


@dataclass
class WindGust:
    start_s: float
    ramp_s: float
    hold_s: float
    decay_s: float
    peak_deg: float
    dir_xy: Tuple[float, float]
    marker_false_trigger: bool = False
    marker_lag_s: float = 3.0

    def tilt_deg(self, t: float) -> float:
        t0 = self.start_s
        if t < t0:
            return 0.0
        if t < t0 + self.ramp_s:
            return self.peak_deg * (t - t0) / self.ramp_s
        if t < t0 + self.ramp_s + self.hold_s:
            return self.peak_deg
        if t < t0 + self.ramp_s + self.hold_s + self.decay_s:
            frac = (t - (t0 + self.ramp_s + self.hold_s)) / self.decay_s
            return self.peak_deg * (1 - frac)
        return 0.0


@dataclass
class WaveWindow:
    start_s: float
    end_s: float
    freq_hz: float
    amp_deg: float
    dir_xy: Tuple[float, float]
    marker_false_trigger: bool = False
    marker_lag_s: float = 5.0

    def tilt_deg(self, t: float) -> float:
        if not (self.start_s <= t < self.end_s):
            return 0.0
        fade = min(1.0, (t - self.start_s) / 1.0, (self.end_s - t) / 1.0)
        return self.amp_deg * fade * math.sin(2 * math.pi * self.freq_hz * (t - self.start_s))


@dataclass
class BiteEvent:
    t_s: float
    gyro_peak_dps: float
    accel_peak_g: float
    dir_xy: Tuple[float, float]
    duration_s: float = 0.09
    marker_lag_s: float = 5.0

    def _envelope(self, t: float) -> float:
        dt = t - self.t_s
        if dt < 0 or dt > self.duration_s:
            return 0.0
        third = self.duration_s / 3
        if dt < third:
            return dt / third
        if dt < 2 * third:
            return 1.0
        return 1 - (dt - 2 * third) / third

    def gyro_dps(self, t: float) -> float:
        return self.gyro_peak_dps * self._envelope(t)

    def accel_g(self, t: float) -> float:
        return self.accel_peak_g * self._envelope(t)


@dataclass
class Scenario:
    duration_s: float
    tackle_weight_oz: float
    gusts: List[WindGust] = dc_field(default_factory=list)
    waves: List[WaveWindow] = dc_field(default_factory=list)
    bites: List[BiteEvent] = dc_field(default_factory=list)
    seed: int = 0


def weight_damping(weight_oz: float) -> float:
    return 1.0 / (1.0 + weight_oz / 4.0)


def default_scenario(tackle_weight_oz: float = 1.0, seed: int = 0) -> Scenario:
    rng = random.Random(seed)
    damp = weight_damping(tackle_weight_oz)

    def rand_dir() -> Tuple[float, float]:
        a = rng.uniform(0, 2 * math.pi)
        return (math.cos(a), math.sin(a))

    gusts = [
        WindGust(start_s=20, ramp_s=1.5, hold_s=6, decay_s=2, peak_deg=22 * damp,
                  dir_xy=rand_dir(), marker_false_trigger=True, marker_lag_s=3.5),
        WindGust(start_s=70, ramp_s=0.8, hold_s=3, decay_s=1.5, peak_deg=14 * damp,
                  dir_xy=rand_dir()),
        WindGust(start_s=140, ramp_s=2.0, hold_s=8, decay_s=3, peak_deg=25 * damp,
                  dir_xy=rand_dir(), marker_false_trigger=True, marker_lag_s=4.0),
    ]
    waves = [
        WaveWindow(start_s=40, end_s=65, freq_hz=0.5, amp_deg=10 * damp,
                    dir_xy=rand_dir(), marker_false_trigger=True, marker_lag_s=6.0),
        WaveWindow(start_s=100, end_s=130, freq_hz=0.35, amp_deg=8 * damp,
                    dir_xy=rand_dir()),
    ]
    bites = [
        BiteEvent(t_s=55.0, gyro_peak_dps=550, accel_peak_g=0.35, dir_xy=rand_dir(), marker_lag_s=4.0),
        BiteEvent(t_s=112.0, gyro_peak_dps=420, accel_peak_g=0.28, dir_xy=rand_dir(), marker_lag_s=7.5),
        BiteEvent(t_s=165.0, gyro_peak_dps=300, accel_peak_g=0.20, dir_xy=rand_dir(), marker_lag_s=3.0),
    ]
    return Scenario(duration_s=180.0, tackle_weight_oz=tackle_weight_oz,
                     gusts=gusts, waves=waves, bites=bites, seed=seed)


def _tilt_xy_deg(scenario: Scenario, t: float) -> Tuple[float, float]:
    tx = ty = 0.0
    for g in scenario.gusts:
        d = g.tilt_deg(t)
        tx += d * g.dir_xy[0]
        ty += d * g.dir_xy[1]
    for w in scenario.waves:
        d = w.tilt_deg(t)
        tx += d * w.dir_xy[0]
        ty += d * w.dir_xy[1]
    return tx, ty


def _sample_row(scenario: Scenario, t: float, t_ms: int, prev_tx: float, prev_ty: float,
                 dt: float, rng: random.Random, marker: int = 0) -> tuple:
    tx_deg, ty_deg = _tilt_xy_deg(scenario, t)

    gx_from_tilt = -(ty_deg - prev_ty) / dt
    gy_from_tilt = (tx_deg - prev_tx) / dt

    ax = math.sin(math.radians(tx_deg))
    ay = math.sin(math.radians(ty_deg))
    az_ = math.sqrt(max(0.0, 1.0 - ax * ax - ay * ay))
    gx, gy, gz = gx_from_tilt, gy_from_tilt, 0.0

    for b in scenario.bites:
        gmag = b.gyro_dps(t)
        amag = b.accel_g(t)
        if gmag or amag:
            gx += b.dir_xy[1] * gmag
            gy += -b.dir_xy[0] * gmag
            ax += b.dir_xy[0] * amag
            ay += b.dir_xy[1] * amag

    ax += rng.gauss(0, 0.005); ay += rng.gauss(0, 0.005); az_ += rng.gauss(0, 0.005)
    gx += rng.gauss(0, 0.5); gy += rng.gauss(0, 0.5); gz += rng.gauss(0, 0.5)

    return (
        t_ms,
        int(round(ax * az.ACC_LSB_PER_G)), int(round(ay * az.ACC_LSB_PER_G)), int(round(az_ * az.ACC_LSB_PER_G)),
        int(round(gx * az.GYRO_LSB_PER_DPS)), int(round(gy * az.GYRO_LSB_PER_DPS)), int(round(gz * az.GYRO_LSB_PER_DPS)),
        marker,
    ), tx_deg, ty_deg


def generate_csv_lines(scenario: Scenario, t0_ms: int = 1000) -> List[str]:
    rng = random.Random(scenario.seed + 1)
    n = int(scenario.duration_s * SAMPLE_HZ)
    dt = 1.0 / SAMPLE_HZ

    rows = []
    prev_tx = prev_ty = 0.0
    for i in range(n):
        t_ms = t0_ms + i * DT_MS
        t = i / SAMPLE_HZ
        row, prev_tx, prev_ty = _sample_row(scenario, t, t_ms, prev_tx, prev_ty, dt, rng)
        rows.append(row)

    marker_events: List[Tuple[float, int]] = []
    for g in scenario.gusts:
        if g.marker_false_trigger:
            marker_events.append((g.start_s + g.ramp_s + g.marker_lag_s, az.MARKER_FALSE_TRIGGER))
    for w in scenario.waves:
        if w.marker_false_trigger:
            marker_events.append((w.start_s + w.marker_lag_s, az.MARKER_FALSE_TRIGGER))
    for b in scenario.bites:
        marker_events.append((b.t_s + b.marker_lag_s, az.MARKER_BITE))

    marker_rng = random.Random(scenario.seed + 2)
    for t_marker, marker_type in marker_events:
        t_ms = t0_ms + int(t_marker * 1000)
        row, _, _ = _sample_row(scenario, t_marker, t_ms, *_tilt_xy_deg(scenario, t_marker - dt),
                                 dt, marker_rng, marker=marker_type)
        rows.append(row)

    rows.sort(key=lambda r: r[0])
    lines = [f"# FishBite log v1 sample_hz={SAMPLE_HZ} record_bytes=17 t0_ms={t0_ms}"]
    lines += [",".join(str(v) for v in r) for r in rows]
    return lines


def main(argv: List[str] | None = None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--tackle-weight-oz", type=float, default=1.0,
                     help="heavier -> less simulated wind/wave tip motion (default 1.0)")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("-o", "--out", default="examples/synthetic/session.csv")
    args = ap.parse_args(argv)

    scenario = default_scenario(tackle_weight_oz=args.tackle_weight_oz, seed=args.seed)
    lines = generate_csv_lines(scenario)

    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text("\n".join(lines) + "\n")

    print(f"wrote {out_path}")
    print(f"  duration_s={scenario.duration_s} tackle_weight_oz={args.tackle_weight_oz} "
          f"damping={weight_damping(args.tackle_weight_oz):.2f}")
    print(f"  {len(scenario.bites)} bites, {len(scenario.gusts)} gusts, "
          f"{len(scenario.waves)} wave windows")
    return 0


if __name__ == "__main__":
    sys.exit(main())
