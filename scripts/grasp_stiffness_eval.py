#!/usr/bin/env python3
"""Grasp stiffness sweep for the tendon-compliant gripper.

Sweeps the pad friction coefficient mu and reports, per configuration:
  - wrench closure (can the grasp resist an arbitrary external wrench)
  - slip margin (how close the tangential load is to the friction cone)
  - tendon tension headroom against the 45 N rated limit

Reads the calibration capture so the force model matches measured hardware.
"""

import csv
import math
from dataclasses import dataclass
from pathlib import Path

TENDON_TENSION_LIMIT_N = 45.0
PAD_FRICTION_DEFAULT = 0.6
CALIBRATION = Path(__file__).resolve().parent.parent / "data" / "flex_force_calibration.csv"


@dataclass
class CalibrationPoint:
    pull_mm: float
    current_ma: float
    distal_force_n: float
    proximal_force_n: float


def load_calibration(path: Path) -> list[CalibrationPoint]:
    points: list[CalibrationPoint] = []
    with path.open(newline="", encoding="utf-8") as fh:
        for row in csv.DictReader(fh):
            points.append(
                CalibrationPoint(
                    pull_mm=float(row["tendon_pull_mm"]),
                    current_ma=float(row["motor_current_ma"]),
                    distal_force_n=float(row["distal_force_n"]),
                    proximal_force_n=float(row["proximal_force_n"]),
                )
            )
    return points


def fit_force_gain(points: list[CalibrationPoint]) -> float:
    """Least-squares N per mm of tendon pull, averaged over both phalanges."""
    if not points:
        return 0.0
    num = 0.0
    den = 0.0
    for p in points:
        force = 0.5 * (p.distal_force_n + p.proximal_force_n)
        num += p.pull_mm * force
        den += p.pull_mm * p.pull_mm
    return num / den if den > 1e-9 else 0.0


def wrench_closure(friction: float, gain: float, pull_mm: float) -> bool:
    """A 3-finger grasp closes the wrench space if the summed friction cones
    span the plane. Approximated by requiring the mean cone half-angle to
    exceed the 30 deg span gap between adjacent fingers."""
    half_angle = math.atan(friction)
    contact_force = gain * pull_mm
    if contact_force < 1.0:
        return False
    return half_angle >= math.radians(30.0)


def slip_margin(friction: float, tangential_n: float, normal_n: float) -> float:
    """Signed margin to the friction cone boundary. Positive = no slip."""
    if normal_n <= 1e-6:
        return -1.0
    return friction - (tangential_n / normal_n)


def tendon_headroom(gain: float, pull_mm: float) -> float:
    """Fraction of the rated tendon tension still available."""
    tension = gain * pull_mm
    return max(0.0, 1.0 - tension / TENDON_TENSION_LIMIT_N)


def evaluate(points: list[CalibrationPoint], mu_values: list[float]) -> list[dict]:
    gain = fit_force_gain(points)
    results = []
    for mu in mu_values:
        closures = 0
        worst_margin = 1.0
        min_headroom = 1.0
        for pull in (4.0, 8.0, 12.0, 16.0, 20.0):
            if wrench_closure(mu, gain, pull):
                closures += 1
            normal = gain * pull
            tangential = 0.6 * normal  # pad preload bias from the pad geometry
            worst_margin = min(worst_margin, slip_margin(mu, tangential, normal))
            min_headroom = min(min_headroom, tendon_headroom(gain, pull))
        results.append(
            {
                "mu": mu,
                "gain_n_per_mm": round(gain, 4),
                "closure_cases": closures,
                "worst_slip_margin": round(worst_margin, 4),
                "min_tendon_headroom": round(min_headroom, 4),
            }
        )
    return results


def main() -> None:
    points = load_calibration(CALIBRATION)
    mu_values = [0.20, 0.30, 0.40, 0.50, 0.60, 0.70, 0.80]
    rows = evaluate(points, mu_values)

    print(f"{'mu':>5} {'gain':>8} {'closure':>8} {'slip_m':>8} {'headroom':>9}")
    for r in rows:
        print(
            f"{r['mu']:>5.2f} {r['gain_n_per_mm']:>8.4f} "
            f"{r['closure_cases']:>8d} {r['worst_slip_margin']:>8.4f} "
            f"{r['min_tendon_headroom']:>9.4f}"
        )

    failing = [r for r in rows if r["closure_cases"] < 3 or r["worst_slip_margin"] < 0.0]
    if failing:
        print("\nconfigs below closure/slip target:")
        for r in failing:
            print(f"  mu={r['mu']:.2f}")
    else:
        print("\nall sampled mu values meet closure and slip targets")


if __name__ == "__main__":
    main()
