"""FRAM process-window CSV -> operating point. SPEC §5.

Reads the screener "Export CSV", keeps stable (pass) cells, and picks a constant
revs/mm ray (n_over_v). Each cell fully determines (RPM, traverse, wire feed).
"""
from __future__ import annotations

import csv
from dataclasses import dataclass
from typing import Optional

# Columns the slicer actually consumes (SPEC §5.1); the export has ~37 columns and
# the rest are kept only for diagnostics.
REQUIRED_COLUMNS = (
    "rpm", "traverse_mm_min", "pass", "n_over_v", "feed_speed_mm_min",
    "feed_ratio_phi", "T_AZ_C", "torque_Nm", "power_kW",
)


@dataclass
class OperatingPoint:
    revs_per_mm: float          # n_over_v = rpm / traverse  [rev/mm]
    v_min_mm_min: float
    v_max_mm_min: float
    # representative cell:
    rpm: int
    traverse_mm_min: float
    feed_speed_mm_min: float    # wire feed
    phi: float
    torque_Nm: float
    power_kW: float
    t_az_c: float
    # SuperPID spindle window (SPEC §1.3); set from config at selection time.
    rpm_min: Optional[int] = None
    rpm_max: Optional[int] = None

    def rpm_for(self, v_mm_min: float) -> int:
        """RPM that holds the ray's revs/mm at traverse v, clamped to the SuperPID
        window when known (SPEC §5.2 step 4)."""
        rpm = round(self.revs_per_mm * v_mm_min)
        if self.rpm_min is not None:
            rpm = max(self.rpm_min, rpm)
        if self.rpm_max is not None:
            rpm = min(self.rpm_max, rpm)
        return rpm

    @property
    def v_grind_floor_mm_min(self) -> float:
        return self.v_min_mm_min

    def summary(self) -> str:
        """One-line operating-point read-out (SPEC §9)."""
        return (
            f"operating point: revs/mm={self.revs_per_mm:.1f} ray, "
            f"v=[{self.v_min_mm_min:.0f},{self.v_max_mm_min:.0f}] mm/min, "
            f"nominal v={self.traverse_mm_min:.0f} -> RPM={self.rpm}, "
            f"wire feed={self.feed_speed_mm_min:.0f} mm/min, Phi={self.phi:.2f}, "
            f"torque={self.torque_Nm:.2f} Nm, power={self.power_kW:.2f} kW, "
            f"T_AZ={self.t_az_c:.0f} C")


def _truthy(s) -> bool:
    return str(s).strip().lower() in ("1", "true", "yes", "y", "pass", "stable", "t")


def load_rows(csv_path: str) -> list[dict]:
    with open(csv_path, newline="") as f:
        reader = csv.DictReader(f)
        missing = [c for c in REQUIRED_COLUMNS if c not in (reader.fieldnames or [])]
        if missing:
            raise ValueError(
                f"screener CSV {csv_path} missing required column(s): {missing} "
                f"(needs at least {list(REQUIRED_COLUMNS)})")
        return list(reader)


def stable_rows(rows) -> list[dict]:
    return [r for r in rows if _truthy(r.get("pass", ""))]


def _nv(r):
    return float(r["n_over_v"])


def _trav(r):
    return float(r["traverse_mm_min"])


def _widest_contiguous_run(ray_rows: list) -> list:
    """Longest run of CONSECUTIVE stable cells along a revs/mm ray, by traverse span.

    ``ray_rows`` must be every cell on the ray (stable AND unstable) so that an
    unstable cell breaks a run — that is what makes the surviving range *contiguous*
    (SPEC §5.2), not just min..max over the stable cells.
    """
    rows = sorted(ray_rows, key=_trav)

    def span(run):
        return _trav(run[-1]) - _trav(run[0]) if run else -1.0

    best: list = []
    cur: list = []
    for r in rows:
        if _truthy(r.get("pass", "")):
            cur.append(r)
            if span(cur) > span(best):
                best = list(cur)
        else:
            cur = []  # an unstable cell breaks contiguity
    return best


def select_operating_point(csv_path: str, mode: str = "auto",
                           target: float = 0.0, tol: float = 5.0,
                           rpm_min: Optional[int] = None,
                           rpm_max: Optional[int] = None) -> OperatingPoint:
    """Pick the operating point. ``rpm_min``/``rpm_max`` (the SuperPID window, from
    config) are recorded on the result and reject a representative cell whose RPM is
    out of range (SPEC §5.2 step 4 / §1.3)."""
    all_rows = load_rows(csv_path)
    stable = stable_rows(all_rows)
    if not stable:
        raise ValueError("no stable (pass) rows in screener CSV")

    if mode == "manual":
        ray = [r for r in all_rows if abs(_nv(r) - target) <= tol]
        sel = _widest_contiguous_run(ray)  # contiguous so the band has no unstable gap
        if not sel:
            raise ValueError(f"no contiguous stable run within {tol} of revs/mm={target}")
        nv_star = target
    else:  # auto: the revs/mm ray with the widest CONTIGUOUS stable traverse run
        best = None  # (span, nv_star, run)
        for r0 in stable:
            nv0 = _nv(r0)
            ray = [r for r in all_rows if abs(_nv(r) - nv0) <= tol]
            run = _widest_contiguous_run(ray)
            if not run:
                continue
            span = _trav(run[-1]) - _trav(run[0])
            if best is None or span > best[0]:
                best = (span, nv0, run)
        if best is None:
            raise ValueError("no contiguous stable run found in screener CSV")
        _, nv_star, sel = best

    v_min = min(_trav(r) for r in sel)
    v_max = max(_trav(r) for r in sel)
    mid = 0.5 * (v_min + v_max)
    rep = min(sel, key=lambda r: abs(_trav(r) - mid))
    rpm = int(round(float(rep["rpm"])))
    if rpm_min is not None and rpm < rpm_min or rpm_max is not None and rpm > rpm_max:
        raise ValueError(
            f"operating point RPM {rpm} outside the SuperPID window "
            f"[{rpm_min},{rpm_max}] (SPEC §1.3) — infeasible screener cell")
    return OperatingPoint(
        revs_per_mm=nv_star, v_min_mm_min=v_min, v_max_mm_min=v_max,
        rpm=rpm, traverse_mm_min=_trav(rep),
        feed_speed_mm_min=float(rep["feed_speed_mm_min"]),
        phi=float(rep["feed_ratio_phi"]), torque_Nm=float(rep["torque_Nm"]),
        power_kW=float(rep["power_kW"]), t_az_c=float(rep["T_AZ_C"]),
        rpm_min=rpm_min, rpm_max=rpm_max,
    )
