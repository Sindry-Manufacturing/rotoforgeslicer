"""FRAM process-window CSV -> operating point. SPEC §5.

Reads the screener "Export CSV", keeps stable (pass) cells, and picks a constant
revs/mm ray (n_over_v). Each cell fully determines (RPM, traverse, wire feed).
"""
from __future__ import annotations

import csv
from dataclasses import dataclass


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

    def rpm_for(self, v_mm_min: float) -> int:
        """RPM that holds the ray's revs/mm at traverse v."""
        return round(self.revs_per_mm * v_mm_min)

    @property
    def v_grind_floor_mm_min(self) -> float:
        return self.v_min_mm_min


def _truthy(s) -> bool:
    return str(s).strip().lower() in ("1", "true", "yes", "y", "pass", "stable", "t")


def load_rows(csv_path: str) -> list[dict]:
    with open(csv_path, newline="") as f:
        return list(csv.DictReader(f))


def stable_rows(rows) -> list[dict]:
    return [r for r in rows if _truthy(r.get("pass", ""))]


def select_operating_point(csv_path: str, mode: str = "auto",
                           target: float = 0.0, tol: float = 5.0) -> OperatingPoint:
    rows = stable_rows(load_rows(csv_path))
    if not rows:
        raise ValueError("no stable (pass) rows in screener CSV")

    def nv(r):
        return float(r["n_over_v"])

    def trav(r):
        return float(r["traverse_mm_min"])

    if mode == "manual":
        sel = [r for r in rows if abs(nv(r) - target) <= tol]
        if not sel:
            raise ValueError(f"no stable cells within {tol} of revs/mm={target}")
        nv_star = target
    else:  # auto: the revs/mm ray whose stable cells span the widest traverse range
        best = None
        for r0 in rows:
            band = [r for r in rows if abs(nv(r) - nv(r0)) <= tol]
            span = max(trav(x) for x in band) - min(trav(x) for x in band)
            if best is None or span > best[0]:
                best = (span, nv(r0), band)
        _, nv_star, sel = best

    v_min = min(trav(r) for r in sel)
    v_max = max(trav(r) for r in sel)
    mid = 0.5 * (v_min + v_max)
    rep = min(sel, key=lambda r: abs(trav(r) - mid))
    return OperatingPoint(
        revs_per_mm=nv_star, v_min_mm_min=v_min, v_max_mm_min=v_max,
        rpm=int(round(float(rep["rpm"]))), traverse_mm_min=trav(rep),
        feed_speed_mm_min=float(rep["feed_speed_mm_min"]),
        phi=float(rep["feed_ratio_phi"]), torque_Nm=float(rep["torque_Nm"]),
        power_kW=float(rep["power_kW"]), t_az_c=float(rep["T_AZ_C"]),
    )
