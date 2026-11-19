"""Comprobaciones del generador de marcha (sin hardware).

Mide tres comportamientos que se corrigieron:

1. Altura de despeje del pie (antes se sumaba dos veces la curva Z).
2. Giro en el sitio (antes el yaw se anulaba si no habia velocidad lineal).
3. Comandos laterales lentos (antes se convertian en avance).

Uso:
    python test/gait_regression.py
    python test/gait_regression.py --legacy-ref 6623a53   # compara antes/despues

El ref legacy debe ser un commit que aun tenga la version antigua de
src/gait/bezier_gait.py y src/control/gait_interface.py.
"""

from __future__ import annotations

import argparse
import importlib
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from src.control.gait_interface import GaitInteface  # noqa: E402


# --------------------------------------------------------------------- legacy
def load_legacy(ref: str):
    """Carga la version antigua desde git en un modulo temporal."""
    def git_show(path: str) -> str:
        return subprocess.check_output(
            ["git", "show", f"{ref}:{path}"], cwd=REPO_ROOT, text=True
        )

    bezier = git_show("src/gait/bezier_gait.py")
    interface = git_show("src/control/gait_interface.py").replace(
        "from src.gait.bezier_gait import BazierGait",
        "from legacy_bezier_gait import BazierGait",
    )

    tmp = Path(tempfile.mkdtemp(prefix="gait_legacy_"))
    (tmp / "legacy_bezier_gait.py").write_text(bezier, encoding="utf-8")
    (tmp / "legacy_gait_interface.py").write_text(interface, encoding="utf-8")

    sys.path.insert(0, str(tmp))
    return importlib.import_module("legacy_gait_interface").GaitInteface


# -------------------------------------------------------------------- medidas
def simulate(impl, vx: float, vy: float, yaw: float, steps: int, dt: float = 0.02):
    gait = impl(dt=dt)
    base = {key: value.copy() for key, value in gait.T_bf_base.items()}
    traj = []
    for _ in range(steps):
        T_bf = gait.compute_gait(vx, vy, yaw)
        traj.append({key: T_bf[key][:3, 3].astype(float).copy() for key in T_bf})
    return base, traj


def metrics(impl) -> dict:
    out: dict[str, float] = {}

    # 1) Marcha recta: altura de despeje
    base, traj = simulate(impl, 0.07, 0.0, 0.0, steps=400)
    out["lift_mm"] = 1000.0 * max(p["FL"][2] - base["FL"][2, 3] for p in traj)

    # Continuidad: mayor salto entre frames consecutivos de FL
    jumps = [
        np.linalg.norm(traj[i]["FL"] - traj[i - 1]["FL"])
        for i in range(1, len(traj))
    ]
    out["jump_mm"] = 1000.0 * max(jumps)

    # 2) Yaw puro (girar en el sitio)
    base, traj = simulate(impl, 0.0, 0.0, 0.5, steps=400)
    exc, drift, moved = [], [], []
    for leg, T in base.items():
        radius = float(np.hypot(T[0, 3], T[1, 3]))
        angle0 = float(np.arctan2(T[1, 3], T[0, 3]))
        angles = [np.arctan2(p[leg][1], p[leg][0]) for p in traj]
        exc.append(max(abs(a - angle0) for a in angles))
        drift.append(max(abs(np.hypot(p[leg][0], p[leg][1]) - radius) for p in traj))
        moved.append(max(np.linalg.norm(p[leg][:2] - T[:2, 3]) for p in traj))
    out["yaw_exc_rad"] = max(exc)
    out["yaw_drift_mm"] = 1000.0 * max(drift)
    out["yaw_moved_mm"] = 1000.0 * max(moved)

    # 3) Comando lateral lento: vx=0, vy=0.01
    base, traj = simulate(impl, 0.0, 0.01, 0.0, steps=60)
    deltas = [p["FL"][:2] - base["FL"][:2, 3] for p in traj]
    out["lat_dx_mm"] = 1000.0 * max(abs(d[0]) for d in deltas)
    out["lat_dy_mm"] = 1000.0 * max(abs(d[1]) for d in deltas)

    return out


# --------------------------------------------------------------------- salida
LABELS = {
    "lift_mm": "Altura de despeje FL (mm)",
    "jump_mm": "Salto maximo entre frames (mm)",
    "yaw_exc_rad": "Giro en sitio: excursion (rad)",
    "yaw_drift_mm": "Giro en sitio: deriva radial (mm)",
    "yaw_moved_mm": "Giro en sitio: desplazamiento (mm)",
    "lat_dx_mm": "Lateral lento: |dx| (mm)",
    "lat_dy_mm": "Lateral lento: |dy| (mm)",
}

CHECKS = [
    ("lift_mm", lambda v: 25.0 <= v <= 55.0, "25-55"),
    ("jump_mm", lambda v: v <= 40.0, "<=40"),
    ("yaw_exc_rad", lambda v: v >= 0.05, ">=0.05"),
    ("yaw_drift_mm", lambda v: v <= 5.0, "<=5"),
    ("lat_dx_mm", lambda v: v <= 0.5, "~0"),
    ("lat_dy_mm", lambda v: v >= 2.0, ">=2"),
]
TARGETS = {key: target for key, _, target in CHECKS}


def main() -> int:
    parser = argparse.ArgumentParser(description="Regresion del generador de marcha")
    parser.add_argument(
        "--legacy-ref",
        default=None,
        help="commit git con la version antigua para comparar (p.ej. 6623a53)",
    )
    args = parser.parse_args()

    current = metrics(GaitInteface)
    legacy = None
    if args.legacy_ref:
        try:
            legacy = metrics(load_legacy(args.legacy_ref))
        except Exception as exc:  # noqa: BLE001
            print(f"Aviso: no se pudo cargar la version antigua: {exc}")

    header = f"{'Metrica':38} {'Antes':>10} {'Ahora':>10}  {'Objetivo':>10}"
    print(header)
    print("-" * len(header))
    for key, label in LABELS.items():
        before = f"{legacy[key]:10.3f}" if legacy else f"{'-':>10}"
        target = TARGETS.get(key, "-")
        print(f"{label:38} {before} {current[key]:10.3f}  {target:>10}")

    print("\nComprobaciones (version actual):")
    ok = True
    for key, check, target in CHECKS:
        passed = check(current[key])
        ok &= passed
        print(f"  [{'OK' if passed else 'FALLA'}] {LABELS[key]} -> {current[key]:.3f} (objetivo {target})")

    print("\nRESULTADO:", "TODO OK" if ok else "HAY FALLOS")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
