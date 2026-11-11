"""
Grafico de las trayectorias de las patas generadas por el modulador de marcha.

Este script es independiente del resto de tests: simula el generador de
marcha de Bezier durante varios ciclos y dibuja el camino que describe el
pie de cada pata (FL, FR, RL, RR) respecto al cuerpo del robot.

Uso:
    python test/plot_leg_trajectory.py
    python test/plot_leg_trajectory.py --save trayectorias.png
    python test/plot_leg_trajectory.py --cycles 5 --vel 0.5 --no-show

Nota: la linea gruesa corresponde a la fase SWING (pie en el aire) y la
linea fina a la fase STANCE (pie en el suelo).
"""

import argparse
import copy
import os
import sys

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401  (registra la proyeccion 3d)

# Permite ejecutar el script directamente desde cualquier directorio.
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from src.gait.bezier_gait import BazierGait
from src.kinematics.robot_model import RobotModel


LEG_NAMES = ["FL", "FR", "RL", "RR"]
LEG_COLORS = {
    "FL": "#1f77b4",
    "FR": "#ff7f0e",
    "RL": "#2ca02c",
    "RR": "#d62728",
}
# En un trote FL/RR y FR/RL comparten fase, por lo que usamos estilos
# de linea distintos para que las trayectorias superpuestas se distingan.
LEG_STYLES = {
    "FL": "-",
    "FR": "--",
    "RL": "-.",
    "RR": ":",
}


def simulate_leg_trajectories(stride_length, vel, Tswing, clearance_height,
                              penetration_depth, cycles, dt=0.002):
    """
    Simula la marcha y devuelve la trayectoria del pie de cada pata.

    :param stride_length: Longitud total de la zancada (2 * L).
    :param vel: Velocidad lineal deseada del cuerpo en m/s.
    :param Tswing: Duracion de la fase swing en segundos.
    :param clearance_height: Altura de despeje del pie.
    :param penetration_depth: Profundidad de penetracion en stance.
    :param cycles: Numero de ciclos (strides) a simular.
    :param dt: Paso de integracion.

    :return: (tiempo, posiciones_abs, posiciones_rel, estados, pos_base, Tstance)
    """
    robot = RobotModel()
    gait = BazierGait(Tswing=Tswing, dt=dt)

    # L es la MITAD de la longitud de la zancada.
    L = stride_length / 2.0

    # Tstance segun la ecuacion del paper: Tstance = 2 * L / vel.
    Tstance = 2.0 * L / vel if vel > 0.0 else 0.0

    # El generador limita Tstance a 1.3 * Tswing, replicamos ese limite.
    if Tstance < dt:
        Tstance = 0.0
    elif Tstance > 1.3 * Tswing:
        Tstance = 1.3 * Tswing

    Tstride = Tstance + Tswing
    num_steps = max(int(cycles * Tstride / dt), 2)
    time_axis = np.arange(num_steps) * dt

    # Posicion neutra del pie de cada pata respecto al cuerpo.
    T_bf_base = copy.deepcopy(robot.WorldToFoot)
    pos_base = {
        key: T_bf_base[key][:3, 3].copy()
        for key in LEG_NAMES
    }

    pos_abs = {key: np.zeros((num_steps, 3)) for key in LEG_NAMES}
    states = {key: np.zeros(num_steps, dtype=int) for key in LEG_NAMES}

    for i in range(num_steps):
        T_bf = gait.GenerateTrajectory(
            L=L,
            LateralFraction=0.0,
            YawRate=0.0,
            vel=vel,
            T_bf_=T_bf_base,
            T_bf_curr=T_bf_base,
            clearance_height=clearance_height,
            penetration_depth=penetration_depth,
            contacts=[1, 1, 1, 1],
            dt=dt,
        )

        for j, key in enumerate(LEG_NAMES):
            pos_abs[key][i] = T_bf[key][:3, 3]
            # GetFootStep guarda la fase en gait.Phases: [0, 1] stance,
            # (1, 2] swing. La usamos para distinguir ambos estados.
            states[key][i] = (
                gait.SWING if gait.Phases[j] > 1.0 else gait.STANCE
            )

    pos_rel = {
        key: pos_abs[key] - pos_base[key]
        for key in LEG_NAMES
    }

    return time_axis, pos_abs, pos_rel, states, pos_base, Tstance


def _plot_colored(ax, x, y, states, color, label, z=None, lw=2.0,
                  linestyle="-"):
    """
    Dibuja una trayectoria resaltando la fase SWING.

    La linea fina muestra la trayectoria completa (STANCE + SWING) y la
    linea gruesa el tramo de SWING (pie en el aire).
    """
    swing = states == BazierGait.SWING

    if z is None:
        ax.plot(x, y, color=color, ls=linestyle, lw=lw * 0.6,
                alpha=0.4, label=label)
        if np.any(swing):
            ax.plot(
                np.where(swing, x, np.nan),
                np.where(swing, y, np.nan),
                color=color,
                ls=linestyle,
                lw=lw + 1.0,
                solid_capstyle="round",
            )
    else:
        ax.plot(x, y, z, color=color, ls=linestyle, lw=lw * 0.6,
                alpha=0.4, label=label)
        if np.any(swing):
            ax.plot(
                np.where(swing, x, np.nan),
                np.where(swing, y, np.nan),
                np.where(swing, z, np.nan),
                color=color,
                ls=linestyle,
                lw=lw + 1.0,
                solid_capstyle="round",
            )


def plot_leg_trajectories(time_axis, pos_abs, pos_rel, states, pos_base,
                          Tstance, Tswing, stride_length, clearance_height,
                          save_path=None, show=True):
    """
    Dibuja las trayectorias en 3D y en los planos XZ, XY y Z(t).
    """
    max_height = max(np.max(pos_rel[key][:, 2]) for key in LEG_NAMES)
    min_height = min(np.min(pos_rel[key][:, 2]) for key in LEG_NAMES)

    fig = plt.figure(figsize=(15, 10))
    fig.suptitle(
        "Trayectoria del pie de cada pata\n"
        f"zancada = {stride_length * 100:.1f} cm, "
        f"Tstance = {Tstance:.3f} s, Tswing = {Tswing:.3f} s, "
        f"clearance (param.) = {clearance_height * 100:.1f} cm | "
        f"altura de pie: {min_height * 100:.1f} a {max_height * 100:.1f} cm\n"
        "(linea gruesa = SWING, linea fina = STANCE; "
        "FL/RR y FR/RL comparten fase en trote)",
        fontsize=13,
    )

    # --- 1. Vista 3D en el marco del cuerpo (posiciones absolutas) ---
    ax3d = fig.add_subplot(221, projection="3d")
    for key in LEG_NAMES:
        xyz = pos_abs[key]
        _plot_colored(
            ax3d, xyz[:, 0], xyz[:, 1], states[key],
            LEG_COLORS[key], key, z=xyz[:, 2],
            linestyle=LEG_STYLES[key],
        )
        ax3d.scatter(*pos_base[key], marker="x", s=60,
                     color=LEG_COLORS[key], zorder=5)
    ax3d.set_title("Vista 3D (marco del cuerpo)")
    ax3d.set_xlabel("X [m]")
    ax3d.set_ylabel("Y [m]")
    ax3d.set_zlabel("Z [m]")
    ax3d.legend(loc="upper right")
    ax3d.view_init(elev=25, azim=-60)
    ax3d.locator_params(nbins=5)
    ax3d.set_box_aspect((1.0, 1.0, 0.7), zoom=1.25)

    # --- 2. Plano XZ: forma clasica de la trayectoria (desplazamiento) ---
    ax_xz = fig.add_subplot(222)
    for key in LEG_NAMES:
        rel = pos_rel[key]
        _plot_colored(
            ax_xz, rel[:, 0], rel[:, 2], states[key],
            LEG_COLORS[key], key,
            linestyle=LEG_STYLES[key],
        )
    ax_xz.set_title("Plano XZ (desplazamiento respecto al pie neutro)")
    ax_xz.set_xlabel("Desplazamiento X [m]")
    ax_xz.set_ylabel("Desplazamiento Z [m]")
    ax_xz.grid(True, alpha=0.3)
    ax_xz.axis("equal")
    ax_xz.legend(loc="upper right")

    # --- 3. Plano XY: vista superior en el marco del cuerpo ---
    ax_xy = fig.add_subplot(223)
    for key in LEG_NAMES:
        xyz = pos_abs[key]
        _plot_colored(
            ax_xy, xyz[:, 0], xyz[:, 1], states[key],
            LEG_COLORS[key], key,
            linestyle=LEG_STYLES[key],
        )
        ax_xy.scatter(*pos_base[key][:2], marker="x", s=60,
                      color=LEG_COLORS[key])
    ax_xy.set_title("Plano XY (marco del cuerpo, vista superior)")
    ax_xy.set_xlabel("X [m]")
    ax_xy.set_ylabel("Y [m]")
    ax_xy.grid(True, alpha=0.3)
    ax_xy.axis("equal")
    ax_xy.legend(loc="upper right")

    # --- 4. Altura del pie frente al tiempo ---
    ax_t = fig.add_subplot(224)
    for key in LEG_NAMES:
        rel = pos_rel[key]
        _plot_colored(
            ax_t, time_axis, rel[:, 2], states[key],
            LEG_COLORS[key], key,
            linestyle=LEG_STYLES[key],
        )
    ax_t.set_title("Altura del pie respecto al pie neutro")
    ax_t.set_xlabel("Tiempo [s]")
    ax_t.set_ylabel("Desplazamiento Z [m]")
    ax_t.grid(True, alpha=0.3)
    ax_t.legend(loc="upper right", ncol=2)

    fig.tight_layout(rect=(0, 0, 1, 0.94))

    if save_path:
        fig.savefig(save_path, dpi=150)
        print(f"Figura guardada en: {os.path.abspath(save_path)}")

    if show:
        plt.show()

    return fig


def main():
    parser = argparse.ArgumentParser(
        description="Grafica las trayectorias del pie generadas por BazierGait."
    )
    parser.add_argument("--stride-length", type=float, default=0.12,
                        help="Longitud total de la zancada en metros (def. 0.12)")
    parser.add_argument("--vel", type=float, default=0.4,
                        help="Velocidad lineal del cuerpo en m/s (def. 0.4)")
    parser.add_argument("--t-swing", type=float, default=0.25,
                        help="Duracion del swing en segundos (def. 0.25)")
    parser.add_argument("--clearance", type=float, default=0.04,
                        help="Altura de despeje en metros (def. 0.04)")
    parser.add_argument("--penetration", type=float, default=0.005,
                        help="Profundidad de penetracion en metros (def. 0.005)")
    parser.add_argument("--cycles", type=int, default=3,
                        help="Numero de ciclos de marcha a simular (def. 3)")
    parser.add_argument("--dt", type=float, default=0.002,
                        help="Paso de integracion en segundos (def. 0.002)")
    parser.add_argument("--save", type=str, default=None,
                        help="Ruta donde guardar la figura (ej. trayectorias.png)")
    parser.add_argument("--no-show", action="store_true",
                        help="No abrir la ventana interactiva")
    args = parser.parse_args()

    if args.vel <= 0.0:
        parser.error("--vel debe ser mayor que 0 para generar una marcha.")
    if args.cycles < 1:
        parser.error("--cycles debe ser al menos 1.")

    (time_axis, pos_abs, pos_rel, states, pos_base,
     Tstance) = simulate_leg_trajectories(
        stride_length=args.stride_length,
        vel=args.vel,
        Tswing=args.t_swing,
        clearance_height=args.clearance,
        penetration_depth=args.penetration,
        cycles=args.cycles,
        dt=args.dt,
    )

    print("Simulacion completada:")
    print(f"  L (mitad de zancada) = {args.stride_length / 2.0:.4f} m")
    print(f"  Tstance = {Tstance:.4f} s, Tswing = {args.t_swing:.4f} s")
    print(f"  Duracion simulada = {time_axis[-1]:.3f} s "
          f"({len(time_axis)} muestras)")

    plot_leg_trajectories(
        time_axis=time_axis,
        pos_abs=pos_abs,
        pos_rel=pos_rel,
        states=states,
        pos_base=pos_base,
        Tstance=Tstance,
        Tswing=args.t_swing,
        stride_length=args.stride_length,
        clearance_height=args.clearance,
        save_path=args.save,
        show=not args.no_show,
    )


if __name__ == "__main__":
    main()
