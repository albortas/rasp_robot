"""
Figura explicativa de YawCircle (src/gait/bezier_gait.py).

Genera dos imagenes didacticas:

  1) yaw_circle_geometria.png:
     vista superior del robot y construccion geometrica de phi_arc (pata FR).
  2) yaw_circle_ecuaciones.png:
     ecuaciones implementadas, convencion de signos y tabla de ejemplo.

Uso:
    python test/plot_yaw_circle.py
    python test/plot_yaw_circle.py --show
    python test/plot_yaw_circle.py --g-mag 0.06
    python test/plot_yaw_circle.py --save-geometry geo.png --save-equations eq.png
"""

import argparse
import os
import sys

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Arc, Circle, FancyBboxPatch, Polygon

# Permite ejecutar el script directamente desde cualquier directorio.
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from src.gait.bezier_gait import BazierGait
from src.kinematics.robot_model import RobotModel
from src.kinematics.lib_algebra import TransToRp


LEG_NAMES = ["FL", "FR", "RL", "RR"]
LEG_COLORS = {
    "FL": "#1f77b4",
    "FR": "#ff7f0e",
    "RL": "#2ca02c",
    "RR": "#d62728",
}

COL_F = "#8e44ad"      # vector f = T_bf
COL_G = "#c0392b"      # vector g
COL_PHI = "#1e8449"    # angulo phi_arc
COL_TAN = "#7f8c8d"    # tangente
COL_AXIS = "#2c3e50"
COL_RAD = "#95a5a6"

DEFAULT_GEOMETRY = os.path.join(os.path.dirname(__file__), "yaw_circle_geometria.png")
DEFAULT_EQUATIONS = os.path.join(os.path.dirname(__file__), "yaw_circle_ecuaciones.png")


def unit(angle):
    return np.array([np.cos(angle), np.sin(angle)])


def arrow(ax, start, end, color, lw=2.2, ls="-", ms=14, zorder=5, alpha=1.0):
    ax.annotate(
        "",
        xy=end,
        xytext=start,
        arrowprops=dict(
            arrowstyle="-|>",
            color=color,
            lw=lw,
            linestyle=ls,
            shrinkA=0,
            shrinkB=0,
            mutation_scale=ms,
            alpha=alpha,
        ),
        zorder=zorder,
    )


def compute_data(g_mag):
    """
    Obtiene las posiciones nominales de las patas y calcula phi_arc con la
    funcion real YawCircle. Para ilustrar la geometria se usa una desviacion
    tangencial de magnitud g_mag.
    """
    robot = RobotModel()
    gait = BazierGait(Tswing=0.25, dt=0.02)
    base = robot.WorldToFoot

    feet = {}
    hips = {}
    thetas = {}
    phis = {}
    thetas_mod = {}

    for i, key in enumerate(LEG_NAMES):
        _, p = TransToRp(base[key])
        p = np.asarray(p, dtype=float)
        feet[key] = p

        _, hip = TransToRp(robot.WorldToHip[key])
        hips[key] = np.asarray(hip, dtype=float)

        theta = np.arctan2(p[1], p[0])
        thetas[key] = theta

        # Desviacion tangencial de ejemplo: g perpendicular al radio.
        tangent = theta + np.pi / 2.0
        g_vec = g_mag * np.array([np.cos(tangent), np.sin(tangent), 0.0])
        gait.Prev_fxyz[i] = p + g_vec

        phis[key] = gait.YawCircle(p, i)
        thetas_mod[key] = np.arctan2(g_mag, np.hypot(p[0], p[1]))

    return {
        "robot": robot,
        "feet": feet,
        "hips": hips,
        "thetas": thetas,
        "phis": phis,
        "thetas_mod": thetas_mod,
        "g_mag": g_mag,
        "r": np.hypot(feet["FL"][0], feet["FL"][1]),
    }


def draw_panel_a(ax, data):
    """Vista superior del robot y direccion phi_arc de cada pata."""
    ax.set_aspect("equal")
    ax.axis("off")
    ax.set_xlim(-0.215, 0.225)
    ax.set_ylim(-0.205, 0.185)

    r = data["r"]
    feet = data["feet"]
    hips = data["hips"]
    phis = data["phis"]

    # Circulo que pasa por las cuatro posiciones nominales.
    ax.add_patch(
        Circle(
            (0.0, 0.0),
            r,
            fill=False,
            ls=(0, (4, 3)),
            lw=1.2,
            color=COL_RAD,
            zorder=0,
        )
    )

    # Cuerpo.
    ax.add_patch(
        FancyBboxPatch(
            (-0.095, -0.072),
            0.19,
            0.144,
            boxstyle="round,pad=0.004,rounding_size=0.022",
            fc="#e8eef7",
            ec="#34495e",
            lw=1.6,
            zorder=3,
        )
    )
    ax.add_patch(
        Polygon(
            [[0.099, -0.022], [0.099, 0.022], [0.118, 0.0]],
            closed=True,
            fc="#34495e",
            ec="none",
            zorder=3,
        )
    )

    for key in LEG_NAMES:
        hip = hips[key][:2]
        foot = feet[key][:2]

        # Radio y pata.
        ax.plot(
            [0.0, foot[0]],
            [0.0, foot[1]],
            color=COL_RAD,
            lw=0.9,
            ls=":",
            zorder=1,
        )
        ax.plot(
            [hip[0], foot[0]],
            [hip[1], foot[1]],
            color="#bdc3c7",
            lw=4.2,
            solid_capstyle="round",
            zorder=2,
        )
        ax.add_patch(
            Circle(hip, 0.011, fc="#ecf0f1", ec="#7f8c8d", lw=1.1, zorder=4)
        )
        ax.add_patch(
            Circle(
                foot,
                0.012,
                fc="white",
                ec=LEG_COLORS[key],
                lw=2.0,
                zorder=5,
            )
        )

        # Direccion del paso rotacional calculada por YawCircle.
        phi = phis[key]
        arrow(
            ax,
            foot,
            foot + 0.058 * unit(phi),
            COL_PHI,
            lw=2.6,
            ms=13,
            zorder=6,
        )

        # Etiqueta de la pata.
        offset = {
            "FL": (0.013, 0.014),
            "FR": (0.013, -0.028),
            "RL": (-0.034, 0.014),
            "RR": (-0.034, -0.028),
        }[key]
        ax.text(
            foot[0] + offset[0],
            foot[1] + offset[1],
            key,
            color=LEG_COLORS[key],
            fontsize=11,
            fontweight="bold",
            zorder=7,
        )

    # Ejes del cuerpo.
    arrow(ax, (0.0, 0.0), (0.155, 0.0), COL_AXIS, lw=1.2, ms=9, zorder=4)
    arrow(ax, (0.0, 0.0), (0.0, 0.155), COL_AXIS, lw=1.2, ms=9, zorder=4)
    ax.text(0.160, 0.006, "x", fontsize=11, color=COL_AXIS)
    ax.text(-0.012, 0.162, "y", fontsize=11, color=COL_AXIS)

    # Flecha de velocidad angular.
    ax.annotate(
        "",
        xy=(-0.165, 0.095),
        xytext=(-0.165, -0.095),
        arrowprops=dict(
            arrowstyle="-|>",
            color="#e67e22",
            lw=2.4,
            connectionstyle="arc3,rad=-0.75",
            mutation_scale=16,
        ),
        zorder=5,
    )
    ax.text(-0.212, 0.006, r"$\omega$", fontsize=15, color="#e67e22")

    ax.set_title(
        "a) Vista superior",
        loc="left",
        fontsize=13,
        fontweight="bold",
    )

    ax.text(
        -0.205,
        -0.195,
        r"Flechas verdes: dirección $\phi_{arc}$   ·   Círculo gris: "
        r"trayectoria nominal",
        fontsize=8.6,
        color="#555555",
        va="bottom",
    )


def draw_panel_b(ax, data):
    """Construccion geometrica detallada para la pata FR (esquema ampliado)."""
    ax.set_aspect("equal")
    ax.axis("off")
    ax.set_xlim(-0.30, 1.98)
    ax.set_ylim(-1.50, 0.50)

    key = "FR"
    O = np.array([0.0, 0.0])
    theta = data["thetas"][key]
    theta_mod = data["thetas_mod"][key]
    phi = data["phis"][key]
    g_mag = data["g_mag"]

    # Esquema no a escala: se agrandan r y g para que los angulos se vean.
    r_plot = 1.35
    g_plot = 0.50
    P = r_plot * unit(theta)
    tangent = theta + np.pi / 2.0
    Q = P + g_plot * unit(tangent)
    phi_end = Q + 0.78 * unit(phi)

    # Ejes.
    arrow(ax, O, (1.58, 0.0), COL_AXIS, lw=1.2, ms=9, zorder=2)
    arrow(ax, O, (0.0, 0.42), COL_AXIS, lw=1.2, ms=9, zorder=2)
    ax.text(1.60, 0.015, "x", fontsize=11, color=COL_AXIS)
    ax.text(-0.018, 0.43, "y", fontsize=11, color=COL_AXIS)

    # Tangente en P (gris) y tangente en Q (verde, paralela a phi_arc).
    t_start = P - 0.38 * unit(tangent)
    t_end = P + 0.58 * unit(tangent)
    ax.plot(
        [t_start[0], t_end[0]],
        [t_start[1], t_end[1]],
        color=COL_TAN,
        lw=1.2,
        ls="--",
        zorder=1,
    )
    q_tan_start = Q - 0.22 * unit(phi)
    q_tan_end = Q + 0.58 * unit(phi)
    ax.plot(
        [q_tan_start[0], q_tan_end[0]],
        [q_tan_start[1], q_tan_end[1]],
        color=COL_PHI,
        lw=1.2,
        ls="--",
        alpha=0.7,
        zorder=1,
    )

    # Vectores.
    ax.plot([O[0], P[0]], [O[1], P[1]], color=COL_F, lw=3.4, zorder=3)
    ax.plot([P[0], Q[0]], [P[1], Q[1]], color=COL_G, lw=3.4, zorder=3)
    ax.plot(
        [O[0], Q[0]],
        [O[1], Q[1]],
        color="#e67e22",
        lw=1.5,
        ls="-.",
        zorder=2,
    )

    # Direccion final del paso (tangente al circulo en Q).
    arrow(ax, Q, phi_end, COL_PHI, lw=3.2, ms=17, zorder=6)

    # Marcas de 90 grados.
    def right_angle(point, u1, u2, size):
        pts = [
            point + size * u1,
            point + size * u1 + size * u2,
            point + size * u2,
        ]
        ax.add_patch(
            Polygon(pts, closed=False, fill=False, ec=COL_TAN, lw=1.4, zorder=4)
        )

    right_angle(
        P,
        (O - P) / np.linalg.norm(O - P),
        (Q - P) / np.linalg.norm(Q - P),
        size=0.10,
    )
    right_angle(
        Q,
        (O - Q) / np.linalg.norm(O - Q),
        unit(phi),
        size=0.075,
    )

    # Arco theta en O (de +x a OP).
    ax.add_patch(
        Arc(
            O,
            0.84,
            0.84,
            theta1=np.degrees(theta),
            theta2=0.0,
            color=COL_AXIS,
            lw=1.4,
            zorder=4,
        )
    )
    mid = theta / 2.0
    ax.text(
        0.29 * np.cos(mid),
        0.29 * np.sin(mid) - 0.015,
        r"$\theta_i$",
        fontsize=13,
        color=COL_AXIS,
        ha="center",
        va="center",
    )

    # Arco theta_mod en O (de OP a OQ): giro del pie alrededor del cuerpo.
    ax.add_patch(
        Arc(
            O,
            0.96,
            0.96,
            theta1=np.degrees(theta),
            theta2=np.degrees(theta + theta_mod),
            color=COL_G,
            lw=1.7,
            zorder=4,
        )
    )
    mid2 = theta + theta_mod / 2.0
    ax.annotate(
        r"$\theta_{mod}$",
        xy=(0.48 * np.cos(mid2), 0.48 * np.sin(mid2)),
        xytext=(-0.02, -0.72),
        fontsize=12,
        color=COL_G,
        ha="center",
        va="center",
        arrowprops=dict(
            arrowstyle="-",
            color=COL_G,
            lw=0.9,
            shrinkA=2,
            shrinkB=1,
        ),
        bbox=dict(fc="white", ec="none", pad=0.5, alpha=0.95),
        zorder=6,
    )

    # Arco theta_mod en Q (de la tangente en P a phi_arc).
    ax.add_patch(
        Arc(
            Q,
            0.52,
            0.52,
            theta1=np.degrees(tangent),
            theta2=np.degrees(phi),
            color=COL_G,
            lw=1.7,
            zorder=4,
        )
    )
    mid3 = tangent + theta_mod / 2.0
    label3 = (
        Q
        + 0.34 * unit(mid3)
        + 0.16 * unit(phi - np.pi / 2.0)
    )
    ax.text(
        label3[0],
        label3[1],
        r"$\theta_{mod}$",
        fontsize=12,
        color=COL_G,
        ha="center",
        va="center",
        bbox=dict(fc="white", ec="none", pad=0.5, alpha=0.95),
        zorder=6,
    )

    # Etiquetas de puntos y vectores.
    ax.plot(*O, "o", color=COL_AXIS, ms=5, zorder=7)
    ax.plot(*P, "o", color=COL_F, ms=7, zorder=7)
    ax.plot(*Q, "o", color=COL_G, ms=7, zorder=7)

    ax.text(-0.10, -0.07, r"$O$", fontsize=13, color=COL_AXIS, ha="right")
    ax.text(P[0] + 0.02, P[1] - 0.10, r"$P$", fontsize=13, color=COL_F)
    ax.text(Q[0] + 0.05, Q[1] - 0.13, r"$Q$", fontsize=13, color=COL_G)

    # Etiqueta de f a lo largo de OP (por debajo de la linea).
    f_mid = 0.52 * P
    f_off = -0.075 * unit(theta + np.pi / 2.0)
    ax.text(
        f_mid[0] + f_off[0],
        f_mid[1] + f_off[1],
        r"$f_{xyz}=T_{bf}$",
        fontsize=10.5,
        color=COL_F,
        rotation=np.degrees(theta),
        rotation_mode="anchor",
        ha="center",
        va="top",
        bbox=dict(fc="white", ec="none", pad=0.5, alpha=0.9),
    )

    # Etiqueta de g a lo largo de PQ (por debajo de la linea).
    g_mid = P + 0.40 * (Q - P)
    g_off = -0.155 * unit(tangent + np.pi / 2.0)
    ax.text(
        g_mid[0] + g_off[0],
        g_mid[1] + g_off[1],
        r"$g_{mag}$",
        fontsize=11.5,
        color=COL_G,
        rotation=np.degrees(tangent),
        rotation_mode="anchor",
        ha="center",
        va="top",
        bbox=dict(fc="white", ec="none", pad=0.5, alpha=0.9),
    )

    ax.text(
        t_start[0] - 0.02,
        t_start[1] - 0.03,
        "tangente en P",
        fontsize=9,
        color=COL_TAN,
        ha="right",
    )
    ax.text(
        phi_end[0] + 0.03,
        phi_end[1] - 0.02,
        r"$\phi_{arc}$",
        fontsize=14,
        color=COL_PHI,
    )

    ax.set_title(
        "b) Construcción de $\\phi_{arc}$ (pata FR)",
        loc="left",
        fontsize=13,
        fontweight="bold",
    )

    note = (
        rf"Pata FR: $r_i$ = {data['r'] * 100:.1f} cm, "
        rf"$|g_{{xy}}|$ = {g_mag * 100:.1f} cm, "
        rf"$\theta_{{mod}}$ = {np.degrees(theta_mod):.1f}°, "
        rf"$\phi_{{arc}}$ = {np.degrees(phi):.1f}°."
        "\n"
        r"Esquema no a escala: $g_{mag}$ lleva el pie de $P$ a $Q$; "
        r"$\phi_{arc}$ es la tangente al círculo en $Q$."
    )
    ax.text(
        -0.28,
        -1.47,
        note,
        fontsize=9.2,
        color="#555555",
        va="bottom",
        linespacing=1.45,
    )


def draw_panel_c(ax, data):
    """Ecuaciones implementadas, convencion de signos y tabla de ejemplo."""
    ax.axis("off")
    ax.set_xlim(0.0, 1.0)
    ax.set_ylim(0.0, 1.0)

    ax.text(
        0.0,
        0.97,
        "Ecuaciones implementadas y convención de signos",
        fontsize=14.5,
        fontweight="bold",
        va="top",
    )

    equations = [
        (r"$r_i=\sqrt{x_{bf}^2+y_{bf}^2}$", "radio nominal"),
        (r"$\theta_i=\mathrm{atan2}(y_{bf},x_{bf})$", "dirección del pie"),
        (
            r"$\mathbf{g}_{xyz}=\mathbf{f}_{prev}-\mathbf{T}_{bf}$",
            "vector de desviación",
        ),
        (
            r"$\theta_{mod}=\mathrm{atan2}(\|\mathbf{g}_{xy}\|,r_i)$",
            "corrección angular",
        ),
        (
            r"$\phi_{arc}=\frac{\pi}{2}\pm\theta_i+\theta_{mod}$",
            "dirección del componente rotacional",
        ),
    ]

    y = 0.80
    for eq, desc in equations:
        ax.text(0.01, y, eq, fontsize=14.5, va="center", color="#111111")
        ax.text(0.30, y, desc, fontsize=10.5, va="center", color="#666666")
        y -= 0.155

    # Convencion de signos.
    ax.text(
        0.60,
        0.86,
        "Signo del término $\\theta_i$:",
        fontsize=11.5,
        fontweight="bold",
        va="center",
    )
    ax.text(
        0.61,
        0.77,
        r"$+$  para FR y RL",
        fontsize=12.5,
        va="center",
        color=COL_G,
    )
    ax.text(
        0.61,
        0.69,
        r"$-$  para FL y RR",
        fontsize=12.5,
        va="center",
        color=COL_G,
    )

    # Tabla de ejemplo.
    ax.text(
        0.60,
        0.58,
        rf"Ejemplo con $|g_{{xy}}|={data['g_mag'] * 100:.0f}$ cm:",
        fontsize=10.5,
        va="center",
        color="#333333",
    )

    table_lines = [f"{'Pata':<6}{'θ_i':>9}{'φ_arc':>10}", "-" * 25]
    for key in LEG_NAMES:
        theta = np.degrees(data["thetas"][key])
        phi = np.degrees(data["phis"][key])
        table_lines.append(f"{key:<6}{theta:>+8.1f}°{phi:>9.1f}°")

    ax.text(
        0.60,
        0.50,
        "\n".join(table_lines),
        fontsize=10.0,
        va="top",
        fontfamily="monospace",
        color="#222222",
        linespacing=1.25,
    )

    # Nota de implementacion.
    note = (
        "Nota de implementación: en el código, Prev_fxyz guarda el último "
        "vector de paso (no la posición absoluta del pie).\n"
        "Por eso, en ejecución g no siempre es perpendicular a f; la figura "
        "geométrica muestra el caso ideal g ⊥ r. La magnitud del paso "
        "rotacional es YawRate·dt."
    )
    ax.text(
        0.01,
        0.02,
        note,
        fontsize=9.0,
        va="bottom",
        ha="left",
        color="#5b4a00",
        linespacing=1.45,
        bbox=dict(
            boxstyle="round,pad=0.5",
            fc="#fdf6e3",
            ec="#d4ac0d",
            lw=1.1,
            alpha=0.95,
        ),
    )


def _set_style():
    plt.rcParams.update(
        {
            "font.size": 11,
            "axes.titlesize": 13.5,
            "axes.titleweight": "bold",
            "figure.facecolor": "white",
        }
    )


def make_geometry_figure(data, save_path=DEFAULT_GEOMETRY, show=False, dpi=200):
    """Figura 1: vista superior y construccion geometrica."""
    _set_style()
    fig = plt.figure(figsize=(10.0, 5.2))
    gs = fig.add_gridspec(
        1,
        2,
        left=0.015,
        right=0.985,
        top=0.90,
        bottom=0.02,
        wspace=0.05,
    )
    ax_a = fig.add_subplot(gs[0, 0])
    ax_b = fig.add_subplot(gs[0, 1])
    draw_panel_a(ax_a, data)
    draw_panel_b(ax_b, data)

    if save_path:
        fig.savefig(save_path, dpi=dpi, bbox_inches="tight")
        print(f"Figura de geometria guardada en: {os.path.abspath(save_path)}")

    if show:
        plt.show()

    return fig


def make_equations_figure(data, save_path=DEFAULT_EQUATIONS, show=False, dpi=200):
    """Figura 2: ecuaciones, convencion de signos y tabla."""
    _set_style()
    fig = plt.figure(figsize=(10.0, 4.8))
    ax = fig.add_axes([0.015, 0.02, 0.97, 0.96])
    draw_panel_c(ax, data)

    if save_path:
        fig.savefig(save_path, dpi=dpi, bbox_inches="tight")
        print(f"Figura de ecuaciones guardada en: {os.path.abspath(save_path)}")

    if show:
        plt.show()

    return fig


def make_figures(
    g_mag,
    geometry_path=DEFAULT_GEOMETRY,
    equations_path=DEFAULT_EQUATIONS,
    show=False,
    dpi=200,
):
    """Genera las dos figuras y devuelve la lista de figuras y los datos."""
    data = compute_data(g_mag)
    fig_geo = make_geometry_figure(data, geometry_path, show=False, dpi=dpi)
    fig_eq = make_equations_figure(data, equations_path, show=False, dpi=dpi)

    if show:
        plt.show()

    return [fig_geo, fig_eq], data


def main():
    parser = argparse.ArgumentParser(
        description="Genera dos figuras explicativas de YawCircle."
    )
    parser.add_argument(
        "--save-geometry",
        type=str,
        default=DEFAULT_GEOMETRY,
        help=f"Ruta de la figura de geometria (def. {DEFAULT_GEOMETRY})",
    )
    parser.add_argument(
        "--save-equations",
        type=str,
        default=DEFAULT_EQUATIONS,
        help=f"Ruta de la figura de ecuaciones (def. {DEFAULT_EQUATIONS})",
    )
    parser.add_argument(
        "--g-mag",
        type=float,
        default=0.05,
        help="Magnitud de la desviación tangencial de ejemplo en metros (def. 0.05)",
    )
    parser.add_argument(
        "--dpi", type=int, default=200, help="Resolución de la imagen (def. 200)"
    )
    parser.add_argument(
        "--show", action="store_true", help="Abrir la ventana interactiva"
    )
    args = parser.parse_args()

    figs, data = make_figures(
        g_mag=args.g_mag,
        geometry_path=args.save_geometry,
        equations_path=args.save_equations,
        show=args.show,
        dpi=args.dpi,
    )
    for fig in figs:
        plt.close(fig)

    print("\nValores calculados por YawCircle:")
    print(f"  r_i = {data['r'] * 100:.2f} cm")
    print(f"  |g_xy| = {data['g_mag'] * 100:.1f} cm")
    print(
        f"  theta_mod = "
        f"{np.degrees(data['thetas_mod']['FL']):.2f}°"
    )
    for key in LEG_NAMES:
        print(
            f"  {key}: theta = {np.degrees(data['thetas'][key]):+7.2f}°, "
            f"phi_arc = {np.degrees(data['phis'][key]):7.2f}°"
        )


if __name__ == "__main__":
    main()
