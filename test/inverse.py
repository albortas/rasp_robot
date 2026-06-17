import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator
from matplotlib.widgets import Slider

from src.kinematics.Forward import Forward
from src.kinematics.Inverse import Inverse

# Iniciamos los objetos
ik = Inverse(legtype="LEFT")
fk = Forward(legtype="LEFT")

# == CONFIGURACION INICIAL ==
x0, y0, z0 = 0.0, 0.0615, -0.15

# Crear figura
fig = plt.figure(figsize=(12, 10))
ax3d = fig.add_subplot(221, projection="3d")
ax_pl_yz = fig.add_subplot(222)
ax_pl_xz = fig.add_subplot(224)
ax_err = fig.add_subplot(223)
plt.subplots_adjust(bottom=0.25)

# Sliders
ax_x = plt.axes([0.25, 0.15, 0.5, 0.03])
ax_y = plt.axes([0.25, 0.1, 0.5, 0.03])
ax_z = plt.axes([0.25, 0.05, 0.5, 0.03])

s_x = Slider(ax_x, "X", -0.24, 0.24, valinit=x0)
s_y = Slider(ax_y, "Y", -0.2, 0.2, valinit=y0)
s_z = Slider(ax_z, "Z", -0.238, -0.0222, valinit=z0)


def graphic_3d(puntos):
    # Limpiar ejes
    ax3d.clear()

    # Grafico
    (line3d,) = ax3d.plot(
        puntos[:, 0],
        puntos[:, 1],
        puntos[:, 2],
        "k-o",
        lw=2,
        markersize=8,
        markerfacecolor="blue",
    )

    # Ajustar limites
    max_range = 0.15
    ax3d.set_xlim(-max_range, max_range)
    ax3d.set_ylim(-max_range, max_range)
    ax3d.set_zlim(-max_range, max_range / 3)

    # Limitar numero de marcas
    ax3d.xaxis.set_major_locator(MaxNLocator(3))
    ax3d.yaxis.set_major_locator(MaxNLocator(3))
    ax3d.zaxis.set_major_locator(MaxNLocator(3))

    # Etiquetas
    ax3d.set_title("Cinematica Inversa")
    ax3d.set_xlabel("X")
    ax3d.set_ylabel("Y")
    ax3d.set_zlabel("Z")


def graphic_err(err):
    ax_err.clear()
    ax_err.bar(["X", "Y", "Z"], err, color=["r", "g", "b"])
    ax_err.set_title("Error FK - IK (m)")
    ax_err.axhline(0, color="k", linewidth=0.8)
    ax_err.set_ylim(-1, 1)


def plane_yz(puntos, position):
    ax_pl_yz.clear()
    # Grafico
    (line_yz,) = ax_pl_yz.plot(
        puntos[:, 1],
        puntos[:, 2],
        "k-o",
        lw=2,
        markersize=8,
        markerfacecolor="blue",
    )
    ax_pl_yz.scatter(position[1], position[2], color="red")

    # Ajustar limites
    max_range = 0.25
    ax_pl_yz.set_xlim(-max_range, max_range)
    ax_pl_yz.set_ylim(-max_range, max_range / 3)

    # Limitar numero de marcas
    ax_pl_yz.xaxis.set_major_locator(MaxNLocator(5))
    ax_pl_yz.yaxis.set_major_locator(MaxNLocator(5))

    # Etiquetas
    ax_pl_yz.set_title("Plano YZ")
    ax_pl_yz.set_xlabel("Y")
    ax_pl_yz.set_ylabel("Z")


def plane_xz(puntos, position):
    ax_pl_xz.clear()
    # Grafico
    (line_yz,) = ax_pl_xz.plot(
        puntos[:, 0],
        puntos[:, 2],
        "k-o",
        lw=2,
        markersize=8,
        markerfacecolor="blue",
    )
    ax_pl_xz.scatter(position[0], position[2], color="red")

    # Ajustar limites
    max_range = 0.25
    ax_pl_xz.set_xlim(max_range, -max_range)
    ax_pl_xz.set_ylim(-max_range, max_range / 3)

    # Limitar numero de marcas
    ax_pl_xz.xaxis.set_major_locator(MaxNLocator(5))
    ax_pl_xz.yaxis.set_major_locator(MaxNLocator(5))

    # Etiquetas
    ax_pl_xz.set_title("Plano XZ")
    ax_pl_xz.set_xlabel("X")
    ax_pl_xz.set_ylabel("Z")


def update(val):
    x = s_x.val
    y = s_y.val
    z = s_z.val
    position = np.array([x, y, z])

    try:
        theta = ik.solve(position)
        p1, p2, p3 = fk.solve(theta)
        err = p3 - position
        p0 = (0, 0, 0)
        puntos = np.array([p0, p1, p2, p3])
    except Exception as e:
        ax3d.set_title(f"Error: {e}")
        fig.canvas.draw_idle()
        return

    graphic_3d(puntos)
    ax3d.scatter(x, y, z, color="red", s=50, label="Objetivo CI")
    ax3d.legend()
    graphic_err(err)
    plane_yz(puntos, position)
    plane_xz(puntos, position)
    fig.canvas.draw_idle()


update(None)
s_x.on_changed(update)
s_y.on_changed(update)
s_z.on_changed(update)
plt.show()
