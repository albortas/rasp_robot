import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator
from matplotlib.widgets import Slider

from src.kinematics.forward import Forward
from src.kinematics.inverse import Inverse

# Iniciamos los objetos
ik = Inverse()
fk = Forward()

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


# == INICIALIZACION DE GRAFICOS ==
max_range = 0.15
ax3d.set_xlim(-max_range, max_range)
ax3d.set_ylim(-max_range, max_range)
ax3d.set_zlim(-max_range, max_range / 3)
ax3d.xaxis.set_major_locator(MaxNLocator(3))
ax3d.yaxis.set_major_locator(MaxNLocator(3))
ax3d.zaxis.set_major_locator(MaxNLocator(3))
ax3d.set_xlabel("X")
ax3d.set_ylabel("Y")
ax3d.set_zlabel("Z")
ax3d.set_title("Cinematica Inversa (Rango Válido)", color="green")
(line3d,) = ax3d.plot([], [], [], "o-", color="black", lw=2, markersize=8, markerfacecolor="blue")
scatter3d = ax3d.scatter([], [], [], color="red", s=50, label="Objetivo CI")
ax3d.legend()

ax_err.set_title("Error FK - IK (m)")
ax_err.axhline(0, color="k", linewidth=0.8)
ax_err.set_ylim(-1, 1)
bar_err = ax_err.bar(["X", "Y", "Z"], [0, 0, 0], color=["r", "g", "b"])

max_range_yz = 0.25
ax_pl_yz.set_xlim(-max_range_yz, max_range_yz)
ax_pl_yz.set_ylim(-max_range_yz, max_range_yz / 3)
ax_pl_yz.xaxis.set_major_locator(MaxNLocator(5))
ax_pl_yz.yaxis.set_major_locator(MaxNLocator(5))
ax_pl_yz.set_title("Plano YZ")
ax_pl_yz.set_xlabel("Y")
ax_pl_yz.set_ylabel("Z")
(line_yz,) = ax_pl_yz.plot([], [], "o-", color="black", lw=2, markersize=8, markerfacecolor="blue")
scatter_yz = ax_pl_yz.scatter([], [], color="red")

ax_pl_xz.set_xlim(max_range_yz, -max_range_yz)
ax_pl_xz.set_ylim(-max_range_yz, max_range_yz / 3)
ax_pl_xz.xaxis.set_major_locator(MaxNLocator(5))
ax_pl_xz.yaxis.set_major_locator(MaxNLocator(5))
ax_pl_xz.set_title("Plano XZ")
ax_pl_xz.set_xlabel("X")
ax_pl_xz.set_ylabel("Z")
(line_xz,) = ax_pl_xz.plot([], [], "o-", color="black", lw=2, markersize=8, markerfacecolor="blue")
scatter_xz = ax_pl_xz.scatter([], [], color="red")


def update(val):
    x = s_x.val
    y = s_y.val
    z = s_z.val
    position = np.array([x, y, z])

    # Validar límites de los parámetros para evitar salidas del espacio de trabajo
    lower_limit_base = ik.L1**2 + (ik.L2 - ik.L3) ** 2
    upper_limit_base = ik.L1**2 + (ik.L2 + ik.L3) ** 2
    current_dist_sq = x**2 + y**2 + z**2

    is_out_of_bounds = current_dist_sq > upper_limit_base or current_dist_sq < lower_limit_base
    color_line = "red" if is_out_of_bounds else "black"

    try:
        theta = ik.solve("LEFT", position)
        p1, p2, p3 = fk.solve("LEFT", theta)
        err = p3 - position
        p0 = (0, 0, 0)
        puntos = np.array([p0, p1, p2, p3])
    except Exception as e:
        ax3d.set_title(f"Error: {e}", color="red")
        fig.canvas.draw_idle()
        return

    # Actualizar grafico 3D
    line3d.set_data(puntos[:, 0], puntos[:, 1])
    line3d.set_3d_properties(puntos[:, 2])
    line3d.set_color(color_line)
    scatter3d._offsets3d = ([x], [y], [z])
    if is_out_of_bounds:
        ax3d.set_title("⚠️ Fuera del Espacio de Trabajo", color="red")
    else:
        ax3d.set_title("Cinematica Inversa (Rango Válido)", color="green")

    # Actualizar Error
    for rect, h in zip(bar_err, err):
        rect.set_height(h)

    # Actualizar Plano YZ
    line_yz.set_data(puntos[:, 1], puntos[:, 2])
    line_yz.set_color(color_line)
    scatter_yz.set_offsets([[position[1], position[2]]])

    # Actualizar Plano XZ
    line_xz.set_data(puntos[:, 0], puntos[:, 2])
    line_xz.set_color(color_line)
    scatter_xz.set_offsets([[position[0], position[2]]])

    fig.canvas.draw_idle()


update(None)
s_x.on_changed(update)
s_y.on_changed(update)
s_z.on_changed(update)
plt.show()
