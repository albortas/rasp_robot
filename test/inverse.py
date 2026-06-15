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
x0, y0, z0 = 0.0, 0.0615, -0.2382

# Crear figura
fig = plt.figure(figsize=(10, 10))
ax3d = fig.add_subplot(121, projection="3d")

# Sliders
ax_x = plt.axes([0.1, 0.3, 0.3, 0.03])
ax_y = plt.axes([0.1, 0.25, 0.3, 0.03])
ax_z = plt.axes([0.1, 0.2, 0.3, 0.03])

s_x = Slider(ax_x, "X", -0.24, 0.24, valinit=x0)
s_y = Slider(ax_y, "Y", -0.2, 0.2, valinit=y0)
s_z = Slider(ax_z, "Z", -0.246, -0.09, valinit=z0)

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
        markerfacecolor="red",
    )
    
    # Ajustar limites
    max_range = 0.25
    ax3d.set_xlim(-max_range, max_range)
    ax3d.set_ylim(-max_range, max_range)
    ax3d.set_zlim(-max_range, max_range / 3)
    
    # Limitar numero de marcas
    ax3d.xaxis.set_major_locator(MaxNLocator(5))
    ax3d.yaxis.set_major_locator(MaxNLocator(5))
    ax3d.zaxis.set_major_locator(MaxNLocator(5))
    
    # Etiquetas
    ax3d.set_title("Cinematica Inversa")
    ax3d.set_xlabel("X")
    ax3d.set_ylabel("Y")
    ax3d.set_zlabel("Z")


def update(val):
    x = s_x.val
    y = s_y.val
    z = s_z.val
    position = [x, y, z]

    try:
        theta = ik.solve(position)
        p1, p2, p3 = fk.solve(theta)
        p0 = (0, 0, 0)
        puntos = np.array([p0, p1, p2, p3])
    except Exception as e:
        ax3d.set_title(f"Error: {e}")
        fig.canvas.draw_idle()
        return
    
    graphic_3d(puntos)
    fig.canvas.draw_idle()

update(None)
s_x.on_changed(update)
s_y.on_changed(update)
s_z.on_changed(update)
plt.show()
