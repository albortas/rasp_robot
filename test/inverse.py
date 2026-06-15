import numpy as np
import matplotlib.pyplot as plt

from src.kinematics.Forward import Forward
from src.kinematics.Inverse import Inverse

# Iniciamos los objetos
ik = Inverse(legtype='LEFT')
fk = Forward(legtype='LEFT')

# Crear figura
fig = plt.figure(figsize=(10,10))
ax3d = fig.add_subplot(121, projection='3d')


# Ajustar limites
max_range = 0.25
ax3d.set_xlim(-max_range, max_range)
ax3d.set_ylim(-max_range, max_range)
ax3d.set_zlim(-max_range, max_range/3)

def update(val):
    position = [0, 0.0685, -0.24655]
    
    try:
        theta = ik.solve(position)
        p1, p2, p3 = fk.solve(theta)
        p0 = (0, 0, 0)
        puntos = np.array([p0, p1, p2, p3])
    except Exception as e:
        ax3d.set_title(f'Error: {e}')
        fig.canvas.draw_idle()
        return
    
    # Limpiar ejes
    ax3d.clear()
    
    # Grafico
    
update(None)
plt.show()