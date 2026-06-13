import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator
from matplotlib.widgets import Slider

from src.kinematics.Forward import Forward

# Iniciamos el objeto
fk = Forward('LEFT')

# Datos del hombro (sholder)
theta0, theta1, theta2 = 0, 0, 0
p0 = np.array([0, 0, 0])
theta = np.array([theta0, theta1, theta2])

# Datos del robot
p1, p2, p3 = fk.solve(theta)
puntos = np.array([p0, p1, p2, p3])

# Crear figura
fig = plt.figure(figsize=(12, 12))
ax1 = fig.add_subplot(221, projection='3d')
# Dejaar un espacio vacio en la parte inferior
# plt.subplots_adjust(bottom=0.3)

# Grafico inicial
line1, = ax1.plot(puntos[:,0], puntos[:,1], puntos[:,2], 'k-o', lw=2, markersize=8, markerfacecolor='red')

# Etiquetas
ax1.set_title('Pierna Robot')
ax1.set_xlabel('X')
ax1.set_ylabel('Y')
ax1.set_zlabel('Z')

# Ajustar limites
max_range = 0.25
ax1.set_xlim(-max_range, max_range)
ax1.set_ylim(-max_range, max_range)
ax1.set_zlim(-max_range, max_range/3)

# Limitar numero de marcas
ax1.xaxis.set_major_locator(MaxNLocator(5))
ax1.yaxis.set_major_locator(MaxNLocator(5))
ax1.zaxis.set_major_locator(MaxNLocator(5))

# Plano YZ
ax2 = fig.add_subplot(222)
line2, = ax2.plot(puntos[:,1], puntos[:,2], 'k-o', lw=2, markersize=8, markerfacecolor='red')

ax2.set_title('Plano YZ')
ax2.set_xlabel('Y')
ax2.set_ylabel('Z')

ax2.set_xlim(-max_range, max_range)
ax2.set_ylim(-max_range, max_range/3)

ax2.xaxis.set_major_locator(MaxNLocator(5))
ax2.yaxis.set_major_locator(MaxNLocator(5))

# Plano XZ
ax3 = fig.add_subplot(224)
line3, = ax3.plot(puntos[:,0], puntos[:,2], 'k-o', lw=2, markersize=8, markerfacecolor='red')

ax3.set_title('Plano XZ')
ax3.set_xlabel('X')
ax3.set_ylabel('Z')
ax3.set_xlim(-max_range, max_range)
ax3.set_ylim(-max_range, max_range/3)
ax3.xaxis.set_major_locator(MaxNLocator(5))
ax3.yaxis.set_major_locator(MaxNLocator(5))

# Crear los ejes para los Sliders
# El formato es una lista: [posicion_x, posicion_y, ancho, alto] (en fraciones de 0 a 1)
ax_theta0 = plt.axes((0.1, 0.3, 0.3, 0.03))
ax_theta1 = plt.axes((0.1, 0.25, 0.3, 0.03))
ax_theta2 = plt.axes((0.1, 0.2, 0.3, 0.03))

s_theta0 = Slider(
    ax=ax_theta0,
    label='Theta_0',
    valmin=-90,
    valmax=90,
    valinit=theta0,
    valstep=1
)

s_theta1 = Slider(ax_theta1, 'Theta_1', -90, 90, valinit= theta1, valstep=1)
s_theta2 = Slider(ax_theta2, 'Theta_2', 0, 180, valinit= theta2, valstep=1)

def update(val):
    # Obtenemos los valores actulaes de los sliders
    th0 =s_theta0.val
    th1 =s_theta1.val
    th2 =s_theta2.val
    
    # Actualizamos theta
    theta_grados = np.array([th0, th1, th2])
    
    # Convertimos todo a radianes
    theta = np.radians(theta_grados)
    
    # Resolvemos la cinematica directa
    p1, p2, p3 = fk.solve(theta)
    puntos = np.array([p0, p1, p2, p3])
    
    # Actualizamos la figura
    line1.set_data_3d(puntos[:, 0], puntos[:, 1], puntos[:, 2])
    line2.set_data(puntos[:,1], puntos[:,2])
    line3.set_data(puntos[:,0], puntos[:,2])
    
    # Redibujamos el canvas
    fig.canvas.draw_idle()
    
# Conectar los sliders a la funcion de actualizacion
s_theta0.on_changed(update)
s_theta1.on_changed(update)
s_theta2.on_changed(update)

plt.show()

