import numpy as np
import matplotlib.pyplot as plt

from src.kinematics.Forward import Forward

# Iniciamos el objeto
fk = Forward()

# Datos del hombro (sholder)
p0 = np.array([0, 0, 0])
theta = np.array([0, 0, 0])

# Datos del robot
p1, p2, p3 = fk.solve(theta)
puntos = np.array([p0, p1, p2, p3])
print(puntos[:,0])

# Crear figura
fig = plt.figure(figsize=(10,6))
ax = fig.add_subplot(121, projection='3d')
line = ax.plot(puntos[:,0], puntos[:,1], puntos[:,2], 'b-o', lw=2, markersize=8, markerfacecolor='black')

# Etiquetas
ax.set_title('Pierna Robot')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

# Ajustar limites
max_range = 0.25
ax.set_xlim(-max_range, max_range)
ax.set_ylim(-max_range, max_range)
ax.set_zlim(-max_range, max_range)

plt.show()

