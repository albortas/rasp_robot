# Resumen de Ecuaciones Kinemáticas

## Visión general

El solucionador de cinemática inversa en `src/kinematics/Inverse.py` calcula los ángulos de articulación para cada pata en función de coordenadas cartesianas (x, y, z) de la posición del pie relativas a la articulación de la cadera.

## Ecuaciones clave

### Cálculo del dominio

La validez del dominio se verifica utilizando:

$$
D = \frac{y² + z² - L1² + x² - L2² - L3²}{2 * L2 * L3}
$$

Donde:

- L1, L2, L3 corresponden a las longitudes de los enlaces
- D debe estar en [-1, 1] para soluciones válidas

### Cálculo de ángulos de articulación

Para un valor dado de D:

$$
\theta_1 = -\arctan\left(\frac{z}{y}\right) - \arctan\left(\frac{AG}{side * L1}\right)
$$

$$
\theta_3 = \arccos(D)
$$

$$
\theta_2 = \arctan\left(\frac{x}{AG}\right) - \arctan\left(\frac{L3 * sen(theta3)}{L2 + L3 * cos(theta3)}\right)
$$

Donde:

- `AG = sqrt(y² + z² - L1²)`
- `side` es 1 para patas IZQUIERDAS y -1 para patas DERECHAS

- Los ángulos finales de articulación se devuelven como: `[-theta1, -theta2, -theta3]`

## Notas

- Las ecuaciones se derivan del modelo cinemático de pierna de 3-DOF.
- Las patas 'Wildcat' (izquierda/derecha) tienen geometría reflejada manejada por el parámetro `side`.
- Casos extremos (ejemplo: raíces cuadradas negativas) se manejan con acotamiento y advertencias.