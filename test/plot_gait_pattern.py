import matplotlib.pyplot as plt
import numpy as np
from src.gait.bezier_gait import BazierGait  # Ajusta según tu archivo

def simulate_gait(gait, Tstance, Tswing, total_time, dt=0.001):
    """
    Simula el generador de marcha durante 'total_time' segundos.
    Devuelve arrays de tiempo y las fases de las 4 patas.
    """
    num_steps = int(total_time / dt)
    time_axis = np.linspace(0, total_time, num_steps)
    phases = np.zeros((num_steps, 4))  # 4 patas
    states = np.zeros((num_steps, 4), dtype=int)  # 0=STANCE, 1=SWING

    gait.time = 0.0
    gait.time_since_last_TD = 0.0
    gait.TD_time = 0.0
    gait.TD = False
    gait.SwRef = 0.0

    Tstride = Tstance + Tswing

    for i, t in enumerate(time_axis):
        # Guardamos fase de cada pata
        for leg in range(4):
            phase, state = gait.GetPhase(leg, Tstance, Tswing)
            phases[i, leg] = phase
            states[i, leg] = state
        # Avanzamos el tiempo
        gait.Increment(dt, Tstride)

    return time_axis, phases, states


def plot_sawtooth_signals(time_axis, phases, states, Tstance, Tswing):
    """
    Dibuja las señales de fase en diente de sierra (similar a Fig.4 y 5 del paper).
    También añade un diagrama de apoyo (barras horizontales) al pie.
    """
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8), sharex=True,
                                   gridspec_kw={'height_ratios': [2, 1]})

    # Colores para cada pata (FL, FR, BL, BR)
    colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728']
    labels = ['FL (ref)', 'FR', 'RL', 'RR']

    # --- Gráfico superior: señales de fase ---
    for leg in range(4):
        ax1.plot(time_axis, phases[:, leg], color=colors[leg], label=labels[leg], linewidth=2)

    # Líneas horizontales para marcar los límites de fase
    ax1.axhline(y=0.0, color='gray', linestyle='--', alpha=0.5)
    ax1.axhline(y=1.0, color='gray', linestyle='--', alpha=0.5)

    # Resaltar regiones de STANCE (fase = 0 en stance? mejor colorear fondo)
    # Alternativamente, podemos marcar los intervalos de stance con bandas
    # Obtener los tiempos donde cada pata está en STANCE (state == 0)
    # Lo haremos de forma sencilla: pintamos fondo transparente

    ax1.set_ylabel('Fase (S)')
    ax1.set_title('Señales de fase generadas por el Gait Pattern Modulator\n'
                  f'Tst = {Tstance}s, Tsw = {Tswing}s')
    ax1.legend(loc='upper right')
    ax1.grid(True, alpha=0.3)
    ax1.set_ylim(-0.1, 1.1)

    # --- Gráfico inferior: diagrama de apoyos (footfall pattern) ---
    # Mostramos barras horizontales: STANCE en azul, SWING en blanco/rojo
    for leg in range(4):
        # Convertimos el estado a una secuencia de intervalos
        # Buscamos cambios de estado
        state_changes = np.diff(states[:, leg], prepend=states[0, leg])
        change_indices = np.where(state_changes != 0)[0]

        # Dibujamos barras para cada intervalo
        for j in range(len(change_indices) - 1):
            start_t = time_axis[change_indices[j]]
            end_t = time_axis[change_indices[j+1]]
            state_val = states[change_indices[j], leg]
            color = 'blue' if state_val == 0 else 'red'  # STANCE = azul, SWING = rojo
            ax2.barh(y=leg, width=end_t - start_t, left=start_t,
                     height=0.8, color=color, alpha=0.6)

    ax2.set_yticks(range(4))
    ax2.set_yticklabels(labels)
    ax2.set_xlabel('Tiempo (s)')
    ax2.set_title('Diagrama de apoyos (STANCE = azul, SWING = rojo)')
    ax2.set_xlim(time_axis[0], time_axis[-1])
    ax2.grid(True, alpha=0.2)

    plt.tight_layout()
    plt.show()


def plot_phase_vs_speed(gait_class, speeds, Tswing=0.25, total_time=2.0):
    """
    Muestra cómo varían las señales de fase al cambiar la velocidad (Tstance).
    """
    fig, axes = plt.subplots(len(speeds), 1, figsize=(10, 8), sharex=True)
    if len(speeds) == 1:
        axes = [axes]

    for idx, vd in enumerate(speeds):
        # Calcular Tstance según la ecuación del paper (11): Tst = 2*Lspan / vd
        # Suponemos Lspan = 0.17 m (valor usado en el paper)
        Lspan = 0.17
        Tstance = 2 * Lspan / vd
        
        gait = gait_class(dSref=[0.0, 0.5, 0.5, 0.0], Tswing=Tswing)
        gait.ref_idx = 0

        time_axis, phases, states = simulate_gait(gait, Tstance, Tswing, total_time)

        for leg in range(4):
            axes[idx].plot(time_axis, phases[:, leg], label=f'Leg {leg}', alpha=0.8)
        axes[idx].set_ylabel(f'v = {vd:.1f} m/s\n(Tst={Tstance:.3f}s)')
        axes[idx].set_ylim(-0.1, 1.1)
        axes[idx].grid(True, alpha=0.3)

    axes[-1].set_xlabel('Tiempo (s)')
    axes[0].legend(['FL', 'FR', 'RL', 'RR'])
    fig.suptitle('Señales de fase para diferentes velocidades (trote)', fontsize=14)
    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    # --- Ejemplo 1: Trote estándar (como en el paper) ---
    gait = BazierGait(dSref=[0.0, 0.5, 0.5, 0.0], Tswing=0.25)
    gait.ref_idx = 0

    Tstance = 0.5   # Tstance típico para ~2.5 m/s (con Lspan=0.17)
    Tswing = 0.25
    total_time = 2.0
    dt = 0.001

    time_axis, phases, states = simulate_gait(gait, Tstance, Tswing, total_time, dt)
    plot_sawtooth_signals(time_axis, phases, states, Tstance, Tswing)

    # --- Ejemplo 2: Comparación de señales a diferentes velocidades ---
    speeds = [2.0, 3.0, 4.5, 6.0]  # m/s (Tstance = 2*0.17/v)
    plot_phase_vs_speed(BazierGait, speeds, Tswing=0.25, total_time=2.0)

    # --- Ejemplo 3: Transición de trote a galope (opcional) ---
    # Aquí puedes modificar dinámicamente dSref y graficar, como en la Fig.25