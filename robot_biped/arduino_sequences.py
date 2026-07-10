"""
Secuencias de Movimiento del Arduino
=====================================
Secuencias de caminata generadas por cinemática inversa (IK)
para el robot bípedo 12GDL. Cada frame contiene los 6 ángulos
de servo sincronizados, produciendo movimiento coordinado.

Mapeo de servos:
    z1=0  Tobillo D7  (índice 0)
    z2=1  Rodilla D6  (índice 1)
    z3=2  Cadera  D5  (índice 2)
    z4=3  Tobillo D4  (índice 3)
    z5=4  Rodilla D3  (índice 4)
    z6=5  Cadera  D2  (índice 5)
"""

from dataclasses import dataclass
from typing import List, Tuple, Optional
import numpy as np


@dataclass
class WalkState:
    """Estado completo de los 6 servos en un instante.

    Usado para secuencias IK donde todos los servos se mueven
    simultáneamente de forma coordinada.
    """
    angles: List[float]    # [z1, z2, z3, z4, z5, z6]
    delay_ms: int = 80     # Duración del frame en ms



# ============================================================
# GENERACIÓN DE SECUENCIAS IK
# ============================================================

def generate_ik_walk_states(
    step_length: float = 3.0,
    step_height: float = 1.5,
    n_points: int = 40,
    frame_delay_ms: int = 80
) -> List[WalkState]:
    """
    Genera una secuencia de caminata completa usando cinemática inversa.

    Retorna una lista de WalkState donde cada estado tiene los 6 ángulos
    de servo sincronizados, produciendo un movimiento coordinado y suave
    idéntico al de ``simulate-ik``.

    Parámetros:
        step_length: Longitud del paso en cm
        step_height: Altura del paso en cm
        n_points: Número de frames por ciclo
        frame_delay_ms: Duración de cada frame en ms

    Retorna:
        Lista de WalkState con ángulos [z1, z2, z3, z4, z5, z6]
    """
    from robot_biped.inverse_kinematics import InverseKinematics

    ik = InverseKinematics()
    walk = ik.generate_walk_cycle(step_length, step_height, n_points)

    states = []
    # HOME en formato IK: (cadera, rodilla, tobillo)
    last_la = (90.0, 90.0, 90.0)
    last_ra = (90.0, 90.0, 80.0)

    for (lx, ly, lz), (rx, ry, rz) in zip(walk['left'], walk['right']):
        la = ik.solve(lx, ly, lz, "left", reference=last_la)
        ra = ik.solve(rx, ry, rz, "right", reference=last_ra)

        if la is None:
            la = last_la
        else:
            last_la = la

        if ra is None:
            ra = last_ra
        else:
            last_ra = ra

        # IK retorna (cadera, rodilla, tobillo)
        # Arduino espera [z1, z2, z3, z4, z5, z6]
        #              = [tob_L, rod_L, cad_L, tob_R, rod_R, cad_R]
        angles = [
            la[2], la[1], la[0],   # Pierna izquierda: tobillo, rodilla, cadera
            ra[2], ra[1], ra[0],   # Pierna derecha: tobillo, rodilla, cadera
        ]

        states.append(WalkState(angles=angles, delay_ms=frame_delay_ms))

    return states


def interpolate_states(
    states: List[WalkState],
    interp_factor: int = 3
) -> List[WalkState]:
    """
    Interpola entre estados para suavizar transiciones.

    Genera 'interp_factor' frames intermedios entre cada par de estados
    originales usando interpolación lineal.

    Parámetros:
        states: Lista de WalkState originales
        interp_factor: Número de frames intermedios entre cada par

    Retorna:
        Lista de WalkState con frames interpolados
    """
    if len(states) < 2 or interp_factor < 1:
        return states

    result = []

    for i in range(len(states)):
        j = (i + 1) % len(states)
        a = np.array(states[i].angles)
        b = np.array(states[j].angles)

        # Dividir la duración del frame entre los sub-frames
        sub_delay = max(1, states[i].delay_ms // (interp_factor + 1))

        for k in range(interp_factor + 1):
            t = k / (interp_factor + 1)
            interp = a + (b - a) * t
            result.append(WalkState(
                angles=interp.tolist(),
                delay_ms=sub_delay
            ))

    return result


class SequencePlayer:
    """
    Reproductor de secuencias de movimiento del Arduino.
    Soporta tanto secuencias de un solo servo (Movement) como
    secuencias coordinadas (WalkState).
    """

    # Mapeo de índices de servo a nombres
    SERVO_NAMES = {
        0: "Tobillo Izq (z1)",
        1: "Rodilla Izq (z2)",
        2: "Cadera Izq (z3)",
        3: "Tobillo Der (z4)",
        4: "Rodilla Der (z5)",
        5: "Cadera Der (z6)",
    }

    def __init__(self):
        self.current_angles = [90.0, 90.0, 90.0, 80.0, 90.0, 90.0]
        self.step_index = 0

    def reset_to_home(self) -> np.ndarray:
        """Resetea a posición home."""
        self.current_angles = [90.0, 90.0, 90.0, 80.0, 90.0, 90.0]
        self.step_index = 0
        return np.array(self.current_angles)

    def get_current_angles(self) -> np.ndarray:
        """Retorna los ángulos actuales como array [z1..z6]."""
        return np.array(self.current_angles)

    def get_leg_angles(self) -> Tuple[List[float], List[float]]:
        """
        Separa los ángulos por pierna.

        Retorna:
            (left_angles, right_angles) donde:
            left_angles = [z1, z2, z3] (tobillo, rodilla, cadera izq)
            right_angles = [z4, z5, z6] (tobillo, rodilla, cadera der)
        """
        return (
            [self.current_angles[0], self.current_angles[1], self.current_angles[2]],
            [self.current_angles[3], self.current_angles[4], self.current_angles[5]],
        )

    def execute_step(self, sequence: List[WalkState]) -> np.ndarray:
        """
        Ejecuta un paso de la secuencia y retorna los ángulos actualizados.
        """
        if self.step_index >= len(sequence):
            self.step_index = 0  # Loop

        state = sequence[self.step_index]
        for i in range(min(6, len(state.angles))):
            self.current_angles[i] = state.angles[i]

        self.step_index += 1
        return np.array(self.current_angles)

    def execute_full_sequence(self, sequence: List[WalkState]) -> List[np.ndarray]:
        """
        Ejecuta una secuencia completa y retorna todos los estados.
        """
        states = []
        self.reset_to_home()

        for state in sequence:
            for i in range(min(6, len(state.angles))):
                self.current_angles[i] = state.angles[i]
            states.append(np.array(self.current_angles.copy()))

        return states

    @staticmethod
    def sequences_to_animation_data(
        states: List[np.ndarray]
    ) -> dict:
        """
        Convierte estados de secuencia a datos de animación.

        Retorna:
            dict con arrays para cada servo a lo largo del tiempo
        """
        data = {
            'z1': [], 'z2': [], 'z3': [],
            'z4': [], 'z5': [], 'z6': [],
        }

        for state in states:
            data['z1'].append(state[0])
            data['z2'].append(state[1])
            data['z3'].append(state[2])
            data['z4'].append(state[3])
            data['z5'].append(state[4])
            data['z6'].append(state[5])

        return {k: np.array(v) for k, v in data.items()}


# ============================================================
# SECUENCIAS PRECALCULADAS
# ============================================================

# Generar secuencia IK al cargar el módulo
_IK_WALK_STATES = generate_ik_walk_states(
    step_length=3.0, step_height=1.5, n_points=40, frame_delay_ms=80
)

# Versión interpolada para animación ultra-suave
IK_WALK_SMOOTH = interpolate_states(_IK_WALK_STATES, interp_factor=2)

# Secuencias disponibles
AVAILABLE_SEQUENCES = {
    'ik_walk': IK_WALK_SMOOTH,
}

