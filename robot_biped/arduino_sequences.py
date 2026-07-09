"""
Secuencias de Movimiento del Arduino
=====================================
Traducción de las secuencias del código Arduino a Python para
reproducirlas en la simulación.

Mapeo de servos:
    z1=0  Tobillo D7  (índice 0)
    z2=1  Rodilla D6  (índice 1)
    z3=2  Cadera  D5  (índice 2)
    z4=3  Tobillo D4  (índice 3)
    z5=4  Rodilla D3  (índice 4)
    z6=5  Cadera  D2  (índice 5)
"""

from dataclasses import dataclass
from typing import List, Tuple
import numpy as np


@dataclass
class Movement:
    """Un movimiento de un servo."""
    servo: int      # Índice del servo (0-5)
    angle: float    # Ángulo en grados
    delay_ms: int   # Espera en ms


# ============================================================
# SECUENCIAS DEL CÓDIGO ARDUINO
# ============================================================

HOME_POSITION: List[Movement] = [
    Movement(0, 90, 500),
    Movement(1, 90, 500),
    Movement(2, 90, 500),
    Movement(3, 80, 500),
    Movement(4, 90, 500),
    Movement(5, 90, 500),
]

WALK_STEP1: List[Movement] = [
    Movement(3, 60, 500),
    Movement(4, 100, 500),
    Movement(5, 70, 500),
    Movement(3, 80, 500),
    Movement(0, 60, 500),
    Movement(1, 100, 100),
    Movement(3, 90, 100),
    Movement(2, 100, 500),
    Movement(1, 90, 500),
    Movement(2, 110, 500),
    Movement(1, 80, 500),
    Movement(2, 120, 500),
    Movement(0, 70, 500),
    Movement(3, 80, 500),
    Movement(5, 70, 500),
    Movement(0, 80, 500),
    Movement(3, 70, 500),
    Movement(3, 60, 500),
    Movement(0, 90, 500),
    Movement(2, 100, 500),
    Movement(1, 90, 500),
]


class SequencePlayer:
    """
    Reproductor de secuencias de movimiento del Arduino.
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

    def execute_step(self, sequence: List[Movement]) -> np.ndarray:
        """
        Ejecuta un paso de la secuencia y retorna los ángulos actualizados.

        Parámetros:
            sequence: Lista de movimientos

        Retorna:
            Array de ángulos [z1..z6] actualizados
        """
        if self.step_index >= len(sequence):
            self.step_index = 0  # Loop

        mov = sequence[self.step_index]
        self.current_angles[mov.servo] = mov.angle
        self.step_index += 1

        return np.array(self.current_angles)

    def execute_full_sequence(self, sequence: List[Movement]) -> List[np.ndarray]:
        """
        Ejecuta una secuencia completa y retorna todos los estados.

        Parámetros:
            sequence: Lista de movimientos

        Retorna:
            Lista de arrays de ángulos para cada paso
        """
        states = []
        self.reset_to_home()

        for mov in sequence:
            self.current_angles[mov.servo] = mov.angle
            states.append(np.array(self.current_angles))

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


# Secuencias disponibles
AVAILABLE_SEQUENCES = {
    'home': HOME_POSITION,
    'walk_step1': WALK_STEP1,
}
