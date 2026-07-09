"""
Cinemática Directa Denavit-Hartenberg
=====================================
Implementación de las matrices de transformación homogénea para ambas
extremidades inferiores del robot bípedo, basado en los parámetros DH
del Trabajo de Grado.

Parámetros geométricos:
    L1 = 4.5 cm (Cadera → Rodilla)
    L2 = 4.0 cm (Rodilla → Tobillo)
    L3 = 3.0 cm (Tobillo → Apoyo del pie)
    Separación cadera-cadera = 10 cm (±5 cm del centro del torso)

Convención de coordenadas (Tesis):
    X = adelante (+) / atrás (-)
    Y = lateral izquierdo (+) / derecho (-)
    Z = profundidad hacia abajo (+) desde la cadera
"""

import numpy as np
from dataclasses import dataclass
from typing import Tuple, List


@dataclass
class DHParams:
    """Parámetros Denavit-Hartenberg para un eslabón."""
    a: float      # Longitud del eslabón (cm)
    alpha: float  # Ángulo de torsión (grados)
    d: float      # Desplazamiento (cm)
    theta_offset: float  # Offset articular (grados)


@dataclass
class RobotConfig:
    """Configuración geométrica completa del robot."""
    L1: float = 4.5   # Cadera → Rodilla
    L2: float = 4.0   # Rodilla → Tobillo
    L3: float = 3.0   # Tobillo → Pie
    hip_separation: float = 10.0  # Separación entre caderas


# ============================================================
# TABLAS DH - Según la Tesis (Tablas 5.2 y 5.3)
# ============================================================

# Pierna Izquierda: z3 (Cadera), z2 (Rodilla), z1 (Tobillo)
# theta_i = ang_z_i - 90° (offset de home)
DH_LEFT_LEG: List[DHParams] = [
    DHParams(a=4.5, alpha=0,   d=0, theta_offset=-90.0),  # z3 - Cadera
    DHParams(a=4.0, alpha=90,  d=0, theta_offset=-90.0),  # z2 - Rodilla
    DHParams(a=3.0, alpha=0,   d=0, theta_offset=-90.0),  # z1 - Tobillo
]

# Pierna Derecha: z6 (Cadera), z5 (Rodilla), z4 (Tobillo)
# theta_i = ang_z_i - 90° (offset de home, excepto z4 que es 80°)
DH_RIGHT_LEG: List[DHParams] = [
    DHParams(a=4.5, alpha=0,   d=0, theta_offset=-90.0),  # z6 - Cadera
    DHParams(a=4.0, alpha=90,  d=0, theta_offset=-90.0),  # z5 - Rodilla
    DHParams(a=3.0, alpha=0,   d=0, theta_offset=-80.0),  # z4 - Tobillo (offset especial)
]

# Posición home (grados) según la tesis Tabla 5.1
HOME_LEFT = [90.0, 90.0, 90.0]    # z1, z2, z3
HOME_RIGHT = [80.0, 90.0, 90.0]   # z4, z5, z6


def servo_to_dh(servo_angles: List[float], dh_params: List[DHParams]) -> List[float]:
    """
    Convierte ángulos de servo a ángulos theta de DH.
    theta_DH = ang_servo + theta_offset
    """
    return [ang + dh.theta_offset for ang, dh in zip(servo_angles, dh_params)]


def dh_to_servo(dh_angles: List[float], dh_params: List[DHParams]) -> List[float]:
    """
    Convierte ángulos theta de DH a ángulos de servo.
    ang_servo = theta_DH - theta_offset
    """
    return [th - dh.theta_offset for th, dh in zip(dh_angles, dh_params)]


def dh_matrix(a: float, alpha: float, d: float, theta: float) -> np.ndarray:
    """
    Construye la matriz de transformación homogénea A_i según DH.

    A_i = Rot(z, θ) · Trans(z, d) · Trans(x, a) · Rot(x, α)

    Parámetros:
        a:      longitud del eslabón (cm)
        alpha:  ángulo de torsión (rad)
        d:      desplazamiento (cm)
        theta:  ángulo articular (rad)

    Retorna:
        Matriz 4×4 de transformación homogénea
    """
    ct = np.cos(theta)
    st = np.sin(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)

    A = np.array([
        [ct,    -st * ca,  st * sa,   a * ct],
        [st,     ct * ca, -ct * sa,   a * st],
        [0.0,    sa,       ca,         d     ],
        [0.0,    0.0,      0.0,        1.0   ]
    ], dtype=float)

    return A


class LegKinematics:
    """
    Cinemática directa de una pierna del robot bípedo.
    Usa ecuaciones analíticas de la tesis (sección 5.0.4) para
    posiciones en el plano sagital, con Y para el desplazamiento lateral.
    """

    def __init__(self, dh_params: List[DHParams], side: str = "left"):
        """
        Inicializa la cinemática de una pierna.

        Parámetros:
            dh_params: Lista de 3 parámetros DH (cadera, rodilla, tobillo)
            side: "left" o "right"
        """
        self.dh = dh_params
        self.side = side
        self.config = RobotConfig()

    def forward_kinematics(self, angles_deg: List[float]) -> Tuple[np.ndarray, List[np.ndarray]]:
        """
        Cinemática directa completa con matrices DH.

        Parámetros:
            angles_deg: [ángulo_tobillo, ángulo_rodilla, ángulo_cadera] en grados
                       NOTA: En orden [z1, z2, z3] (tobillo, rodilla, cadera)

        Retorna:
            T_total: Matriz de transformación total 4×4 (cadera → pie)
            transforms: Lista de matrices intermedias [A1, A2, A3]
        """
        # Reordenar a [cadera, rodilla, tobillo] para aplicar DH en orden
        ordered = [angles_deg[2], angles_deg[1], angles_deg[0]]
        transforms = []
        T = np.eye(4)

        for i, (param, ang) in enumerate(zip(self.dh, ordered)):
            theta_rad = np.radians(ang + param.theta_offset)
            alpha_rad = np.radians(param.alpha)

            A = dh_matrix(param.a, alpha_rad, param.d, theta_rad)
            transforms.append(A)
            T = T @ A

        return T, transforms

    def forward_kinematics_analytic(self, angles_deg: List[float]) -> dict:
        """
        Cinemática directa usando las ecuaciones analíticas de la tesis.
        Más eficiente y consistente con el modelo matemático.

        Ecuaciones (sección 5.0.4):
            X = L1·sin(θ1) + (L2 + L3·cos(θ3))·sin(θ1 + θ2)
            Y = L3·sin(θ3)
            Z = L1·cos(θ1) + (L2 + L3·cos(θ3))·cos(θ1 + θ2)

        Parámetros:
            angles_deg: [ángulo_tobillo, ángulo_rodilla, ángulo_cadera] en grados
                        NOTA: En orden [z1, z2, z3] (tobillo, rodilla, cadera)

        Retorna:
            dict con posición del pie y ángulos DH
        """
        # Reordenar a [cadera, rodilla, tobillo] para las ecuaciones
        # angles_deg = [z1, z2, z3] -> [z3, z2, z1] = [cadera, rodilla, tobillo]
        ordered = [angles_deg[2], angles_deg[1], angles_deg[0]]
        dh_angles = servo_to_dh(ordered, self.dh)
        t1, t2, t3 = np.radians(dh_angles)

        L1, L2, L3 = self.config.L1, self.config.L2, self.config.L3

        # Posición del pie (ecuaciones de la tesis)
        x = L1 * np.sin(t1) + (L2 + L3 * np.cos(t3)) * np.sin(t1 + t2)
        y = L3 * np.sin(t3)
        z = L1 * np.cos(t1) + (L2 + L3 * np.cos(t3)) * np.cos(t1 + t2)

        return {
            'position': np.array([x, y, z]),
            'dh_angles': dh_angles,
        }

    def get_joint_positions(self, angles_deg: List[float]) -> np.ndarray:
        """
        Obtiene las posiciones (x, y, z) de todas las articulaciones
        en el sistema de coordenadas de la cadera.

        Usa ecuaciones analíticas paso a paso:
        - Cadera: (0, 0, 0)
        - Rodilla: (L1·sin(θ1), 0, L1·cos(θ1))
        - Tobillo: (L1·sin(θ1) + L2·sin(θ1+θ2), 0, L1·cos(θ1) + L2·cos(θ1+θ2))
        - Pie: (X_total, L3·sin(θ3), Z_total_con_L3)

        Parámetros:
            angles_deg: [ángulo_tobillo, ángulo_rodilla, ángulo_cadera] en grados
                       NOTA: En orden [z1, z2, z3] (tobillo, rodilla, cadera)

        Retorna:
            array de forma (4, 3) con [cadera, rodilla, tobillo, pie]
        """
        # Reordenar a [cadera, rodilla, tobillo]
        ordered = [angles_deg[2], angles_deg[1], angles_deg[0]]
        dh_angles = servo_to_dh(ordered, self.dh)
        t1, t2, t3 = np.radians(dh_angles)

        L1, L2, L3 = self.config.L1, self.config.L2, self.config.L3

        # Posiciones intermedias
        cadera = np.array([0.0, 0.0, 0.0])

        rodilla = np.array([
            L1 * np.sin(t1),
            0.0,
            L1 * np.cos(t1)
        ])

        tobillo = np.array([
            L1 * np.sin(t1) + L2 * np.sin(t1 + t2),
            0.0,
            L1 * np.cos(t1) + L2 * np.cos(t1 + t2)
        ])

        L2_eff = L2 + L3 * np.cos(t3)
        pie = np.array([
            L1 * np.sin(t1) + L2_eff * np.sin(t1 + t2),
            L3 * np.sin(t3),
            L1 * np.cos(t1) + L2_eff * np.cos(t1 + t2)
        ])

        return np.array([cadera, rodilla, tobillo, pie])

    def foot_position(self, angles_deg: List[float]) -> np.ndarray:
        """Retorna solo la posición (x, y, z) del pie."""
        result = self.forward_kinematics_analytic(angles_deg)
        return result['position']


class BipedRobot:
    """
    Modelo completo del robot bípedo con ambas piernas.
    """

    def __init__(self):
        self.config = RobotConfig()
        self.left_leg = LegKinematics(DH_LEFT_LEG, side="left")
        self.right_leg = LegKinematics(DH_RIGHT_LEG, side="right")

        # Offset lateral de cada cadera respecto al centro del torso
        self.half_hip = self.config.hip_separation / 2.0

    def forward_kinematics_both(
        self,
        angles_left: List[float],
        angles_right: List[float]
    ) -> dict:
        """
        Cinemática directa de ambas piernas.

        Parámetros:
            angles_left:  [z1, z2, z3] en grados (tobillo, rodilla, cadera izq)
            angles_right: [z4, z5, z6] en grados (tobillo, rodilla, cadera der)

        Retorna:
            dict con posiciones de todas las articulaciones en el sistema
            de coordenadas global (centro del torso)
        """
        # Pierna izquierda: offset +Y
        positions_left = self.left_leg.get_joint_positions(angles_left)
        positions_left[:, 1] += self.half_hip  # Offset lateral

        # Pierna derecha: offset -Y
        positions_right = self.right_leg.get_joint_positions(angles_right)
        positions_right[:, 1] -= self.half_hip  # Offset lateral

        return {
            'left': positions_left,      # [cadera, rodilla, tobillo, pie]
            'right': positions_right,    # [cadera, rodilla, tobillo, pie]
            'torso_center': np.array([0.0, 0.0, 0.0]),
        }

    def foot_positions(
        self,
        angles_left: List[float],
        angles_right: List[float]
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Posiciones de ambos pies en coordenadas globales."""
        foot_l = self.left_leg.foot_position(angles_left)
        foot_l[1] += self.half_hip

        foot_r = self.right_leg.foot_position(angles_right)
        foot_r[1] -= self.half_hip

        return foot_l, foot_r

    def analytic_foot_position(self, angles_deg: List[float]) -> dict:
        """
        Posición del pie usando las ecuaciones analíticas de la tesis
        (sección 5.0.4 - Cinemática directa resultante).

        Parámetros:
            angles_deg: [theta_1, theta_2, theta_3] en grados donde:
                theta_1 = ángulo cadera (home corrected)
                theta_2 = ángulo rodilla (home corrected)
                theta_3 = ángulo tobillo (home corrected)

        Retorna:
            dict con X_adelante, Y_lateral, Z_profundidad
        """
        t1, t2, t3 = np.radians(angles_deg)
        L1, L2, L3 = self.config.L1, self.config.L2, self.config.L3

        x = L1 * np.sin(t1) + (L2 + L3 * np.cos(t3)) * np.sin(t1 + t2)
        y = L3 * np.sin(t3)
        z = L1 * np.cos(t1) + (L2 + L3 * np.cos(t3)) * np.cos(t1 + t2)

        return {
            'X_adelante': x,
            'Y_lateral': y,
            'Z_profundidad': z,
        }
