"""
Cinemática Inversa
==================
Resolución analítica de la cinemática inversa para el robot bípedo
12GDL basado en las ecuaciones de la cinemática directa resultante
de la tesis (sección 5.0.4).

Ecuaciones directas:
    X = L1·sin(θ1) + (L2 + L3·cos(θ3))·sin(θ1 + θ2)
    Y = L3·sin(θ3)
    Z = L1·cos(θ1) + (L2 + L3·cos(θ3))·cos(θ1 + θ2)

Sistema de coordenadas (desde la cadera):
    X = adelante (+) / atrás (-)
    Y = lateral (rol del tobillo)
    Z = profundidad hacia abajo (+) = distancia desde la cadera
"""

import numpy as np
from typing import Tuple, Optional, List
from robot_biped.dh_kinematics import RobotConfig, servo_to_dh, dh_to_servo, DH_LEFT_LEG


class InverseKinematics:
    """
    Resuelve la cinemática inversa para posicionar el pie en (X, Y, Z).

    El sistema de coordenadas usa:
        X: dirección de marcha (adelante positivo)
        Y: dirección lateral (rol del tobillo)
        Z: profundidad desde la cadera (abajo positivo)

    Nota: Z es la distancia desde la cadera hacia abajo.
    El suelo está típicamente en Z = L1 + L2 + L3 = 11.5 cm
    """

    def __init__(self):
        self.config = RobotConfig()
        self.L1 = self.config.L1
        self.L2 = self.config.L2
        self.L3 = self.config.L3

    def solve(
        self,
        target_x: float,
        target_y: float,
        target_z: float,
        leg: str = "left",
        reference: Optional[Tuple[float, float, float]] = None
    ) -> Optional[Tuple[float, float, float]]:
        """
        Resuelve cinemática inversa para una pierna.

        Retorna ángulos de SERVO (z_cadera, z_rodilla, z_tobillo) en grados.

        Parámetros:
            target_x: Posición deseada adelante (+) / atrás (-) en cm
            target_y: Posición lateral (rol del tobillo) en cm
            target_z: Profundidad desde la cadera (+ = abajo) en cm
            leg: "left" o "right"
            reference: Ángulos de referencia (cadera, rodilla, tobillo) para
                       seleccionar la solución más cercana. Si es None, se usa
                       la posición HOME. Usar los ángulos del frame anterior
                       para evitar saltos entre ramas de solución.

        Retorna:
            (z_cadera, z_rodilla, z_tobillo) en grados, o None si no es alcanzable
        """
        X, Y, Z = target_x, target_y, target_z
        L1, L2, L3 = self.L1, self.L2, self.L3

        # --- Paso 1: Calcular θ3 desde Y = L3·sin(θ3) ---
        sin_t3 = Y / L3

        if abs(sin_t3) > 1.0:
            return None  # Posición no alcanzable

        t3 = np.arcsin(sin_t3)

        # --- Paso 2: Definir L2' = L2 + L3·cos(θ3) ---
        L2_prime = L2 + L3 * np.cos(t3)

        if abs(L2_prime) < 1e-6:
            return None

        # --- Paso 3: Resolver θ1 y θ2 desde X y Z ---
        # X = L1·sin(θ1) + L2'·sin(θ1 + θ2)
        # Z = L1·cos(θ1) + L2'·cos(θ1 + θ2)
        #
        # Sea φ = θ1 + θ2, entonces:
        # X = L1·sin(θ1) + L2'·sin(φ)
        # Z = L1·cos(θ1) + L2'·cos(φ)
        #
        # De donde:
        # (X - L1·sin(θ1))² + (Z - L1·cos(θ1))² = L2'²
        #
        # Expandir:
        # X² + Z² + L1² - 2·L1·X·sin(θ1) - 2·L1·Z·cos(θ1) = L2'²
        #
        # 2·L1·(X·sin(θ1) + Z·cos(θ1)) = X² + Z² + L1² - L2'²

        rhs = X**2 + Z**2 + L1**2 - L2_prime**2

        # Usar sustitución: X·sin(θ) + Z·cos(θ) = R·sin(θ + α)
        # donde R = sqrt(X² + Z²), α = atan2(Z, X)
        R_dist = np.sqrt(X**2 + Z**2)

        if R_dist < 1e-6:
            return None

        # X·sin(θ1) + Z·cos(θ1) = rhs / (2·L1)
        k = rhs / (2 * L1)

        # Sustitución trigonométrica
        # Sea ψ tal que cos(ψ) = X/R_dist, sin(ψ) = Z/R_dist
        # Entonces: R_dist·sin(θ1 + ψ) = k
        psi = np.arctan2(Z, X)
        sin_val = k / R_dist

        if abs(sin_val) > 1.0:
            return None  # No hay solución

        # Dos soluciones posibles para θ1
        theta1_sol1 = np.arcsin(sin_val) - psi
        theta1_sol2 = np.pi - np.arcsin(sin_val) - psi

        # Elegir la solución con θ1 más cercano a 0 (pierna más extendida)
        solutions = []
        for t1 in [theta1_sol1, theta1_sol2]:
            # Normalizar a [-π, π]
            t1 = np.arctan2(np.sin(t1), np.cos(t1))

            # Calcular φ = θ1 + θ2
            # sin(φ) = (X - L1·sin(θ1)) / L2'
            # cos(φ) = (Z - L1·cos(θ1)) / L2'
            sin_phi = (X - L1 * np.sin(t1)) / L2_prime
            cos_phi = (Z - L1 * np.cos(t1)) / L2_prime
            phi = np.arctan2(sin_phi, cos_phi)

            t2 = phi - t1
            t2 = np.arctan2(np.sin(t2), np.cos(t2))

            # Convertir ángulos DH a ángulos de servo
            # ang_servo = theta_DH - theta_offset
            # theta_offset = -90° para cadera y rodilla, -90° o -80° para tobillo
            dh_params = DH_LEFT_LEG  # mismo offset para ambas piernas excepto tobillo

            if leg == "left":
                servo_cadera = np.degrees(t1) - dh_params[0].theta_offset   # -(-90) = +90
                servo_rodilla = np.degrees(t2) - dh_params[1].theta_offset  # -(-90) = +90
                servo_tobillo = np.degrees(t3) - dh_params[2].theta_offset  # -(-90) = +90
            else:
                # Pierna derecha usa offsets ligeramente diferentes
                dh_r = [DH_LEFT_LEG[0], DH_LEFT_LEG[1], DH_LEFT_LEG[2]]
                dh_r[2] = type(dh_r[2])(**{**dh_r[2].__dict__, 'theta_offset': -80.0})
                servo_cadera = np.degrees(t1) - dh_r[0].theta_offset
                servo_rodilla = np.degrees(t2) - dh_r[1].theta_offset
                servo_tobillo = np.degrees(t3) - dh_r[2].theta_offset

            # Validar límites articulares (servos SG90: 0° - 180°)
            sv = [servo_cadera, servo_rodilla, servo_tobillo]

            if all(0 <= a <= 180 for a in sv):
                solutions.append(tuple(sv))

        if not solutions:
            return None

        # Usar todas las soluciones válidas (dentro del rango 0°-180°).
        # La selección por referencia (abajo) elige la más cercana al frame
        # anterior, manteniendo continuidad sin forzar dirección de rodilla.
        candidates = solutions

        # Elegir la solución más cercana a la referencia (frame anterior)
        # para mantener continuidad y evitar saltos entre ramas de solución.
        # Si no hay referencia, usar la posición HOME.
        ref = np.array(reference) if reference is not None else np.array([90.0, 90.0, 90.0])
        best = min(candidates, key=lambda s: np.sum((np.array(s) - ref)**2))

        return best

    def solve_trajectory(
        self,
        waypoints: List[Tuple[float, float, float]],
        leg: str = "left"
    ) -> List[Optional[Tuple[float, float, float]]]:
        """
        Resuelve cinemática inversa para una trayectoria completa.

        Parámetros:
            waypoints: Lista de (X, Y, Z) en cm
            leg: "left" o "right"

        Retorna:
            Lista de (z_cadera, z_rodilla, z_tobillo) en grados
        """
        return [self.solve(x, y, z, leg) for x, y, z in waypoints]

    def generate_walk_cycle(
        self,
        step_length: float = 3.0,
        step_height: float = 1.5,
        n_points: int = 40,
        leg_height: float = 11.5,
        lateral_shift: float = 1.0
    ) -> dict:
        """
        Genera una trayectoria de paso natural para ambas piernas,
        con inclinación perpendicular del pie para transferencia de peso.

        El ciclo de marcha se divide en fase de balanceo (swing, 40%)
        y fase de apoyo (stance, 60%), con interpolación cosenoidal
        para suavizar aceleraciones y transiciones.

        El pie (tobillo, θ3) es la articulación que se mueve
        perpendicularmente (lateral, eje Y). Las demás articulaciones
        (cadera θ1, rodilla θ2) se mueven al frente (plano sagital X-Z).

        Ecuación perpendicular: Y = L3·sin(θ3)
        Durante la fase de apoyo (stance), el pie se inclina lateralmente
        para desplazar el centro de gravedad sobre la pierna de soporte,
        permitiendo que la otra pierna se levante de forma estable.

        Parámetros:
            step_length: Longitud del paso en cm
            step_height: Altura máxima del pie sobre el suelo en cm
            n_points: Número total de puntos por ciclo
            leg_height: Longitud total de la pierna extendida (cm)
            lateral_shift: Desplazamiento lateral máximo del pie en cm.
                          Controla cuánto se inclina perpendicularmente
                          el pie durante apoyo (default: 1.0 cm ≈ 20° tobillo)

        Retorna:
            dict con waypoints para pierna izquierda y derecha
        """
        # Altura funcional del suelo: 95% de extensión total para que
        # el IK siempre tenga solución con offsets en X
        ground_z = leg_height * 0.95

        # Distribución natural: 40% swing, 60% stance
        n_swing = max(int(n_points * 0.4), 4)
        n_stance = n_points - n_swing

        # === FASE DE BALANCEO (swing): pierna avanza en arco ===
        t_swing = np.linspace(0, np.pi, n_swing)

        # X: avance con easing cosenoidal (arranca y frena suave)
        swing_x = step_length * (1 - np.cos(t_swing)) / 2 - step_length / 2

        # Z: arco sobre el suelo (la pierna se levanta y vuelve a bajar)
        swing_z = ground_z - step_height * np.sin(t_swing)

        # Y (perpendicular): durante el swing el pie está en el aire,
        # ligero movimiento lateral natural al avanzar
        swing_y = 0.3 * np.sin(t_swing)

        # === FASE DE APOYO (stance): pie en el suelo, cuerpo avanza ===
        # Interpolación cosenoidal para suavizar inicio/final
        t_stance_angle = np.linspace(0, np.pi, n_stance)
        t_stance = (1 - np.cos(t_stance_angle)) / 2  # easing [0→1]

        # X: pie retrocede suavemente (el cuerpo avanza sobre el pie fijo)
        stance_x = step_length / 2 - step_length * t_stance

        # Z: ligera variación vertical (cuerpo sube ~0.2cm en mid-stance,
        # simulando el efecto péndulo invertido de la marcha humana)
        stance_z = ground_z + 0.2 * np.sin(t_stance_angle)

        # Y (perpendicular): inclinación lateral del pie durante apoyo.
        # El pie se inclina perpendicularmente para transferir el peso
        # del robot sobre esta pierna, permitiendo que la otra se levante.
        # Perfil sinusoidal: se inclina al inicio del apoyo (cuando la otra
        # pierna comienza a levantarse), máximo a mitad de stance, y
        # vuelve a neutral al final (cuando la otra pierna aterriza).
        stance_y = lateral_shift * np.sin(t_stance_angle)

        # Combinar fases
        traj_x = np.concatenate([swing_x, stance_x])
        traj_y = np.concatenate([swing_y, stance_y])
        traj_z = np.concatenate([swing_z, stance_z])

        # Pierna izquierda balancea primero
        left_waypoints = list(zip(traj_x, traj_y, traj_z))

        # Pierna derecha desfasada medio ciclo, con Y espejado
        half = n_points // 2
        right_waypoints = list(zip(
            np.roll(traj_x, half),
            np.roll(-traj_y, half),   # Espejo lateral para pierna derecha
            np.roll(traj_z, half)
        ))

        return {
            'left': left_waypoints,
            'right': right_waypoints,
        }

