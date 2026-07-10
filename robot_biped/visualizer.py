"""
Visualizador 3D del Robot Bípedo
=================================
Visualización interactiva con matplotlib 3D del robot bípedo 12GDL.
Muestra el modelo esquelético con articulaciones, eslabones y trayectorias.

Sistema de coordenadas de visualización:
    X = adelante (+) / atrás (-)
    Y = lateral izquierdo (+) / derecho (-)
    Z = altura (+ arriba, - abajo)

La cinemática interna usa Z+ hacia abajo (convención tesis), por lo que
se invierte Z para la visualización.
"""

import os
import numpy as np
# En sesiones Wayland, Qt5 intenta usar xcb (X11) por defecto y falla.
# Configurar el plugin de plataforma a 'wayland' si estamos en una sesión Wayland.
if os.environ.get('XDG_SESSION_TYPE') == 'wayland' and 'QT_QPA_PLATFORM' not in os.environ:
    os.environ['QT_QPA_PLATFORM'] = 'wayland'
import matplotlib
# Seleccionar backend interactivo automáticamente.
# Se verifica importando el módulo del backend para detectar dependencias faltantes
# (e.g., tkinter no instalado). Si ninguno está disponible, se mantiene Agg.
for _backend in ('QtAgg', 'Qt5Agg', 'TkAgg', 'GTK3Agg'):
    try:
        matplotlib.use(_backend)
        __import__(f'matplotlib.backends.backend_{_backend.lower()}')
        break
    except (ImportError, ModuleNotFoundError):
        matplotlib.use('Agg')  # Reset al default antes de probar el siguiente
        continue
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from typing import List, Optional, Tuple
import time

from robot_biped.dh_kinematics import BipedRobot, HOME_LEFT, HOME_RIGHT
from robot_biped.arduino_sequences import SequencePlayer, AVAILABLE_SEQUENCES


class RobotVisualizer:
    """
    Visualizador 3D interactivo del robot bípedo.
    """

    # Colores del robot
    COLOR_LEFT_LEG = '#E74C3C'      # Rojo
    COLOR_RIGHT_LEG = '#3498DB'     # Azul
    COLOR_TORSO = '#2ECC71'         # Verde
    COLOR_JOINTS = '#F39C12'        # Naranja
    COLOR_GROUND = '#95A5A6'        # Gris
    COLOR_TRAJECTORY_L = '#E74C3C'  # Rojo claro
    COLOR_TRAJECTORY_R = '#3498DB'  # Azul claro

    def __init__(self, figsize: Tuple[int, int] = (14, 10)):
        self.robot = BipedRobot()
        self.player = SequencePlayer()

        # Figura y ejes 3D
        self.fig = plt.figure(figsize=figsize)
        self.ax = self.fig.add_subplot(111, projection='3d')

        # Líneas del robot (inicializadas vacías)
        self.line_left = None
        self.line_right = None
        self.line_torso = None

        # Trayectorias
        self.trajectory_left: List[np.ndarray] = []
        self.trajectory_right: List[np.ndarray] = []
        self.traj_line_left = None
        self.traj_line_right = None

        # Info text
        self._info_text = None

        # Configuración de la vista
        self._setup_axes()

        # Animación
        self.anim = None
        self.is_paused = False

    def _to_display_coords(self, pos: np.ndarray) -> np.ndarray:
        """
        Convierte coordenadas de la cinemática (Z+ abajo) a coordenadas
        de visualización (Z+ arriba).
        """
        result = pos.copy()
        result[2] = -result[2]  # Invertir Z
        return result

    def _setup_axes(self):
        """Configura los ejes 3D."""
        self.ax.set_xlabel('X (adelante) [cm]', fontsize=10, fontweight='bold')
        self.ax.set_ylabel('Y (lateral) [cm]', fontsize=10, fontweight='bold')
        self.ax.set_zlabel('Z (altura) [cm]', fontsize=10, fontweight='bold')
        self.ax.set_title(
            'Robot Bípedo 12GDL - Modelo Virtual DH\n'
            'Félix David Henríquez Córdoba - UCC 2026',
            fontsize=12, fontweight='bold', pad=20
        )

        # Límites de visualización
        # X: -15 a 15 (adelante/atrás)
        # Y: -12 a 12 (lateral)
        # Z: -14 a 5 (altura: suelo en -11.5, cadera en 0)
        self.ax.set_xlim(-15, 15)
        self.ax.set_ylim(-12, 12)
        self.ax.set_zlim(-14, 5)

        # Vista isométrica
        self.ax.view_init(elev=20, azim=-60)

        # Grid
        self.ax.grid(True, alpha=0.3)

        # Suelo (plano en Z = -11.5, que es -(L1+L2+L3))
        xx, yy = np.meshgrid(
            np.linspace(-15, 15, 2),
            np.linspace(-12, 12, 2)
        )
        zz = np.full_like(xx, -11.5)  # Suelo
        self.ax.plot_surface(xx, yy, zz, alpha=0.15, color=self.COLOR_GROUND)

        # Línea del suelo
        self.ax.plot([-15, 15], [0, 0], [-11.5, -11.5],
                     'k--', alpha=0.3, linewidth=1)

    def _draw_robot(self, angles_left: List[float], angles_right: List[float]):
        """
        Dibuja el robot en la posición dada.

        Parámetros:
            angles_left: [z1, z2, z3] en grados
            angles_right: [z4, z5, z6] en grados
        """
        # Obtener posiciones de las articulaciones (Z+ abajo en cinemática)
        kin = self.robot.forward_kinematics_both(angles_left, angles_right)

        pos_left = kin['left']    # [cadera, rodilla, tobillo, pie]
        pos_right = kin['right']  # [cadera, rodilla, tobillo, pie]
        torso = kin['torso_center']

        # Convertir a coordenadas de visualización (invertir Z)
        disp_left = np.array([self._to_display_coords(p) for p in pos_left])
        disp_right = np.array([self._to_display_coords(p) for p in pos_right])
        disp_torso = self._to_display_coords(torso)

        # --- Dibujar pierna izquierda ---
        if self.line_left is None:
            self.line_left, = self.ax.plot(
                disp_left[:, 0], disp_left[:, 1], disp_left[:, 2],
                'o-', color=self.COLOR_LEFT_LEG, linewidth=4, markersize=10,
                markerfacecolor=self.COLOR_JOINTS, markeredgecolor='black',
                markeredgewidth=1.5, label='Pierna Izquierda'
            )
        else:
            self.line_left.set_data(disp_left[:, 0], disp_left[:, 1])
            self.line_left.set_3d_properties(disp_left[:, 2])

        # --- Dibujar pierna derecha ---
        if self.line_right is None:
            self.line_right, = self.ax.plot(
                disp_right[:, 0], disp_right[:, 1], disp_right[:, 2],
                'o-', color=self.COLOR_RIGHT_LEG, linewidth=4, markersize=10,
                markerfacecolor=self.COLOR_JOINTS, markeredgecolor='black',
                markeredgewidth=1.5, label='Pierna Derecha'
            )
        else:
            self.line_right.set_data(disp_right[:, 0], disp_right[:, 1])
            self.line_right.set_3d_properties(disp_right[:, 2])

        # --- Dibujar torso ---
        torso_line = np.array([
            disp_torso,
            disp_left[0],   # cadera izquierda
            disp_torso,
            disp_right[0],  # cadera derecha
        ])

        if self.line_torso is None:
            self.line_torso, = self.ax.plot(
                torso_line[:, 0], torso_line[:, 1], torso_line[:, 2],
                's-', color=self.COLOR_TORSO, linewidth=6, markersize=12,
                markerfacecolor=self.COLOR_TORSO, markeredgecolor='darkgreen',
                markeredgewidth=2, label='Torso'
            )
        else:
            self.line_torso.set_data(torso_line[:, 0], torso_line[:, 1])
            self.line_torso.set_3d_properties(torso_line[:, 2])

        # --- Actualizar trayectorias ---
        foot_l = disp_left[-1]  # Pie izquierdo en coords de display
        foot_r = disp_right[-1]  # Pie derecho en coords de display

        self.trajectory_left.append(foot_l.copy())
        self.trajectory_right.append(foot_r.copy())

        # Limitar longitud de trayectoria
        max_traj = 200
        if len(self.trajectory_left) > max_traj:
            self.trajectory_left = self.trajectory_left[-max_traj:]
            self.trajectory_right = self.trajectory_right[-max_traj:]

        # Dibujar trayectorias
        if len(self.trajectory_left) > 1:
            traj_l = np.array(self.trajectory_left)
            if self.traj_line_left is None:
                self.traj_line_left, = self.ax.plot(
                    traj_l[:, 0], traj_l[:, 1], traj_l[:, 2],
                    '.', color=self.COLOR_TRAJECTORY_L, alpha=0.3, markersize=2
                )
            else:
                self.traj_line_left.set_data(traj_l[:, 0], traj_l[:, 1])
                self.traj_line_left.set_3d_properties(traj_l[:, 2])

        if len(self.trajectory_right) > 1:
            traj_r = np.array(self.trajectory_right)
            if self.traj_line_right is None:
                self.traj_line_right, = self.ax.plot(
                    traj_r[:, 0], traj_r[:, 1], traj_r[:, 2],
                    '.', color=self.COLOR_TRAJECTORY_R, alpha=0.3, markersize=2
                )
            else:
                self.traj_line_right.set_data(traj_r[:, 0], traj_r[:, 1])
                self.traj_line_right.set_3d_properties(traj_r[:, 2])

        # --- Información en pantalla ---
        info_text = (
            f"Pierna Izq: z1={angles_left[0]:.1f} z2={angles_left[1]:.1f} z3={angles_left[2]:.1f}\n"
            f"Pierna Der: z4={angles_right[0]:.1f} z5={angles_right[1]:.1f} z6={angles_right[2]:.1f}\n"
            f"Pie Izq: ({pos_left[-1][0]:.1f}, {pos_left[-1][1]:.1f}, {pos_left[-1][2]:.1f})\n"
            f"Pie Der: ({pos_right[-1][0]:.1f}, {pos_right[-1][1]:.1f}, {pos_right[-1][2]:.1f})"
        )

        if self._info_text is not None:
            self._info_text.remove()

        self._info_text = self.ax.text2D(
            0.02, 0.98, info_text,
            transform=self.ax.transAxes,
            fontsize=9, verticalalignment='top',
            fontfamily='monospace',
            bbox=dict(boxstyle='round', facecolor='white', alpha=0.8)
        )

        # Ejes de coordenadas en el origen (solo una vez)
        if not hasattr(self, '_axes_drawn'):
            self._draw_coordinate_axes()
            self._axes_drawn = True

    def _draw_coordinate_axes(self, origin=None, length=2):
        """Dibuja ejes de coordenadas XYZ en el centro del torso."""
        if origin is None:
            origin = np.zeros(3)

        # X - rojo (adelante)
        self.ax.quiver(*origin, length, 0, 0, color='red', arrow_length_ratio=0.3, linewidth=2)
        # Y - verde (lateral izquierdo)
        self.ax.quiver(*origin, 0, length, 0, color='green', arrow_length_ratio=0.3, linewidth=2)
        # Z - azul (arriba)
        self.ax.quiver(*origin, 0, 0, length, color='blue', arrow_length_ratio=0.3, linewidth=2)

        self.ax.text(origin[0] + length + 0.5, origin[1], origin[2], 'X', color='red', fontweight='bold')
        self.ax.text(origin[0], origin[1] + length + 0.5, origin[2], 'Y', color='green', fontweight='bold')
        self.ax.text(origin[0], origin[1], origin[2] + length + 0.5, 'Z', color='blue', fontweight='bold')

    def show_static(self, angles_left: List[float], angles_right: List[float]):
        """
        Muestra el robot en una posición estática.

        Parámetros:
            angles_left: [z1, z2, z3] en grados
            angles_right: [z4, z5, z6] en grados
        """
        self._draw_robot(angles_left, angles_right)
        self.ax.legend(loc='upper right', fontsize=9)
        plt.tight_layout()
        plt.show(block=True)

    def animate_sequence(
        self,
        sequence: List,
        interval_ms: int = 100,
        loop: bool = True
    ):
        """
        Anima una secuencia de movimiento del Arduino.

        Parámetros:
            sequence: Lista de movimientos (clase Movement)
            interval_ms: Intervalo entre frames en ms
            loop: Si True, repite la animación
        """
        self.player.reset_to_home()
        self.trajectory_left = []
        self.trajectory_right = []

        n_frames = len(sequence)

        def init():
            self.player.reset_to_home()
            left, right = self.player.get_leg_angles()
            self._draw_robot(left, right)
            return []

        def update(frame):
            if not self.is_paused:
                self.player.execute_step(sequence)
                left, right = self.player.get_leg_angles()
                self._draw_robot(left, right)
            return []

        self.anim = FuncAnimation(
            self.fig, update, init_func=init,
            frames=n_frames, interval=interval_ms,
            blit=False, repeat=loop
        )

        self._setup_controls()
        plt.tight_layout()
        plt.show(block=True)

    def animate_ik_walk(
        self,
        step_length: float = 3.0,
        step_height: float = 1.5,
        n_points: int = 50,
        interval_ms: int = 80
    ):
        """
        Anima una caminata generada por cinemática inversa.

        Parámetros:
            step_length: Longitud del paso en cm
            step_height: Altura del paso en cm
            n_points: Puntos por ciclo
            interval_ms: Intervalo entre frames
        """
        from robot_biped.inverse_kinematics import InverseKinematics

        ik = InverseKinematics()
        walk = ik.generate_walk_cycle(step_length, step_height, n_points)

        # Resolver IK para ambas piernas
        left_angles_list = []
        right_angles_list = []

        # Último resultado válido: sirve como referencia para el IK
        # (evita saltos entre ramas de solución) y como fallback si falla.
        last_la = (90.0, 90.0, 90.0)  # HOME izq: (cadera, rodilla, tobillo)
        last_ra = (90.0, 90.0, 80.0)  # HOME der: (cadera, rodilla, tobillo)

        for (lx, ly, lz), (rx, ry, rz) in zip(walk['left'], walk['right']):
            # Pasar ángulos anteriores como referencia para mantener continuidad
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

            # IK retorna (cadera, rodilla, tobillo) pero _draw_robot espera
            # [z1, z2, z3] = [tobillo, rodilla, cadera]. Invertir orden.
            left_angles_list.append((la[2], la[1], la[0]))
            right_angles_list.append((ra[2], ra[1], ra[0]))

        self.trajectory_left = []
        self.trajectory_right = []

        n_frames = len(left_angles_list)
        self._frame = 0

        def init():
            self._frame = 0
            la = left_angles_list[0]
            ra = right_angles_list[0]
            self._draw_robot(list(la), list(ra))
            return []

        def update(frame):
            if not self.is_paused:
                la = left_angles_list[self._frame]
                ra = right_angles_list[self._frame]
                self._draw_robot(list(la), list(ra))
                self._frame = (self._frame + 1) % n_frames
            return []

        self.anim = FuncAnimation(
            self.fig, update, init_func=init,
            frames=n_frames, interval=interval_ms,
            blit=False, repeat=True
        )

        self._setup_controls()
        plt.tight_layout()
        plt.show(block=True)

    def animate_realtime(self, angle_source, interval_ms: int = 50):
        """
        Anima con fuente de ángulos en tiempo real.

        Parámetros:
            angle_source: Callable que retorna (angles_left, angles_right)
            interval_ms: Intervalo de actualización
        """
        self.trajectory_left = []
        self.trajectory_right = []

        def update(frame):
            if not self.is_paused:
                result = angle_source()
                if result:
                    left, right = result
                    self._draw_robot(left, right)
            return []

        self.anim = FuncAnimation(
            self.fig, update,
            interval=interval_ms,
            blit=False, repeat=True
        )

        self._setup_controls()
        plt.tight_layout()
        plt.show(block=True)

    def _setup_controls(self):
        """Configura controles de teclado."""
        self.fig.canvas.mpl_connect('key_press_event', self._on_key)

        # Texto de ayuda
        help_text = (
            "Controles:\n"
            "[ESPACIO] Pausar/Reanudar\n"
            "[R] Resetear vista\n"
            "[ESC/Q] Salir"
        )
        self.ax.text2D(
            0.98, 0.02, help_text,
            transform=self.ax.transAxes,
            fontsize=8, verticalalignment='bottom', horizontalalignment='right',
            fontfamily='monospace',
            bbox=dict(boxstyle='round', facecolor='lightgray', alpha=0.7)
        )

    def _on_key(self, event):
        """Manejador de eventos de teclado."""
        if event.key == ' ':
            self.is_paused = not self.is_paused
        elif event.key.lower() == 'r':
            self.ax.view_init(elev=20, azim=-60)
            self.fig.canvas.draw()
        elif event.key.lower() in ('q', 'escape'):
            plt.close(self.fig)
