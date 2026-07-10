#!/usr/bin/env python3
"""
Robot Bípedo 12GDL - Modelo Virtual
====================================
Punto de entrada principal. Permite ejecutar el simulador en varios modos:

    1. Simulación offline con secuencias Arduino
    2. Simulación con cinemática inversa (caminata autónoma)
    3. Conexión en tiempo real con Arduino
    4. Visualización estática de posiciones específicas
    5. Test de matrices Denavit-Hartenberg

Uso:
    python -m robot_biped.main [opciones]

Autor: Félix David Henríquez Córdoba
Universidad Cooperativa de Colombia, 2026
"""

import argparse
import sys
import numpy as np

from robot_biped.dh_kinematics import BipedRobot, HOME_LEFT, HOME_RIGHT
from robot_biped.inverse_kinematics import InverseKinematics
from robot_biped.arduino_sequences import SequencePlayer, AVAILABLE_SEQUENCES
from robot_biped.visualizer import RobotVisualizer
from robot_biped.serial_interface import ArduinoInterface


def print_banner():
    """Muestra el banner de inicio."""
    print("""
    ╔═══════════════════════════════════════════════════════════╗
    ║                                                           ║
    ║   ROBOT BÍPEDO 12GDL - MODELO VIRTUAL                    ║
    ║   Cinemática Denavit-Hartenberg + Visualización 3D       ║
    ║                                                           ║
    ║   Félix David Henríquez Córdoba                          ║
    ║   Universidad Cooperativa de Colombia - 2026             ║
    ║                                                           ║
    ╚═══════════════════════════════════════════════════════════╝
    """)


def cmd_simulate_offline(args):
    """Modo simulación offline con secuencias Arduino IK."""
    print("[MODO] Simulación Offline - Secuencia IK Arduino")
    print(f"Secuencia: {args.sequence}")
    print(f"Velocidad: {args.speed}x")

    sequence = AVAILABLE_SEQUENCES.get(args.sequence)
    if sequence is None:
        print(f"[ERROR] Secuencia '{args.sequence}' no encontrada.")
        print(f"Disponibles: {list(AVAILABLE_SEQUENCES.keys())}")
        return

    base_interval = sequence[0].delay_ms
    interval = max(10, int(base_interval / args.speed))

    viz = RobotVisualizer()
    viz.animate_sequence(sequence, interval_ms=interval)


def cmd_simulate_ik(args):
    """Modo simulación con cinemática inversa."""
    print("[MODO] Simulación con Cinemática Inversa")
    print(f"Longitud de paso: {args.step_length} cm")
    print(f"Altura de paso: {args.step_height} cm")

    viz = RobotVisualizer()
    viz.animate_ik_walk(
        step_length=args.step_length,
        step_height=args.step_height,
        n_points=args.points,
        interval_ms=args.interval
    )


def cmd_static(args):
    """Modo visualización estática."""
    print("[MODO] Visualización Estática")

    if args.home:
        angles_left = list(HOME_LEFT)
        angles_right = list(HOME_RIGHT)
        print("Mostrando posición HOME")
    elif args.custom:
        vals = [float(x) for x in args.custom.split(',')]
        if len(vals) != 6:
            print("Error: Se requieren 6 ángulos (z1,z2,z3,z4,z5,z6)")
            return
        angles_left = vals[:3]
        angles_right = vals[3:]
        print(f"Ángulos personalizados: {vals}")
    else:
        angles_left = list(HOME_LEFT)
        angles_right = list(HOME_RIGHT)

    viz = RobotVisualizer()
    viz.show_static(angles_left, angles_right)


def cmd_arduino_live(args):
    """Modo conexión en tiempo real con Arduino."""
    print("[MODO] Conexión en Tiempo Real con Arduino")

    try:
        arduino = ArduinoInterface(port=args.port)
        arduino.connect()

        print(f"Conectado a: {arduino.port}")
        print("Presiona Ctrl+C para salir")

        viz = RobotVisualizer()
        viz.trajectory_left = []
        viz.trajectory_right = []

        # Enviar home inicial
        angles = list(HOME_LEFT) + list(HOME_RIGHT)
        arduino.send_angles(angles)

        # Función que lee ángulos del Arduino
        def get_arduino_angles():
            read = arduino.read_angles()
            if read:
                return read[:3], read[3:]
            return None

        viz.animate_realtime(get_arduino_angles, interval_ms=50)
        arduino.disconnect()

    except ConnectionError as e:
        print(f"[ERROR] {e}")
        print("\nAsegúrate de que:")
        print("  1. El Arduino está conectado por USB")
        print("  2. El firmware está cargado")
        print("  3. Tienes permisos para acceder al puerto serial")
        print("\nPuertos disponibles:")
        for p in ArduinoInterface.list_ports():
            print(f"    - {p}")


def cmd_firmware(args):
    """Muestra la ubicación del código Arduino."""
    import os
    firmware_path = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        'firmware', 'robot_bipedo_nano', 'robot_bipedo_nano.ino'
    )
    print("=" * 60)
    print("FIRMWARE ARDUINO - Robot Bípedo 12GDL")
    print("=" * 60)
    if os.path.exists(firmware_path):
        print(f"Archivo: {firmware_path}")
        print()
        with open(firmware_path, 'r') as f:
            print(f.read())
    else:
        print(f"[ERROR] No se encontró el firmware en: {firmware_path}")
        print("Debería estar en: firmware/robot_bipedo_nano/robot_bipedo_nano.ino")
    print("=" * 60)


def cmd_test_dh(args):
    """Modo test de matrices DH."""
    print("[MODO] Test de Matrices Denavit-Hartenberg")
    print()

    robot = BipedRobot()

    print("=" * 60)
    print("Parámetros DH - Pierna Izquierda")
    print("=" * 60)
    print(f"{'Art.':<12} {'a (cm)':<10} {'α (°)':<10} {'d (cm)':<10} {'θ offset':<10}")
    print("-" * 60)

    names = ["Cadera (z3)", "Rodilla (z2)", "Tobillo (z1)"]
    for name, dh in zip(names, robot.left_leg.dh):
        print(f"{name:<12} {dh.a:<10.1f} {dh.alpha:<10.1f} {dh.d:<10.1f} {dh.theta_offset:<10.1f}")

    print()
    print("=" * 60)
    print("Parámetros DH - Pierna Derecha")
    print("=" * 60)
    print(f"{'Art.':<12} {'a (cm)':<10} {'α (°)':<10} {'d (cm)':<10} {'θ offset':<10}")
    print("-" * 60)

    names_r = ["Cadera (z6)", "Rodilla (z5)", "Tobillo (z4)"]
    for name, dh in zip(names_r, robot.right_leg.dh):
        print(f"{name:<12} {dh.a:<10.1f} {dh.alpha:<10.1f} {dh.d:<10.1f} {dh.theta_offset:<10.1f}")

    # Test cinemática directa en home
    print()
    print("=" * 60)
    print("Test Cinemática Directa - Posición HOME")
    print("=" * 60)

    kin = robot.forward_kinematics_both(HOME_LEFT, HOME_RIGHT)
    pos_l = kin['left']
    pos_r = kin['right']

    print(f"\nPierna Izquierda (HOME: z1=90°, z2=90°, z3=90°):")
    labels = ["Cadera", "Rodilla", "Tobillo", "Pie"]
    for label, pos in zip(labels, pos_l):
        print(f"  {label:<10} → ({pos[0]:7.2f}, {pos[1]:7.2f}, {pos[2]:7.2f}) cm")

    print(f"\nPierna Derecha (HOME: z4=80°, z5=90°, z6=90°):")
    for label, pos in zip(labels, pos_r):
        print(f"  {label:<10} → ({pos[0]:7.2f}, {pos[1]:7.2f}, {pos[2]:7.2f}) cm")

    # Test cinemática inversa
    print()
    print("=" * 60)
    print("Test Cinemática Inversa")
    print("=" * 60)

    ik = InverseKinematics()

    # Probar IK con puntos objetivo alcanzables
    # Z es profundidad desde la cadera (abajo positivo)
    # Suelo está en Z = L1 + L2 + L3 = 11.5 cm
    # Los pasos adelante requieren Z < 11.5 (la pierna se inclina)
    test_points = [
        (0.0, 0.0, 11.5, "Pierna extendida (HOME)"),
        (3.0, 0.0, 8.0,  "Paso adelante"),
        (-2.0, 0.0, 9.0, "Paso atrás"),
        (2.0, 1.0, 8.0,  "Paso con altura y rol"),
        (0.0, 0.0, 9.0,  "Agachamiento"),
        (5.0, 0.0, 6.0,  "Gran paso"),
    ]

    print(f"\n{'Objetivo':<25} {'(X, Y, Z) cm':<25} {'Ángulos servo (°)':<35} {'Error (cm)'}")
    print("-" * 100)

    for x, y, z, desc in test_points:
        result = ik.solve(x, y, z, "left")

        if result:
            servo_cad, servo_rod, servo_tob = result
            angles_str = f"z3={servo_cad:.1f}°, z2={servo_rod:.1f}°, z1={servo_tob:.1f}°"

            # Verificar con cinemática directa
            # Pasar en orden [z1, z2, z3] = [tobillo, rodilla, cadera]
            robot_fk = robot.left_leg.foot_position([servo_tob, servo_rod, servo_cad])
            err = np.linalg.norm(robot_fk - np.array([x, y, z]))
            status = f"{err:.4f}"
        else:
            angles_str = "Sin solución"
            status = "FALLA"

        xyz_str = f"({x:5.1f}, {y:5.1f}, {z:5.1f})"
        print(f"{desc:<25} {xyz_str:<25} {angles_str:<35} {status}")

    print()


def main():
    """Punto de entrada principal."""
    print_banner()

    parser = argparse.ArgumentParser(
        description='Robot Bípedo 12GDL - Modelo Virtual con Cinemática DH',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Ejemplos:
  # Simulación offline con secuencia de caminata
  python -m robot_biped.main simulate-offline

  # Simulación con cinemática inversa
  python -m robot_biped.main simulate-ik --step-length 4 --step-height 2

  # Visualización estática de posición home
  python -m robot_biped.main static --home

  # Conexión con Arduino físico
  python -m robot_biped.main arduino-live

  # Ver matrices DH
  python -m robot_biped.main test-dh

  # Mostrar firmware Arduino
  python -m robot_biped.main firmware
        """
    )

    subparsers = parser.add_subparsers(dest='command', help='Comando a ejecutar')

    # --- Simulación Offline ---
    p_off = subparsers.add_parser('simulate-offline', help='Simulación con secuencias Arduino')
    p_off.add_argument('--sequence', choices=list(AVAILABLE_SEQUENCES.keys()),
                       default='ik_walk', help='Secuencia a reproducir (default: ik_walk)')
    p_off.add_argument('--speed', type=float, default=1.0,
                       help='Velocidad de reproducción (default: 1x)')

    # --- Simulación IK ---
    p_ik = subparsers.add_parser('simulate-ik', help='Simulación con cinemática inversa')
    p_ik.add_argument('--step-length', type=float, default=3.0, help='Longitud del paso (cm)')
    p_ik.add_argument('--step-height', type=float, default=1.5, help='Altura del paso (cm)')
    p_ik.add_argument('--points', type=int, default=50, help='Puntos por ciclo')
    p_ik.add_argument('--interval', type=int, default=80, help='Intervalo entre frames (ms)')

    # --- Visualización Estática ---
    p_stat = subparsers.add_parser('static', help='Visualización estática')
    p_stat.add_argument('--home', action='store_true', help='Mostrar posición home')
    p_stat.add_argument('--custom', type=str, help='Ángulos personalizados z1,z2,z3,z4,z5,z6')

    # --- Arduino Live ---
    p_live = subparsers.add_parser('arduino-live', help='Conexión en tiempo real con Arduino')
    p_live.add_argument('--port', type=str, default=None, help='Puerto serial (auto-detecta)')

    # --- Firmware ---
    subparsers.add_parser('firmware', help='Mostrar código Arduino')

    # --- Test DH ---
    subparsers.add_parser('test-dh', help='Test de matrices Denavit-Hartenberg')

    args = parser.parse_args()

    if not args.command:
        parser.print_help()
        return

    # Dispatch
    commands = {
        'simulate-offline': cmd_simulate_offline,
        'simulate-ik': cmd_simulate_ik,
        'static': cmd_static,
        'arduino-live': cmd_arduino_live,
        'firmware': cmd_firmware,
        'test-dh': cmd_test_dh,
    }

    cmd_func = commands.get(args.command)
    if cmd_func:
        cmd_func(args)
    else:
        parser.print_help()


if __name__ == '__main__':
    main()
