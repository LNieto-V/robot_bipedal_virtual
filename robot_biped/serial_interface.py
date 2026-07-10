"""
Interfaz Serial para Arduino
=============================
Comunicación en tiempo real con el Arduino Nano que controla
los servomotores SG90 del prototipo físico.

Protocolo:
    - Puerto: /dev/ttyUSB0 (Linux) o COMx (Windows)
    - Baudrate: 115200
    - Comandos:
        "PING\\n"                     → "PONG"
        "A,z1,z2,z3,z4,z5,z6\\n"     → Mover instantáneo
        "S,z1,z2,z3,z4,z5,z6,ms\\n"  → Mover con interpolación
        "W\\n"                        → Iniciar caminata IK autónoma
        "H\\n"                        → Volver a HOME
        "X\\n"                        → Detener caminata
        "R\\n"                        → Reportar ángulos actuales
"""

import serial
import serial.tools.list_ports
import numpy as np
import time
from typing import Optional, List, Callable
import threading


class ArduinoInterface:
    """
    Interfaz de comunicación con Arduino Nano.

    Soporta el protocolo completo del firmware robot_bipedo_nano.ino:
    control instantáneo, interpolado, caminata autónoma y lectura de ángulos.
    """

    DEFAULT_BAUDRATE = 115200
    TIMEOUT = 2.0

    def __init__(self, port: Optional[str] = None, baudrate: int = DEFAULT_BAUDRATE):
        """
        Inicializa la interfaz serial.

        Parámetros:
            port: Puerto serial (auto-detecta si es None)
            baudrate: Velocidad de comunicación
        """
        self.port = port
        self.baudrate = baudrate
        self.serial: Optional[serial.Serial] = None
        self.connected = False
        self._read_thread: Optional[threading.Thread] = None
        self._running = False
        self._callback: Optional[Callable] = None

    @staticmethod
    def list_ports() -> List[str]:
        """Lista los puertos seriales disponibles."""
        return [p.device for p in serial.tools.list_ports.comports()]

    @staticmethod
    def find_arduino() -> Optional[str]:
        """
        Intenta detectar automáticamente el puerto del Arduino.

        Retorna:
            Ruta del puerto o None si no se encuentra
        """
        ports = serial.tools.list_ports.comports()

        for p in ports:
            # Patrones comunes de identificación de Arduino
            desc = p.description.lower()
            hwid = p.hwid.lower()

            if any(x in desc for x in ['arduino', 'ch340', 'ftdi', 'usb-serial']):
                return p.device

            if 'vid:pid=1a86:7523' in hwid:  # CH340 (clones Arduino)
                return p.device
            if 'vid:pid=2341' in hwid:        # Arduino original
                return p.device

        # Si solo hay un puerto, intentar con ese
        if len(ports) == 1:
            return ports[0].device

        return None

    def connect(self) -> bool:
        """
        Conecta al Arduino.

        Retorna:
            True si la conexión fue exitosa
        """
        if self.connected:
            return True

        # Auto-detectar puerto si no se especificó
        if self.port is None:
            self.port = self.find_arduino()
            if self.port is None:
                raise ConnectionError(
                    "No se encontró Arduino. Puertos disponibles: " +
                    str(self.list_ports())
                )

        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.TIMEOUT,
                write_timeout=self.TIMEOUT
            )

            # Esperar a que el Arduino se reinicie
            time.sleep(2.5)

            # Limpiar buffer
            self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()

            # Verificar conexión
            self.serial.write(b"PING\n")
            time.sleep(0.1)

            if self.serial.in_waiting:
                response = self.serial.readline().decode('utf-8').strip()
                self.connected = True
                print(f"[OK] Arduino conectado en {self.port} ({response})")
                return True

            # Si no responde PING, intentar de todos modos
            self.connected = True
            print(f"[OK] Arduino conectado en {self.port}")
            return True

        except serial.SerialException as e:
            raise ConnectionError(f"Error al conectar: {e}")

    def disconnect(self):
        """Desconecta del Arduino."""
        self._running = False
        if self._read_thread:
            self._read_thread.join(timeout=1.0)

        if self.serial and self.serial.is_open:
            self.serial.close()

        self.connected = False
        print("[OK] Arduino desconectado")

    def send_angles(self, angles: List[float]) -> bool:
        """
        Envía ángulos al Arduino (movimiento instantáneo).

        Parámetros:
            angles: [z1, z2, z3, z4, z5, z6] en grados

        Retorna:
            True si el envío fue exitoso
        """
        if not self.connected or not self.serial:
            return False

        # Validar rangos
        clamped = [max(0, min(180, int(a))) for a in angles]

        # Formato: "A,90,90,90,80,90,90\n"
        cmd = f"A,{clamped[0]},{clamped[1]},{clamped[2]}," \
              f"{clamped[3]},{clamped[4]},{clamped[5]}\n"

        try:
            self.serial.write(cmd.encode('utf-8'))
            self.serial.flush()
            return True
        except serial.SerialException:
            self.connected = False
            return False

    def send_smooth_angles(self, angles: List[float], duration_ms: int = 80) -> bool:
        """
        Envía ángulos al Arduino con interpolación suave.

        El firmware interpola desde la posición actual hacia la objetivo
        en 'duration_ms' milisegundos, con pasos de ~10ms.

        Parámetros:
            angles: [z1, z2, z3, z4, z5, z6] en grados
            duration_ms: Duración de la transición en ms

        Retorna:
            True si el envío fue exitoso
        """
        if not self.connected or not self.serial:
            return False

        clamped = [max(0, min(180, int(a))) for a in angles]

        # Formato: "S,90,90,90,80,90,90,80\n"
        cmd = f"S,{clamped[0]},{clamped[1]},{clamped[2]}," \
              f"{clamped[3]},{clamped[4]},{clamped[5]},{duration_ms}\n"

        try:
            self.serial.write(cmd.encode('utf-8'))
            self.serial.flush()
            return True
        except serial.SerialException:
            self.connected = False
            return False

    def send_walk_command(self) -> bool:
        """
        Inicia la caminata autónoma IK en el Arduino.

        El Arduino ejecuta la secuencia de 20 frames IK almacenada
        en PROGMEM de forma cíclica. Enviar 'X' para detener.

        Retorna:
            True si el comando fue enviado
        """
        if not self.connected or not self.serial:
            return False

        try:
            self.serial.write(b"W\n")
            self.serial.flush()
            return True
        except serial.SerialException:
            self.connected = False
            return False

    def send_stop_command(self) -> bool:
        """Detiene la caminata autónoma."""
        if not self.connected or not self.serial:
            return False

        try:
            self.serial.write(b"X\n")
            self.serial.flush()
            return True
        except serial.SerialException:
            self.connected = False
            return False

    def send_home_command(self) -> bool:
        """Envía al Arduino a la posición HOME."""
        if not self.connected or not self.serial:
            return False

        try:
            self.serial.write(b"H\n")
            self.serial.flush()
            return True
        except serial.SerialException:
            self.connected = False
            return False

    def read_angles(self) -> Optional[List[float]]:
        """
        Lee los ángulos actuales del Arduino.

        Retorna:
            Lista de 6 ángulos o None
        """
        if not self.connected or not self.serial:
            return None

        try:
            self.serial.write(b"R\n")
            time.sleep(0.05)

            if self.serial.in_waiting:
                line = self.serial.readline().decode('utf-8').strip()
                parts = line.split(',')
                if len(parts) == 6:
                    return [float(p) for p in parts]
        except (serial.SerialException, ValueError):
            pass

        return None

    def start_reading(self, callback: Callable[[List[float]], None]):
        """
        Inicia thread de lectura continua.

        Parámetros:
            callback: Función a llamar con los ángulos leídos
        """
        self._callback = callback
        self._running = True
        self._read_thread = threading.Thread(target=self._read_loop, daemon=True)
        self._read_thread.start()

    def _read_loop(self):
        """Loop de lectura continua."""
        while self._running:
            angles = self.read_angles()
            if angles and self._callback:
                self._callback(angles)
            time.sleep(0.05)

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, *args):
        self.disconnect()

