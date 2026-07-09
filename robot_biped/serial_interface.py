"""
Interfaz Serial para Arduino
=============================
Comunicación en tiempo real con el Arduino Nano que controla
los servomotores SG90 del prototipo físico.

Protocolo:
    - Puerto: /dev/ttyUSB0 (Linux) o COMx (Windows)
    - Baudrate: 115200
    - Formato: "A,z1,z2,z3,z4,z5,z6\n"
    - Respuesta: "OK" o "ERR"
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
        Envía ángulos al Arduino.

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

    def send_sequence_step(self, servo: int, angle: float, delay_ms: int) -> bool:
        """
        Envía un paso de secuencia al Arduino.

        Parámetros:
            servo: Índice del servo (0-5)
            angle: Ángulo en grados
            delay_ms: Espera en ms

        Retorna:
            True si el envío fue exitoso
        """
        if not self.connected or not self.serial:
            return False

        angle = max(0, min(180, int(angle)))
        cmd = f"M,{servo},{angle},{delay_ms}\n"

        try:
            self.serial.write(cmd.encode('utf-8'))
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


# ============================================================
# CÓDIGO ARDUINO COMPLEMENTARIO (para subir al Nano)
# ============================================================
ARDUINO_FIRMWARE = '''
// Firmware para Arduino Nano - Robot Bípedo 12GDL
// Comunicación serial con PC para control de servomotores

#include <Servo.h>

Servo servos[6];
const uint8_t PINS[6] = {7, 6, 5, 4, 3, 2};

// Ángulos actuales
int currentAngles[6] = {90, 90, 90, 80, 90, 90};

// Buffer para lectura serial
String inputBuffer = "";

void setup() {
  Serial.begin(115200);

  // Attach servos
  for (int i = 0; i < 6; i++) {
    servos[i].attach(PINS[i]);
    servos[i].write(currentAngles[i]);
  }

  Serial.println("READY");
}

void loop() {
  // Leer comandos seriales
  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '\\n') {
      processCommand(inputBuffer);
      inputBuffer = "";
    } else {
      inputBuffer += c;
    }
  }
}

void processCommand(String cmd) {
  cmd.trim();

  if (cmd == "PING") {
    Serial.println("PONG");
    return;
  }

  if (cmd == "R") {
    // Report current angles
    Serial.print(currentAngles[0]);
    for (int i = 1; i < 6; i++) {
      Serial.print(",");
      Serial.print(currentAngles[i]);
    }
    Serial.println();
    return;
  }

  // Format: A,z1,z2,z3,z4,z5,z6
  if (cmd.startsWith("A,")) {
    int idx = 2;
    for (int i = 0; i < 6; i++) {
      int nextComma = cmd.indexOf(',', idx);
      String val = (nextComma == -1) ? cmd.substring(idx) : cmd.substring(idx, nextComma);
      int angle = val.toInt();
      angle = constrain(angle, 0, 180);
      currentAngles[i] = angle;
      servos[i].write(angle);
      idx = nextComma + 1;
    }
    Serial.println("OK");
    return;
  }

  // Format: M,servo,angle,delay
  if (cmd.startsWith("M,")) {
    int comma1 = cmd.indexOf(',', 2);
    int comma2 = cmd.indexOf(',', comma1 + 1);

    int servoIdx = cmd.substring(2, comma1).toInt();
    int angle = cmd.substring(comma1 + 1, comma2).toInt();
    int delayMs = cmd.substring(comma2 + 1).toInt();

    if (servoIdx >= 0 && servoIdx < 6) {
      angle = constrain(angle, 0, 180);
      currentAngles[servoIdx] = angle;
      servos[servoIdx].write(angle);
      delay(delayMs);
      Serial.println("OK");
    } else {
      Serial.println("ERR");
    }
    return;
  }

  Serial.println("ERR");
}
'''


def print_arduino_firmware():
    """Imprime el código Arduino para copiar y subir al Nano."""
    print("=" * 60)
    print("CÓDIGO ARDUINO - Copiar y subir al Nano")
    print("=" * 60)
    print(ARDUINO_FIRMWARE)
    print("=" * 60)
