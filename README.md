# 🤖 Robot Bípedo 12GDL - Modelo Virtual

**Modelo virtual de locomoción bípeda con cinemática Denavit-Hartenberg, visualización 3D interactiva y control de prototipo físico Arduino.**

---

## 📋 Información del Proyecto

- **Autor:** Félix David Henríquez Córdoba
- **Institución:** Universidad Cooperativa de Colombia
- **Programa:** Ingeniería Electrónica
- **Año:** 2026
- **Tesis:** *Generación de patrones de marcha en robots bípedos de 12 GDL mediante modelado Denavit-Hartenberg y resolución de cinemática inversa*

---

## 🏗️ Arquitectura del Robot

```
                    [Torso]
                      |
        +-------------+-------------+
        |                           |
   [Cadera L]                 [Cadera R]    ← z3 (D5), z6 (D2)
        |                           |
   [Rodilla L]                [Rodilla R]    ← z2 (D6), z5 (D3)
        |                           |
   [Tobillo L]                [Tobillo R]    ← z1 (D7), z4 (D4)
        |                           |
   [Pie L]                    [Pie R]
```

**Parámetros geométricos:**
| Eslabón | Longitud |
|---------|----------|
| L1 (Cadera → Rodilla) | 4.5 cm |
| L2 (Rodilla → Tobillo) | 4.0 cm |
| L3 (Tobillo → Pie) | 3.0 cm |
| Separación cadera-cadera | 10.0 cm |

---

## 🚀 Instalación

### Requisitos
- Python 3.10+
- [UV](https://docs.astral.sh/uv/) (gestor de paquetes)

### Instalar dependencias

```bash
cd robot_bipedal_virtual
uv sync
```

### Instalar en modo desarrollo

```bash
uv pip install -e .
```

---

## 🎮 Uso

### 1. Simulación Offline (secuencias Arduino)

Reproduce las secuencias de movimiento del código Arduino en la simulación 3D:

```bash
# Secuencia de caminata paso1
uv run robot-biped simulate-offline

# Cambiar velocidad
uv run robot-biped simulate-offline --speed 3.0
```

### 2. Simulación con Cinemática Inversa

Genera caminata autónoma usando el modelo DH y cinemática inversa:

```bash
# Caminata básica
uv run robot-biped simulate-ik

# Personalizar parámetros
uv run robot-biped simulate-ik --step-length 4 --step-height 2 --points 40
```

### 3. Visualización Estática

Muestra el robot en una posición fija:

```bash
# Posición home
uv run robot-biped static --home

# Ángulos personalizados (z1,z2,z3,z4,z5,z6)
uv run robot-biped static --custom "80,100,110,70,80,100"
```

### 4. Conexión con Arduino (Tiempo Real)

Controla el robot físico y visualiza en 3D simultáneamente:

```bash
# Auto-detectar puerto
uv run robot-biped arduino-live

# Especificar puerto
uv run robot-biped arduino-live --port /dev/ttyUSB0
```

> **Nota:** Requiere cargar el firmware en el Arduino (ver sección Firmware)

### 5. Verificar Matrices DH

```bash
uv run robot-biped test-dh
```

### 6. Mostrar Firmware Arduino

```bash
uv run robot-biped firmware
```

---

## 📁 Estructura del Proyecto

```
robot_bipedal_virtual/
├── pyproject.toml              # Configuración UV
├── README.md                   # Este archivo
├── robot_biped/
│   ├── __init__.py
│   ├── dh_kinematics.py        # Cinemática directa DH
│   ├── inverse_kinematics.py   # Cinemática inversa
│   ├── arduino_sequences.py    # Secuencias del Arduino
│   ├── visualizer.py           # Visualización 3D matplotlib
│   ├── serial_interface.py     # Comunicación serial
│   └── main.py                 # Punto de entrada CLI
```

---

## 🔧 Firmware Arduino

Para usar el modo en tiempo real, sube este código al Arduino Nano:

```cpp
#include <Servo.h>

Servo servos[6];
const uint8_t PINS[6] = {7, 6, 5, 4, 3, 2};
int currentAngles[6] = {90, 90, 90, 80, 90, 90};

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < 6; i++) {
    servos[i].attach(PINS[i]);
    servos[i].write(currentAngles[i]);
  }
  Serial.println("READY");
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    if (cmd.startsWith("A,")) {
      // Formato: A,z1,z2,z3,z4,z5,z6
      int idx = 2;
      for (int i = 0; i < 6; i++) {
        int next = cmd.indexOf(',', idx);
        String val = (next == -1) ? cmd.substring(idx) : cmd.substring(idx, next);
        currentAngles[i] = constrain(val.toInt(), 0, 180);
        servos[i].write(currentAngles[i]);
        idx = next + 1;
      }
      Serial.println("OK");
    }
  }
}
```

---

## 🧪 API Python

### Cinemática Directa (DH)

```python
from robot_biped.dh_kinematics import BipedRobot

robot = BipedRobot()

# Calcular posición del pie con ángulos
kin = robot.forward_kinematics_both(
    angles_left=[90, 90, 90],   # z1, z2, z3
    angles_right=[80, 90, 90]   # z4, z5, z6
)

print(kin['left'])   # Posiciones [cadera, rodilla, tobillo, pie]
print(kin['right'])  # Posiciones [cadera, rodilla, tobillo, pie]
```

### Cinemática Inversa

```python
from robot_biped.inverse_kinematics import InverseKinematics

ik = InverseKinematics()

# Calcular ángulos para posición deseada del pie (X, Y, Z) en cm
angles = ik.solve(
    target_x=3.0,    # adelante
    target_y=0.0,    # lateral
    target_z=-11.5,  # altura (suelo)
    leg="left"
)

if angles:
    theta1, theta2, theta3 = angles
    print(f"Cadera: {theta1:.1f}°, Rodilla: {theta2:.1f}°, Tobillo: {theta3:.1f}°")
```

### Visualización 3D

```python
from robot_biped.visualizer import RobotVisualizer

viz = RobotVisualizer()

# Vista estática
viz.show_static(angles_left=[90, 90, 90], angles_right=[80, 90, 90])

# Animación de secuencia
from robot_biped.arduino_sequences import WALK_STEP1
viz.animate_sequence(WALK_STEP1, interval_ms=100)

# Animación con cinemática inversa
viz.animate_ik_walk(step_length=3.0, step_height=1.5)
```

---

## 🎯 Controles durante la animación

| Tecla | Acción |
|-------|--------|
| `ESPACIO` | Pausar / Reanudar |
| `R` | Resetear vista |
| `Q` / `ESC` | Salir |

---

## 📚 Referencias

- Denavit, J. & Hartenberg, R.S. (1955). A kinematic notation for lower-pair mechanisms.
- Ortega, J.A. & Gomez, M.E. (2023). Inverse kinematics algorithms for low-cost bipedal robots.
- Ponce, R. et al. (2022). Solución a la cinemática directa e inversa de manipuladores robóticos.

---

## 📄 Licencia

MIT License - 2026
