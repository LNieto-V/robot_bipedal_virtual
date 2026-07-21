/*
 * ═══════════════════════════════════════════════════════════════
 *  ROBOT BÍPEDO 12GDL - Firmware Arduino Nano
 * ═══════════════════════════════════════════════════════════════
 *
 *  Controla 6 servomotores SG90 para un robot bípedo de 12GDL
 *  (6 articulaciones activas por cinemática DH).
 *
 *  Modos de operación:
 *    1. Control serial desde PC (protocolo texto)
 *    2. Caminata autónoma con secuencia IK precalculada
 *
 *  Mapeo de servos:
 *    z1 = Pin D7 - Tobillo Izquierdo  (índice 0)
 *    z2 = Pin D6 - Rodilla Izquierda  (índice 1)
 *    z3 = Pin D5 - Cadera Izquierda   (índice 2)
 *    z4 = Pin D4 - Tobillo Derecho    (índice 3)
 *    z5 = Pin D3 - Rodilla Derecha    (índice 4)
 *    z6 = Pin D2 - Cadera Derecha     (índice 5)
 *
 *  Protocolo serial (115200 baud):
 *    PC → Arduino:
 *      "PING\n"                      → Responde "PONG"
 *      "R\n"                         → Reporta ángulos actuales
 *      "A,z1,z2,z3,z4,z5,z6\n"      → Mueve todos los servos (instantáneo)
 *      "S,z1,z2,z3,z4,z5,z6,ms\n"   → Mueve servos con interpolación (ms)
 *      "M,servo,angle,delay\n"       → Mueve un solo servo
 *      "W\n"                         → Inicia caminata autónoma IK
 *      "H\n"                         → Volver a posición HOME
 *      "X\n"                         → Detener caminata
 *
 *  Autor: Félix David Henríquez Córdoba
 *  Universidad Cooperativa de Colombia - 2026
 */

#include <Servo.h>

// ═══════════════════════════════════════════════════════════════
// CONFIGURACIÓN DE HARDWARE
// ═══════════════════════════════════════════════════════════════

#define NUM_SERVOS 6

Servo servos[NUM_SERVOS];
const uint8_t SERVO_PINS[NUM_SERVOS] = {7, 8, 9, 10, 11, 12};

// Ángulos actuales y objetivo
int currentAngles[NUM_SERVOS] = {90, 90, 90, 80, 90, 90};  // HOME
int targetAngles[NUM_SERVOS]  = {90, 90, 90, 80, 90, 90};

// Posición HOME
const int HOME_ANGLES[NUM_SERVOS] = {90, 90, 90, 80, 90, 90};

// Límites articulares (servos SG90)
#define SERVO_MIN 0
#define SERVO_MAX 180

// ═══════════════════════════════════════════════════════════════
// SECUENCIA DE CAMINATA IK (precalculada con cinemática inversa)
// ═══════════════════════════════════════════════════════════════
// Generada con: python -m robot_biped.main simulate-ik --step-length 4 --step-height 2 --points 40
// Parámetros: step_length=3.0cm, step_height=1.5cm, n_points=40, lateral_shift=1.0cm
// Formato: {z1, z2, z3, z4, z5, z6} por cada frame
// Inclinación perpendicular incluida, rodilla flexiona hacia atrás (natural)

#define WALK_FRAMES 120
#define WALK_DELAY_MS 80   // Intervalo entre frames (ajustar para velocidad)

const uint8_t WALK_IK[WALK_FRAMES][NUM_SERVOS] PROGMEM = {
  {  90,  56, 103,  70, 119,  79 },  // Frame 0
  {  90,  53, 105,  69, 119,  79 },  // Frame 1
  {  91,  50, 107,  69, 118,  79 },  // Frame 2
  {  91,  46, 109,  68, 118,  79 },  // Frame 3
  {  92,  44, 111,  67, 117,  79 },  // Frame 4
  {  92,  41, 112,  67, 117,  79 },  // Frame 5
  {  92,  38, 114,  66, 117,  79 },  // Frame 6
  {  93,  36, 116,  65, 116,  79 },  // Frame 7
  {  93,  34, 117,  65, 116,  79 },  // Frame 8
  {  93,  32, 119,  64, 115,  79 },  // Frame 9
  {  94,  30, 121,  64, 115,  79 },  // Frame 10
  {  94,  28, 122,  63, 115,  79 },  // Frame 11
  {  94,  27, 124,  63, 114,  79 },  // Frame 12
  {  94,  25, 125,  62, 114,  79 },  // Frame 13
  {  95,  24, 126,  62, 114,  79 },  // Frame 14
  {  95,  23, 127,  62, 113,  79 },  // Frame 15
  {  95,  22, 129,  61, 113,  78 },  // Frame 16
  {  95,  21, 130,  61, 113,  78 },  // Frame 17
  {  95,  20, 131,  61, 113,  78 },  // Frame 18
  {  96,  19, 132,  61, 113,  78 },  // Frame 19
  {  96,  19, 133,  61, 112,  77 },  // Frame 20
  {  96,  19, 134,  61, 112,  77 },  // Frame 21
  {  96,  19, 134,  61, 112,  77 },  // Frame 22
  {  96,  19, 135,  61, 112,  76 },  // Frame 23
  {  96,  19, 136,  61, 112,  76 },  // Frame 24
  {  96,  19, 136,  61, 112,  76 },  // Frame 25
  {  96,  19, 136,  61, 113,  75 },  // Frame 26
  {  95,  20, 136,  61, 113,  75 },  // Frame 27
  {  95,  21, 136,  61, 113,  74 },  // Frame 28
  {  95,  22, 136,  61, 113,  74 },  // Frame 29
  {  95,  23, 136,  62, 113,  73 },  // Frame 30
  {  95,  24, 136,  62, 114,  73 },  // Frame 31
  {  94,  25, 136,  62, 114,  72 },  // Frame 32
  {  94,  27, 135,  63, 114,  72 },  // Frame 33
  {  94,  28, 134,  63, 115,  71 },  // Frame 34
  {  94,  30, 134,  64, 115,  71 },  // Frame 35
  {  93,  32, 133,  64, 115,  70 },  // Frame 36
  {  93,  34, 132,  65, 116,  70 },  // Frame 37
  {  93,  36, 131,  65, 116,  69 },  // Frame 38
  {  92,  38, 129,  66, 117,  69 },  // Frame 39
  {  92,  41, 128,  67, 117,  68 },  // Frame 40
  {  92,  44, 126,  67, 117,  68 },  // Frame 41
  {  91,  46, 125,  68, 118,  67 },  // Frame 42
  {  91,  50, 123,  69, 118,  67 },  // Frame 43
  {  90,  53, 121,  69, 119,  66 },  // Frame 44
  {  90,  56, 118,  70, 119,  66 },  // Frame 45
  {  90,  56, 118,  71, 120,  65 },  // Frame 46
  {  90,  56, 118,  72, 120,  65 },  // Frame 47
  {  90,  56, 118,  72, 120,  64 },  // Frame 48
  {  91,  57, 118,  73, 121,  64 },  // Frame 49
  {  92,  57, 118,  74, 121,  64 },  // Frame 50
  {  93,  57, 118,  75, 122,  63 },  // Frame 51
  {  93,  58, 117,  76, 122,  63 },  // Frame 52
  {  94,  58, 117,  77, 122,  63 },  // Frame 53
  {  95,  58, 117,  77, 123,  62 },  // Frame 54
  {  96,  59, 116,  78, 123,  62 },  // Frame 55
  {  97,  59, 116,  79, 123,  62 },  // Frame 56
  {  98,  60, 116,  80, 124,  62 },  // Frame 57
  {  98,  60, 115,  80, 124,  62 },  // Frame 58
  {  99,  60, 115,  80, 124,  62 },  // Frame 59
  { 100,  61, 114,  80, 124,  62 },  // Frame 60
  { 101,  61, 114,  80, 127,  59 },  // Frame 61
  { 101,  62, 113,  79, 130,  57 },  // Frame 62
  { 102,  62, 113,  79, 134,  55 },  // Frame 63
  { 103,  63, 112,  78, 136,  54 },  // Frame 64
  { 103,  63, 112,  78, 139,  52 },  // Frame 65
  { 104,  63, 111,  78, 142,  51 },  // Frame 66
  { 105,  64, 111,  77, 144,  49 },  // Frame 67
  { 105,  64, 110,  77, 146,  48 },  // Frame 68
  { 106,  65, 110,  77, 148,  47 },  // Frame 69
  { 106,  65, 109,  76, 150,  46 },  // Frame 70
  { 107,  65, 109,  76, 152,  46 },  // Frame 71
  { 107,  66, 108,  76, 153,  45 },  // Frame 72
  { 108,  66, 108,  76, 155,  44 },  // Frame 73
  { 108,  66, 107,  75, 156,  44 },  // Frame 74
  { 108,  67, 107,  75, 157,  44 },  // Frame 75
  { 109,  67, 106,  75, 158,  44 },  // Frame 76
  { 109,  67, 106,  75, 159,  44 },  // Frame 77
  { 109,  67, 105,  75, 160,  44 },  // Frame 78
  { 109,  67, 105,  74, 161,  44 },  // Frame 79
  { 109,  68, 104,  74, 161,  44 },  // Frame 80
  { 109,  68, 104,  74, 161,  44 },  // Frame 81
  { 109,  68, 104,  74, 161,  45 },  // Frame 82
  { 109,  68, 103,  74, 161,  46 },  // Frame 83
  { 109,  68, 103,  74, 161,  46 },  // Frame 84
  { 109,  68, 103,  74, 161,  47 },  // Frame 85
  { 109,  67, 102,  74, 161,  48 },  // Frame 86
  { 109,  67, 102,  75, 160,  49 },  // Frame 87
  { 109,  67, 102,  75, 159,  50 },  // Frame 88
  { 109,  67, 102,  75, 158,  51 },  // Frame 89
  { 108,  67, 101,  75, 157,  53 },  // Frame 90
  { 108,  66, 101,  75, 156,  54 },  // Frame 91
  { 108,  66, 101,  76, 155,  55 },  // Frame 92
  { 107,  66, 101,  76, 153,  56 },  // Frame 93
  { 107,  65, 101,  76, 152,  58 },  // Frame 94
  { 106,  65, 101,  76, 150,  59 },  // Frame 95
  { 106,  65, 101,  77, 148,  61 },  // Frame 96
  { 105,  64, 101,  77, 146,  63 },  // Frame 97
  { 105,  64, 101,  77, 144,  64 },  // Frame 98
  { 104,  63, 101,  78, 142,  66 },  // Frame 99
  { 103,  63, 101,  78, 139,  68 },  // Frame 100
  { 103,  63, 101,  78, 136,  69 },  // Frame 101
  { 102,  62, 101,  79, 134,  71 },  // Frame 102
  { 101,  62, 101,  79, 130,  73 },  // Frame 103
  { 101,  61, 101,  80, 127,  75 },  // Frame 104
  { 100,  61, 101,  80, 124,  77 },  // Frame 105
  {  99,  60, 101,  80, 124,  77 },  // Frame 106
  {  98,  60, 101,  80, 124,  77 },  // Frame 107
  {  98,  60, 101,  80, 124,  77 },  // Frame 108
  {  97,  59, 102,  79, 123,  77 },  // Frame 109
  {  96,  59, 102,  78, 123,  78 },  // Frame 110
  {  95,  58, 102,  77, 123,  78 },  // Frame 111
  {  94,  58, 102,  77, 122,  78 },  // Frame 112
  {  93,  58, 102,  76, 122,  78 },  // Frame 113
  {  93,  57, 102,  75, 122,  78 },  // Frame 114
  {  92,  57, 102,  74, 121,  78 },  // Frame 115
  {  91,  57, 103,  73, 121,  78 },  // Frame 116
  {  90,  56, 103,  72, 120,  79 },  // Frame 117
  {  90,  56, 103,  72, 120,  79 },  // Frame 118
  {  90,  56, 103,  71, 120,  79 }  // Frame 119
};

// ═══════════════════════════════════════════════════════════════
// ESTADO DEL SISTEMA
// ═══════════════════════════════════════════════════════════════

bool walkActive = false;       // Caminata autónoma activa
uint8_t walkFrame = 0;         // Frame actual de la caminata
String inputBuffer = "";       // Buffer de lectura serial

// ═══════════════════════════════════════════════════════════════
// FUNCIONES DE SERVOS
// ═══════════════════════════════════════════════════════════════

/**
 * Mueve todos los servos instantáneamente a los ángulos objetivo.
 */
void moveServosImmediate(int angles[NUM_SERVOS]) {
  for (int i = 0; i < NUM_SERVOS; i++) {
    int angle = constrain(angles[i], SERVO_MIN, SERVO_MAX);
    currentAngles[i] = angle;
    servos[i].write(angle);
  }
}

/**
 * Mueve todos los servos suavemente a los ángulos objetivo en 'durationMs' ms.
 * Usa interpolación lineal con pasos de ~10ms para movimiento fluido.
 */
void moveServosSmooth(int target[NUM_SERVOS], unsigned int durationMs) {
  if (durationMs < 10) {
    moveServosImmediate(target);
    return;
  }

  const int steps = durationMs / 10;  // Un paso cada ~10ms
  int startAngles[NUM_SERVOS];

  // Guardar posición inicial
  for (int i = 0; i < NUM_SERVOS; i++) {
    startAngles[i] = currentAngles[i];
  }

  // Interpolar
  for (int step = 1; step <= steps; step++) {
    float t = (float)step / (float)steps;

    for (int i = 0; i < NUM_SERVOS; i++) {
      int angle = startAngles[i] + (int)((target[i] - startAngles[i]) * t);
      angle = constrain(angle, SERVO_MIN, SERVO_MAX);
      currentAngles[i] = angle;
      servos[i].write(angle);
    }

    delay(10);

    // Verificar si hay comando de parada durante la interpolación
    if (Serial.available() > 0) {
      char peek = Serial.peek();
      if (peek == 'X' || peek == 'H') {
        return;  // Interrumpir movimiento
      }
    }
  }
}

/**
 * Ejecuta un frame de la caminata IK desde PROGMEM.
 */
void executeWalkFrame(uint8_t frame) {
  int target[NUM_SERVOS];

  for (int i = 0; i < NUM_SERVOS; i++) {
    target[i] = pgm_read_byte(&WALK_IK[frame][i]);
  }

  moveServosSmooth(target, WALK_DELAY_MS);
}

/**
 * Vuelve a la posición HOME suavemente.
 */
void goHome() {
  int home[NUM_SERVOS];
  for (int i = 0; i < NUM_SERVOS; i++) {
    home[i] = HOME_ANGLES[i];
  }
  moveServosSmooth(home, 500);
}

// ═══════════════════════════════════════════════════════════════
// PROCESAMIENTO DE COMANDOS SERIAL
// ═══════════════════════════════════════════════════════════════

void processCommand(String cmd) {
  cmd.trim();

  // --- PING: verificar conexión ---
  if (cmd == "PING") {
    Serial.println("PONG");
    return;
  }

  // --- R: reportar ángulos actuales ---
  if (cmd == "R") {
    Serial.print(currentAngles[0]);
    for (int i = 1; i < NUM_SERVOS; i++) {
      Serial.print(",");
      Serial.print(currentAngles[i]);
    }
    Serial.println();
    return;
  }

  // --- H: ir a HOME ---
  if (cmd == "H") {
    walkActive = false;
    goHome();
    Serial.println("HOME");
    return;
  }

  // --- W: iniciar caminata IK ---
  if (cmd == "W") {
    walkActive = true;
    walkFrame = 0;
    Serial.println("WALK_START");
    return;
  }

  // --- X: detener caminata ---
  if (cmd == "X") {
    walkActive = false;
    Serial.println("WALK_STOP");
    return;
  }

  // --- A,z1,z2,z3,z4,z5,z6: mover todos los servos (instantáneo) ---
  if (cmd.startsWith("A,")) {
    int angles[NUM_SERVOS];
    int idx = 2;

    for (int i = 0; i < NUM_SERVOS; i++) {
      int nextComma = cmd.indexOf(',', idx);
      String val = (nextComma == -1) ? cmd.substring(idx) : cmd.substring(idx, nextComma);
      angles[i] = constrain(val.toInt(), SERVO_MIN, SERVO_MAX);
      idx = nextComma + 1;
    }

    moveServosImmediate(angles);
    Serial.println("OK");
    return;
  }

  // --- S,z1,z2,z3,z4,z5,z6,ms: mover con interpolación ---
  if (cmd.startsWith("S,")) {
    int angles[NUM_SERVOS];
    int idx = 2;

    for (int i = 0; i < NUM_SERVOS; i++) {
      int nextComma = cmd.indexOf(',', idx);
      String val = (nextComma == -1) ? cmd.substring(idx) : cmd.substring(idx, nextComma);
      angles[i] = constrain(val.toInt(), SERVO_MIN, SERVO_MAX);
      idx = nextComma + 1;
    }

    // Último valor: duración en ms
    int nextComma = cmd.indexOf(',', idx);
    String msVal = (nextComma == -1) ? cmd.substring(idx) : cmd.substring(idx, nextComma);
    unsigned int durationMs = msVal.toInt();

    moveServosSmooth(angles, durationMs);
    Serial.println("OK");
    return;
  }

  // --- M,servo,angle,delay: mover un solo servo ---
  if (cmd.startsWith("M,")) {
    int comma1 = cmd.indexOf(',', 2);
    int comma2 = cmd.indexOf(',', comma1 + 1);

    int servoIdx = cmd.substring(2, comma1).toInt();
    int angle = cmd.substring(comma1 + 1, comma2).toInt();
    int delayMs = cmd.substring(comma2 + 1).toInt();

    if (servoIdx >= 0 && servoIdx < NUM_SERVOS) {
      angle = constrain(angle, SERVO_MIN, SERVO_MAX);
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

// ═══════════════════════════════════════════════════════════════
// SETUP Y LOOP PRINCIPAL
// ═══════════════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);

  // Inicializar servos en posición HOME
  for (int i = 0; i < NUM_SERVOS; i++) {
    servos[i].attach(SERVO_PINS[i]);
    servos[i].write(currentAngles[i]);
  }

  delay(500);  // Esperar a que los servos se posicionen
  Serial.println("READY");
}

void loop() {
  // --- Procesar comandos seriales ---
  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '\n') {
      processCommand(inputBuffer);
      inputBuffer = "";
    } else if (c != '\r') {
      inputBuffer += c;
    }
  }

  // --- Caminata autónoma IK ---
  if (walkActive) {
    executeWalkFrame(walkFrame);
    walkFrame = (walkFrame + 1) % WALK_FRAMES;

    // Reportar frame actual
    Serial.print("F,");
    Serial.println(walkFrame);
  }
}
