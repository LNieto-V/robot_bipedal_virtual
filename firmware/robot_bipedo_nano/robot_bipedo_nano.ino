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
const uint8_t SERVO_PINS[NUM_SERVOS] = {7, 6, 5, 4, 3, 2};

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
// Generada con: python -m robot_biped.main simulate-ik
// Parámetros: step_length=3.0cm, step_height=1.5cm, n_points=20
// Formato: {z1, z2, z3, z4, z5, z6} por cada frame
// La restricción biomecánica garantiza θ2(rodilla) ≥ 90° (flexión adelante)

#define WALK_FRAMES 20
#define WALK_DELAY_MS 80   // Intervalo entre frames (ajustar para velocidad)

const uint8_t WALK_IK[WALK_FRAMES][NUM_SERVOS] PROGMEM = {
  { 90, 124,  62,  80, 121,  78},  // Frame  0 - Inicio swing izq
  { 92, 143,  50,  80, 120,  77},  // Frame  1
  { 94, 155,  44,  80, 120,  75},  // Frame  2
  { 96, 161,  44,  80, 120,  73},  // Frame  3 - Pico swing izq
  { 96, 161,  48,  80, 120,  71},  // Frame  4
  { 94, 155,  55,  80, 120,  68},  // Frame  5
  { 92, 143,  65,  80, 120,  66},  // Frame  6
  { 90, 124,  77,  80, 121,  65},  // Frame  7 - Contacto izq
  { 90, 124,  77,  80, 122,  63},  // Frame  8 - Inicio stance izq
  { 90, 122,  78,  80, 124,  62},  // Frame  9
  { 90, 121,  78,  80, 124,  62},  // Frame 10 - Inicio swing der
  { 90, 120,  77,  78, 143,  50},  // Frame 11
  { 90, 120,  75,  76, 155,  44},  // Frame 12
  { 90, 120,  73,  74, 161,  44},  // Frame 13 - Pico swing der
  { 90, 120,  71,  74, 161,  48},  // Frame 14
  { 90, 120,  68,  76, 155,  55},  // Frame 15
  { 90, 120,  66,  78, 143,  65},  // Frame 16
  { 90, 121,  65,  80, 124,  77},  // Frame 17 - Contacto der
  { 90, 122,  63,  80, 124,  77},  // Frame 18
  { 90, 124,  62,  80, 122,  78}   // Frame 19 - Ciclo completo
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
