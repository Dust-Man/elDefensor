#include "ultrasonicos.h"

// ---------------- Ultrasonicos HC-SR04 ----------------
// Se disparan de uno en uno para que el eco de uno no lo escuche otro.
// pulseIn bloquea como maximo ~US_DIST_MAX_CM*58 us por sensor (~12 ms con 200 cm).
const char* const NOMBRE_US[NUM_ULTRASONICOS] = { "frente", "izq", "der" };
const unsigned long US_TIMEOUT_US = (unsigned long)(US_DIST_MAX_CM * 58.0f) + 1000;   // +1 ms: el HC-SR04 tarda en empezar el eco

bool ultrasonicoAsignado(uint8_t i) {
  return PIN_US_TRIG[i] != US_SIN_PIN && PIN_US_ECHO[i] != US_SIN_PIN;
}

void ultrasonicosIniciar() {
  for (uint8_t i = 0; i < NUM_ULTRASONICOS; i++) {
    if (!ultrasonicoAsignado(i)) continue;
    pinMode(PIN_US_TRIG[i], OUTPUT);
    digitalWrite(PIN_US_TRIG[i], LOW);
    pinMode(PIN_US_ECHO[i], INPUT);
  }
}

float ultrasonicoLeerCm(uint8_t i) {
  if (!ultrasonicoAsignado(i)) return US_NO_ASIGNADO;
  digitalWrite(PIN_US_TRIG[i], LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_US_TRIG[i], HIGH);   // pulso de disparo de 10 us
  delayMicroseconds(10);
  digitalWrite(PIN_US_TRIG[i], LOW);
  unsigned long t = pulseIn(PIN_US_ECHO[i], HIGH, US_TIMEOUT_US);
  if (t == 0) return US_SIN_ECO;
  return t / 58.0f;   // ida y vuelta a ~343 m/s
}

// Una lectura suelta puede salir sin eco o rebotada; la mediana descarta una mala de cada tres
float ultrasonicoMedianaCm(uint8_t i, uint8_t n) {
  if (!ultrasonicoAsignado(i)) return US_NO_ASIGNADO;
  float d[5];
  n = constrain(n, 1, 5);
  for (uint8_t k = 0; k < n; k++) {
    if (k > 0) delay(US_PAUSA_MS);   // que se apague el eco anterior
    d[k] = ultrasonicoLeerCm(i);
  }
  // insercion: n es chico
  for (uint8_t a = 1; a < n; a++) {
    float v = d[a];
    int8_t b = a - 1;
    while (b >= 0 && d[b] > v) { d[b + 1] = d[b]; b--; }
    d[b + 1] = v;
  }
  return d[n / 2];
}

void ultrasonicosImprimir(const float d[NUM_ULTRASONICOS]) {
  Serial.print(F("US"));
  for (uint8_t i = 0; i < NUM_ULTRASONICOS; i++) {
    Serial.print(F("  ")); Serial.print(NOMBRE_US[i]); Serial.print('=');
    if (d[i] == US_NO_ASIGNADO)  Serial.print(F("sin pin"));
    else if (d[i] == US_SIN_ECO) Serial.print(F("---"));
    else { Serial.print(d[i], 1); Serial.print(F("cm")); }
  }
  Serial.println();
}

// Una lectura de cada uno, con pausa entre sensores para que no se crucen los ecos
void ultrasonicosLeerTodos() {
  float d[NUM_ULTRASONICOS];
  for (uint8_t i = 0; i < NUM_ULTRASONICOS; i++) {
    d[i] = ultrasonicoLeerCm(i);
    if (ultrasonicoAsignado(i)) delay(US_PAUSA_MS);
  }
  ultrasonicosImprimir(d);
}
