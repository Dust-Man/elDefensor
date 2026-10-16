#include "ultrasonicos.h"

// ---------------- Ultrasonicos HC-SR04 ----------------
// Se disparan de uno en uno para que el eco de uno no lo escuche otro.
// pulseIn bloquea como maximo ~US_DIST_MAX_CM*58 us por sensor (~12 ms con 200 cm).
const char* const NOMBRE_US[NUM_ULTRASONICOS] = { "frente", "izq", "der" };

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

float ultrasonicoLeerCm(uint8_t i, float maxCm) {
  if (!ultrasonicoAsignado(i)) return US_NO_ASIGNADO;
  unsigned long timeout = (unsigned long)(maxCm * 58.0f) + 1000;   // +1 ms: el HC-SR04 tarda en empezar el eco
  digitalWrite(PIN_US_TRIG[i], LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_US_TRIG[i], HIGH);   // pulso de disparo de 10 us
  delayMicroseconds(10);
  digitalWrite(PIN_US_TRIG[i], LOW);
  unsigned long t = pulseIn(PIN_US_ECHO[i], HIGH, timeout);
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

// ---------------- Laterales mientras avanza ----------------
// Un sensor por llamada alternando izq/der: con el lazo de 20 ms cada lado se lee
// a ~25 Hz y el eco de uno no lo escucha el otro. Timeout corto (US_LATERAL_MAX_CM)
// para no frenar el lazo. Sin eco aqui NO significa "no hay pared", solo que la
// lectura no sirve para centrar (el mapeo sigue siendo con el robot parado).
struct Lateral {
  float cm;        // ultima lectura aceptada
  float previa;    // ultima lectura en rango (para detectar saltos), o US_SIN_ECO
  bool ok;         // la ultima lectura fue aceptada
  uint32_t t;      // millis de la ultima lectura aceptada
};
Lateral lateral[2];   // 0 = izq, 1 = der
uint8_t lateralSiguiente = 0;
// cm que se suman al objetivo y al maximo de cada lado (0 = celda del laberinto)
float desfaseLateral[2] = { 0, 0 };

void ultrasonicosLateralDesfase(float izqCm, float derCm) {
  desfaseLateral[0] = izqCm;
  desfaseLateral[1] = derCm;
}

void ultrasonicosLateralReiniciar() {
  for (uint8_t k = 0; k < 2; k++) {
    lateral[k].previa = US_SIN_ECO;
    lateral[k].ok = false;
  }
}

void ultrasonicosLateralActualizar() {
  uint8_t k = lateralSiguiente;
  lateralSiguiente = 1 - k;
  Lateral& L = lateral[k];
  float maxCm = US_LATERAL_MAX_CM + desfaseLateral[k];
  float d = ultrasonicoLeerCm(k == 0 ? US_IZQ : US_DER, maxCm);
  bool enRango = d > US_LATERAL_MIN_CM && d < maxCm;
  // Se acepta con dos lecturas seguidas parecidas: un filo o un hueco da un salto
  bool estable = enRango && L.previa > 0 && fabs(d - L.previa) <= US_LATERAL_SALTO_CM;
  L.previa = enRango ? d : US_SIN_ECO;
  L.ok = estable;
  if (estable) { L.cm = d; L.t = millis(); }
}

static bool lateralVigente(uint8_t k) {
  return lateral[k].ok && millis() - lateral[k].t < US_LATERAL_VIGENCIA_MS;
}

void ultrasonicosLateral(float& izq, float& der) {
  izq = lateralVigente(0) ? lateral[0].cm : US_SIN_ECO;
  der = lateralVigente(1) ? lateral[1].cm : US_SIN_ECO;
}

bool ultrasonicosErrorLateral(float& eCm, bool& ambas) {
  bool izq = lateralVigente(0);
  bool der = lateralVigente(1);
  ambas = izq && der;
  float objIzq = US_LATERAL_OBJ_IZQ_CM + desfaseLateral[0];
  float objDer = US_LATERAL_OBJ_DER_CM + desfaseLateral[1];
  if (ambas)    eCm = (lateral[0].cm - lateral[1].cm) / 2 - (objIzq - objDer) / 2;
  else if (izq) eCm = lateral[0].cm - objIzq;
  else if (der) eCm = objDer - lateral[1].cm;
  return izq || der;
}
