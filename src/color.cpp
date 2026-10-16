#include <Wire.h>
#include <EEPROM.h>
#include "color.h"

// ---------------- Sensores de color TCS34725 (via multiplexor TCA9548A) ----------------
// Todos los TCS34725 tienen la direccion 0x29 fija: cada uno va en su propio canal
// del multiplexor y antes de hablarle se conecta su canal al bus.
// Se leen por registros (como el MPU) en vez de con la libreria de Adafruit:
// getRGB() hace un delay() del tiempo de integracion y frenaria el lazo de 20 ms.
// Cada sensor integra solo todo el tiempo (AEN); aqui solo se leen los datos.
bool tcaOk = false;
bool colorOk[NUM_SENSORES_COLOR];

// Conecta al bus solo el canal del sensor s (el MPU sigue en el bus principal)
bool tcaSeleccionar(uint8_t s) {
  Wire.beginTransmission(TCA_DIRECCION);
  Wire.write(1 << CANAL_COLOR[s]);
  return Wire.endTransmission() == 0;
}

bool tcsEscribir(uint8_t s, uint8_t reg, uint8_t val) {
  if (!tcaSeleccionar(s)) return false;
  Wire.beginTransmission(TCS_DIRECCION);
  Wire.write(0x80 | reg);   // bit de comando
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool tcsLeer(uint8_t s, uint8_t reg, uint8_t* buf, uint8_t n) {
  if (!tcaSeleccionar(s)) return false;
  Wire.beginTransmission(TCS_DIRECCION);
  Wire.write(0xA0 | reg);   // comando + auto-incremento
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(TCS_DIRECCION, n) != n) return false;
  for (uint8_t k = 0; k < n; k++) buf[k] = Wire.read();
  return true;
}

bool colorIniciar(uint8_t s) {
  uint8_t id;
  if (!tcsLeer(s, 0x12, &id, 1)) return false;
  if (id != 0x44 && id != 0x10 && id != 0x4D) {
    Serial.print(F("!! S")); Serial.print(s);
    Serial.print(F(" ID inesperado: 0x")); Serial.println(id, HEX);
    return false;
  }
  tcsEscribir(s, 0x01, TCS_ATIME);      // ATIME
  tcsEscribir(s, 0x0F, TCS_GANANCIA);   // CONTROL
  tcsEscribir(s, 0x00, 0x01);           // ENABLE: PON
  delay(3);
  return tcsEscribir(s, 0x00, 0x03);    // ENABLE: PON + AEN
}

// Lista todo lo que contesta en el bus principal (MPU 0x68, multiplexor 0x70-0x77,
// y lo que este en el canal seleccionado del multiplexor)
void escanearI2C() {
  Serial.print(F("I2C:"));
  uint8_t encontrados = 0;
  for (uint8_t dir = 1; dir < 127; dir++) {
    Wire.beginTransmission(dir);
    if (Wire.endTransmission() != 0) continue;
    Serial.print(F(" 0x")); Serial.print(dir, HEX);
    if (dir == MPU_DIRECCION)        Serial.print(F("(MPU)"));
    else if (dir == TCA_DIRECCION)   Serial.print(F("(multiplexor)"));
    else if (dir >= 0x71 && dir <= 0x77) Serial.print(F("(multiplexor? A0-A2 no estan en GND)"));
    else if (dir == TCS_DIRECCION)   Serial.print(F("(TCS34725)"));
    encontrados++;
  }
  if (encontrados == 0) Serial.print(F(" nada (revisar SDA/SCL, GND comun)"));
  Serial.println();
}

// Llamar despues de mpuIniciar() (ahi se hace Wire.begin)
void colorIniciarTodos() {
  Wire.beginTransmission(TCA_DIRECCION);
  tcaOk = (Wire.endTransmission() == 0);
  if (!tcaOk) {
    Serial.println(F("!! Multiplexor TCA9548A no responde en 0x70: A0-A2 y RST no deben quedar sueltos."));
    escanearI2C();
  }
  for (uint8_t s = 0; s < NUM_SENSORES_COLOR; s++) {
    colorOk[s] = tcaOk && colorIniciar(s);
    Serial.print(F("Sensor de color S")); Serial.print(s);
    Serial.print(F(" (canal ")); Serial.print(CANAL_COLOR[s]); Serial.print(F("): "));
    Serial.println(colorOk[s] ? F("OK") : F("NO RESPONDE"));
  }
}

// Ultimo dato crudo (clear, rojo, verde, azul). false si falla el bus o no hay dato valido.
bool colorLeerCrudo(uint8_t s, uint16_t& c, uint16_t& r, uint16_t& g, uint16_t& b) {
  uint8_t st, d[8];
  if (!tcsLeer(s, 0x13, &st, 1) || !(st & 0x01)) return false;   // STATUS.AVALID
  if (!tcsLeer(s, 0x14, d, 8)) return false;
  c = d[0] | (d[1] << 8);
  r = d[2] | (d[3] << 8);
  g = d[4] | (d[5] << 8);
  b = d[6] | (d[7] << 8);
  return true;
}

// ---- Calibracion y clasificacion (una por sensor) ----
// 1) Blanco/negro: cada canal se escala para que la hoja blanca de 1.0 y la negra 0.0.
//    (getRGB divide entre clear ~ R+G+B, por eso el blanco salia gris ~85.)
// 2) Cada color se aprende con el sensor sobre la muestra real y se guarda su
//    cromaticidad (r,g,b / suma): no depende del brillo pero si de la saturacion,
//    asi rosa y rojo quedan separados. Se clasifica por el aprendido mas cercano.
const char* const NOMBRE_COLOR[NUM_COLORES] = { "verde", "rojo", "rosa", "naranja", "amarillo", "cyan" };
const uint16_t COLOR_MAGIA = 0xC0A1;

struct CalColor {
  uint16_t magia;
  float blanco[3];               // crudo R,G,B sobre blanco (0 = sin calibrar)
  float negro[3];                // crudo R,G,B sobre negro
  float ref[NUM_COLORES][3];     // cromaticidad aprendida; ref[i][0] < 0 = sin aprender
};
CalColor cal[NUM_SENSORES_COLOR];

void colorCalDefecto(uint8_t s) {
  cal[s].magia = COLOR_MAGIA;
  for (uint8_t k = 0; k < 3; k++) { cal[s].blanco[k] = 0; cal[s].negro[k] = 0; }
  for (uint8_t i = 0; i < NUM_COLORES; i++) cal[s].ref[i][0] = -1;
}

// Cada sensor tiene su bloque en la EEPROM (S0 conserva la calibracion de antes)
void colorCalCargar() {
  for (uint8_t s = 0; s < NUM_SENSORES_COLOR; s++) {
    EEPROM.get(COLOR_EEPROM_DIR + s * sizeof(CalColor), cal[s]);
    if (cal[s].magia != COLOR_MAGIA) colorCalDefecto(s);
  }
}

void colorCalGuardar(uint8_t s) { EEPROM.put(COLOR_EEPROM_DIR + s * sizeof(CalColor), cal[s]); }

// Promedia varias integraciones (bloquea ~0.5 s: solo con el robot quieto)
bool colorPromedioCrudo(uint8_t s, float out[4]) {
  float suma[4] = { 0, 0, 0, 0 };
  uint8_t validas = 0;
  for (uint8_t m = 0; m < COLOR_MUESTRAS_CAL; m++) {
    delay(COLOR_INTEGRACION_MS);
    uint16_t c, r, g, b;
    if (!colorLeerCrudo(s, c, r, g, b)) continue;
    suma[0] += r; suma[1] += g; suma[2] += b; suma[3] += c;
    validas++;
  }
  if (validas == 0) return false;
  for (uint8_t k = 0; k < 4; k++) out[k] = suma[k] / validas;
  return true;
}

// Crudo -> 0..1 por canal usando blanco/negro. Sin calibrar: aproxima con clear/3.
void colorCorregir(uint8_t s, const float crudo[4], float out[3]) {
  for (uint8_t k = 0; k < 3; k++) {
    float escala = cal[s].blanco[k] - cal[s].negro[k];
    if (cal[s].blanco[k] <= 0 || escala < 1) out[k] = crudo[3] > 0 ? crudo[k] / (crudo[3] / 3.0f) : 0;
    else                                     out[k] = (crudo[k] - cal[s].negro[k]) / escala;
    if (out[k] < 0) out[k] = 0;
  }
}

bool cromaticidad(const float cor[3], float out[3]) {
  float suma = cor[0] + cor[1] + cor[2];
  if (suma < 1e-3f) return false;
  for (uint8_t k = 0; k < 3; k++) out[k] = cor[k] / suma;
  return true;
}

// Devuelve el indice del color aprendido mas cercano (o -1) y su distancia
int8_t colorMasCercano(uint8_t s, const float cor[3], float& dist) {
  float cr[3];
  dist = 1e9f;
  if (!cromaticidad(cor, cr)) return -1;
  int8_t mejor = -1;
  for (uint8_t i = 0; i < NUM_COLORES; i++) {
    if (cal[s].ref[i][0] < 0) continue;
    float d = 0;
    for (uint8_t k = 0; k < 3; k++) d += sq(cr[k] - cal[s].ref[i][k]);
    d = sqrt(d);
    if (d < dist) { dist = d; mejor = i; }
  }
  return mejor;
}

// Color clasificado (indice en NOMBRE_COLOR) o -1 si es blanco/negro/desconocido
int8_t colorClasificar(uint8_t s, const float cor[3], float& dist) {
  if (max(cor[0], max(cor[1], cor[2])) < COLOR_BRILLO_MIN) { dist = 1e9f; return -1; }
  int8_t i = colorMasCercano(s, cor, dist);
  return (dist <= COLOR_DIST_MAX) ? i : -1;
}

bool colorDetectar(uint8_t s, int8_t& color, bool* blanco) {
  if (!colorOk[s]) return false;
  uint16_t c, r, g, b;
  if (!colorLeerCrudo(s, c, r, g, b)) return false;
  float crudo[4] = { (float)r, (float)g, (float)b, (float)c };
  float cor[3], dist;
  colorCorregir(s, crudo, cor);
  color = colorClasificar(s, cor, dist);
  // Blanco: los tres canales cerca de la referencia de blanco (cb)
  if (blanco) *blanco = min(cor[0], min(cor[1], cor[2])) > COLOR_BLANCO_MIN;
  return true;
}

// Tono (0-360) y saturacion (0-1), solo para depurar
void colorHSV(const float cor[3], float& h, float& sat) {
  float mx = max(cor[0], max(cor[1], cor[2]));
  float mn = min(cor[0], min(cor[1], cor[2]));
  float d = mx - mn;
  sat = mx > 0 ? d / mx : 0;
  if (d < 1e-4f)          h = 0;
  else if (mx == cor[0])  h = 60 * fmod((cor[1] - cor[2]) / d, 6.0f);
  else if (mx == cor[1])  h = 60 * ((cor[2] - cor[0]) / d + 2);
  else                    h = 60 * ((cor[0] - cor[1]) / d + 4);
  if (h < 0) h += 360;
}

// Imprime RGB corregido 0-255, HSV, color detectado y los valores crudos del sensor s
void colorImprimir(uint8_t s) {
  Serial.print('S'); Serial.print(s); Serial.print(F("  "));
  if (!colorOk[s]) { Serial.println(F("!! no inicializado (ci para reintentar)")); return; }
  uint16_t c, r, g, b;
  if (!colorLeerCrudo(s, c, r, g, b)) { Serial.println(F("!! sin dato valido / error I2C")); return; }
  float crudo[4] = { (float)r, (float)g, (float)b, (float)c };
  float cor[3], h, sat, dist;
  colorCorregir(s, crudo, cor);
  colorHSV(cor, h, sat);
  int8_t i = colorClasificar(s, cor, dist);

  Serial.print(F("RGB ")); Serial.print(constrain((int)(cor[0] * 255), 0, 255));
  Serial.print(','); Serial.print(constrain((int)(cor[1] * 255), 0, 255));
  Serial.print(','); Serial.print(constrain((int)(cor[2] * 255), 0, 255));
  Serial.print(F("  H=")); Serial.print(h, 0);
  Serial.print(F(" S=")); Serial.print(sat, 2);
  Serial.print(F("  -> ")); Serial.print(i >= 0 ? NOMBRE_COLOR[i] : "?");
  if (dist < 1e8f) { Serial.print(F(" (d=")); Serial.print(dist, 3); Serial.print(')'); }
  Serial.print(F("   crudo C=")); Serial.print(c);
  Serial.print(F(" R=")); Serial.print(r);
  Serial.print(F(" G=")); Serial.print(g);
  Serial.print(F(" B=")); Serial.print(b);
  if (cal[s].blanco[0] <= 0) Serial.print(F("  [sin blanco: cb]"));
  if (c >= 65000) Serial.print(F("  SATURADO (bajar ganancia)"));
  else if (c < 50) Serial.print(F("  MUY OSCURO (subir ganancia)"));
  Serial.println();
}

void colorImprimirTodos() {
  for (uint8_t s = 0; s < NUM_SENSORES_COLOR; s++) colorImprimir(s);
}

// cb / cn: guardar la referencia de blanco o negro (sensor s sobre la hoja)
void colorCalibrarExtremo(uint8_t s, bool blanco) {
  float m[4];
  if (!colorOk[s] || !colorPromedioCrudo(s, m)) { Serial.println(F("!! Color: no se pudo leer")); return; }
  float* destino = blanco ? cal[s].blanco : cal[s].negro;
  for (uint8_t k = 0; k < 3; k++) destino[k] = m[k];
  colorCalGuardar(s);
  Serial.print('S'); Serial.print(s);
  Serial.print(blanco ? F(" blanco") : F(" negro"));
  Serial.print(F(" guardado: R=")); Serial.print(m[0], 0);
  Serial.print(F(" G=")); Serial.print(m[1], 0);
  Serial.print(F(" B=")); Serial.println(m[2], 0);
  if (blanco && m[3] >= 60000) Serial.println(F("!! Blanco casi saturado: bajar TCS_GANANCIA o alejar el sensor"));
}

// ca<n>: aprender el color n con el sensor s sobre la muestra
void colorAprender(uint8_t s, int n) {
  if (n < 0 || n >= NUM_COLORES) {
    Serial.print('S'); Serial.print(s); Serial.print(F(": "));
    for (uint8_t i = 0; i < NUM_COLORES; i++) {
      Serial.print(F("ca")); Serial.print(i); Serial.print('='); Serial.print(NOMBRE_COLOR[i]);
      Serial.print(cal[s].ref[i][0] < 0 ? F(" (falta)  ") : F(" (ok)  "));
    }
    Serial.println();
    return;
  }
  float m[4], cor[3], cr[3], dist;
  if (!colorOk[s] || !colorPromedioCrudo(s, m)) { Serial.println(F("!! Color: no se pudo leer")); return; }
  colorCorregir(s, m, cor);
  if (!cromaticidad(cor, cr)) { Serial.println(F("!! Muy oscuro para aprender")); return; }
  // Aviso si se parece demasiado a otro color ya aprendido
  cal[s].ref[n][0] = -1;
  int8_t otro = colorMasCercano(s, cor, dist);
  for (uint8_t k = 0; k < 3; k++) cal[s].ref[n][k] = cr[k];
  colorCalGuardar(s);
  Serial.print('S'); Serial.print(s);
  Serial.print(F(" aprendido ")); Serial.print(NOMBRE_COLOR[n]);
  Serial.print(F(": r=")); Serial.print(cr[0], 3);
  Serial.print(F(" g=")); Serial.print(cr[1], 3);
  Serial.print(F(" b=")); Serial.println(cr[2], 3);
  if (otro >= 0 && dist < 2 * COLOR_DIST_MAX) {
    Serial.print(F("!! Muy parecido a ")); Serial.print(NOMBRE_COLOR[otro]);
    Serial.print(F(" (d=")); Serial.print(dist, 3); Serial.println(F("), pueden confundirse"));
  }
}
