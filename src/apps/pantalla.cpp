// Banco del LCD: escanea el bus I2C, inicia la pantalla y pasa por varias
// pruebas en ciclo. Por Serial: "1 texto" / "2 texto" escribe esa fila y
// detiene el ciclo; "c" lo reanuda. Compilar/subir con [env:pantalla].
// No mueve motores.
#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "pantalla.h"

constexpr uint16_t PASO_MS = 2500;   // tiempo de cada prueba

static uint8_t prueba = 0;
static uint32_t tPrueba = 0;
static bool ciclo = true;

static void escanearI2C() {
  Serial.println(F("Escaneando I2C..."));
  uint8_t n = 0;
  for (uint8_t dir = 1; dir < 127; dir++) {
    Wire.beginTransmission(dir);
    if (Wire.endTransmission() == 0) {
      Serial.print(F("  0x")); Serial.println(dir, HEX);
      n++;
    }
  }
  if (n == 0) Serial.println(F("  nada (revisar SDA 20, SCL 21, GND y 5V)"));
}

static void mostrarPrueba() {
  char fila[PANTALLA_COLUMNAS + 1];
  switch (prueba) {
    case 0:
      pantallaFila(0, "Prueba LCD 16x2");
      snprintf(fila, sizeof(fila), "I2C 0x%02X OK", LCD_DIRECCION);
      pantallaFila(1, fila);
      break;
    case 1: {   // todos los pixeles encendidos
      memset(fila, 0xFF, PANTALLA_COLUMNAS);
      fila[PANTALLA_COLUMNAS] = '\0';
      pantallaFila(0, fila);
      pantallaFila(1, fila);
      break;
    }
    case 2:     // bordes: que no falte la primera ni la ultima columna
      pantallaFila(0, "<              >");
      pantallaFila(1, "0123456789ABCDEF");
      break;
    case 3:
      pantallaFila(0, "abcdefghijklmnop");
      pantallaFila(1, "!#$%&()*+-./:;=?");
      break;
    case 4:
      pantallaFila(0, "");
      pantallaFila(1, "");
      break;
  }
}

// Ultima prueba: contador vivo, para ver que pantallaActualizar no se traba
static void contador() {
  static uint32_t tAnterior = 0;
  if (millis() - tAnterior < 100) return;
  tAnterior = millis();
  char fila[PANTALLA_COLUMNAS + 1];
  pantallaFila(0, "Contador");
  snprintf(fila, sizeof(fila), "t=%lu.%lu s", millis() / 1000, (millis() / 100) % 10);
  pantallaFila(1, fila);
}

static void leerSerial() {
  static char linea[PANTALLA_COLUMNAS + 4];
  static uint8_t largo = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c != '\n') {
      if (largo < sizeof(linea) - 1) linea[largo++] = c;
      continue;
    }
    linea[largo] = '\0';
    largo = 0;
    if (linea[0] == 'c') {
      ciclo = true;
      tPrueba = millis() - PASO_MS;   // pasa a la siguiente ya
      Serial.println(F("ciclo reanudado"));
    } else if ((linea[0] == '1' || linea[0] == '2') && linea[1] == ' ') {
      ciclo = false;
      pantallaFila(linea[0] - '1', linea + 2);
      Serial.print(F("fila ")); Serial.print(linea[0]);
      Serial.print(F(": ")); Serial.println(linea + 2);
    } else {
      Serial.println(F("uso: '1 texto', '2 texto' o 'c'"));
    }
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(I2C_RELOJ_HZ);
  Wire.setWireTimeout(3000, true);
  escanearI2C();
  pantallaIniciar();
  if (pantallaOk) Serial.println(F("LCD listo. '1 texto', '2 texto' o 'c'"));
  mostrarPrueba();
  tPrueba = millis();
}

void loop() {
  leerSerial();
  if (ciclo) {
    if (millis() - tPrueba >= PASO_MS) {
      tPrueba = millis();
      prueba = (prueba + 1) % 6;
      if (prueba < 5) mostrarPrueba();
    }
    if (prueba == 5) contador();
  }
  pantallaActualizar();
}
