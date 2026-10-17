#include "pantalla.h"
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

bool pantallaOk = false;

// Lo que se quiere ver y lo que el LCD tiene de verdad
static char deseado[PANTALLA_FILAS][PANTALLA_COLUMNAS];
static char mostrado[PANTALLA_FILAS][PANTALLA_COLUMNAS];
static int8_t cursorFila = -1, cursorCol = -1;   // -1 = posicion desconocida
static uint32_t tUltimaEscritura = 0;

// Dentro de una funcion: los main que no usan el LCD no cargan el objeto
static LiquidCrystal_I2C& lcd() {
  static LiquidCrystal_I2C l(LCD_DIRECCION, PANTALLA_COLUMNAS, PANTALLA_FILAS);
  return l;
}

void pantallaIniciar() {
  Wire.beginTransmission(LCD_DIRECCION);
  pantallaOk = (Wire.endTransmission() == 0);
  if (!pantallaOk) {
    Serial.print(F("!! LCD no responde en 0x")); Serial.println(LCD_DIRECCION, HEX);
    return;
  }
  Wire.setClock(LCD_RELOJ_HZ);
  lcd().init();          // hace su propio Wire.begin(): el reloj se restaura abajo
  lcd().backlight();
  lcd().clear();
  Wire.setClock(I2C_RELOJ_HZ);
  memset(deseado, ' ', sizeof(deseado));
  memset(mostrado, ' ', sizeof(mostrado));
  cursorFila = cursorCol = 0;
}

void pantallaFila(uint8_t fila, const char* texto) {
  if (fila >= PANTALLA_FILAS) return;
  uint8_t i = 0;
  for (; i < PANTALLA_COLUMNAS && texto[i]; i++) deseado[fila][i] = texto[i];
  for (; i < PANTALLA_COLUMNAS; i++) deseado[fila][i] = ' ';
}

void pantallaActualizar() {
  if (!pantallaOk || millis() - tUltimaEscritura < LCD_PAUSA_MS) return;
  for (uint8_t f = 0; f < PANTALLA_FILAS; f++) {
    for (uint8_t c = 0; c < PANTALLA_COLUMNAS; c++) {
      if (deseado[f][c] == mostrado[f][c]) continue;
      Wire.setClock(LCD_RELOJ_HZ);
      if (f != cursorFila || c != cursorCol) lcd().setCursor(c, f);
      lcd().write(deseado[f][c]);
      Wire.setClock(I2C_RELOJ_HZ);
      mostrado[f][c] = deseado[f][c];
      cursorFila = f;
      cursorCol = c + 1;   // el LCD avanza solo
      tUltimaEscritura = millis();
      return;
    }
  }
}
