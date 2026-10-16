#pragma once
#include <Arduino.h>
#include "config.h"

// Sensores de color TCS34725 (via multiplexor TCA9548A)

// Indices de NOMBRE_COLOR (el orden es el de la calibracion guardada en EEPROM)
constexpr uint8_t NUM_COLORES = 6;
enum : int8_t {
  COLOR_NINGUNO = -1,   // blanco, negro o desconocido
  COLOR_VERDE = 0, COLOR_ROJO, COLOR_ROSA, COLOR_NARANJA, COLOR_AMARILLO, COLOR_CYAN
};
extern const char* const NOMBRE_COLOR[NUM_COLORES];

extern bool colorOk[NUM_SENSORES_COLOR];

void escanearI2C();
void colorIniciarTodos();   // llamar despues de movimientoIniciar() (ahi se hace Wire.begin)

void colorCalCargar();
void colorCalDefecto(uint8_t s);
void colorCalGuardar(uint8_t s);
void colorCalibrarExtremo(uint8_t s, bool blanco);
void colorAprender(uint8_t s, int n);

bool colorLeerCrudo(uint8_t s, uint16_t& c, uint16_t& r, uint16_t& g, uint16_t& b);
// Lectura clasificada del sensor s: color = indice de NOMBRE_COLOR o COLOR_NINGUNO.
// false si no hay dato nuevo valido. No bloquea (el sensor integra solo).
bool colorDetectar(uint8_t s, int8_t& color);

void colorImprimir(uint8_t s);
void colorImprimirTodos();
