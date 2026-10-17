#pragma once
#include <Arduino.h>
#include "config.h"

// LCD 16x2 I2C (PCF8574) que no frena el lazo de control: pantallaFila() solo
// cambia el texto deseado y pantallaActualizar() escribe un caracter cambiado
// por llamada (cada LCD_PAUSA_MS). Llamarla seguido, p. ej. desde tareaFondo().
constexpr uint8_t PANTALLA_COLUMNAS = 16;
constexpr uint8_t PANTALLA_FILAS = 2;

extern bool pantallaOk;

// Llamar despues de movimientoIniciar() (ahi se hace Wire.begin y el reloj del bus).
// Sin LCD en LCD_DIRECCION todo lo demas no hace nada.
void pantallaIniciar();
// Texto de una fila completa: se rellena con espacios, lo que sobre se corta
void pantallaFila(uint8_t fila, const char* texto);
void pantallaActualizar();
