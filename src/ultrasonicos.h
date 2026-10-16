#pragma once
#include <Arduino.h>
#include "config.h"

// Ultrasonicos HC-SR04, indices de PIN_US_TRIG / PIN_US_ECHO
enum { US_FRENTE = 0, US_IZQ = 1, US_DER = 2 };
extern const char* const NOMBRE_US[NUM_ULTRASONICOS];

constexpr float US_SIN_ECO = -1.0f;        // nada dentro de US_DIST_MAX_CM (o sensor desconectado)
constexpr float US_NO_ASIGNADO = -2.0f;    // pines aun en US_SIN_PIN

bool ultrasonicoAsignado(uint8_t i);
void ultrasonicosIniciar();
// Distancia en cm, o US_SIN_ECO / US_NO_ASIGNADO
float ultrasonicoLeerCm(uint8_t i);
// Mediana de n lecturas (n <= 5) del sensor i. Bloquea ~n*(12+US_PAUSA_MS) ms
float ultrasonicoMedianaCm(uint8_t i, uint8_t n);
void ultrasonicosImprimir(const float d[NUM_ULTRASONICOS]);
void ultrasonicosLeerTodos();
