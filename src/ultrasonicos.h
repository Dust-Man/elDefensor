#pragma once
#include <Arduino.h>
#include "config.h"

// Ultrasonicos HC-SR04, indices de PIN_US_TRIG / PIN_US_ECHO
enum { US_FRENTE = 0, US_IZQ = 1, US_DER = 2, US_ATRAS = 3 };
extern const char* const NOMBRE_US[NUM_ULTRASONICOS];

constexpr float US_SIN_ECO = -1.0f;        // nada dentro de US_DIST_MAX_CM (o sensor desconectado)
constexpr float US_NO_ASIGNADO = -2.0f;    // pines aun en US_SIN_PIN

bool ultrasonicoAsignado(uint8_t i);
void ultrasonicosIniciar();
// Distancia en cm, o US_SIN_ECO / US_NO_ASIGNADO. Mas lejos que maxCm = sin eco
// (un maxCm chico acorta el bloqueo: ~maxCm*58 us + 1 ms)
float ultrasonicoLeerCm(uint8_t i, float maxCm = US_DIST_MAX_CM);
// Mediana de n lecturas (n <= 5) del sensor i. Bloquea ~n*(12+US_PAUSA_MS) ms
float ultrasonicoMedianaCm(uint8_t i, uint8_t n);
void ultrasonicosImprimir(const float d[NUM_ULTRASONICOS]);
void ultrasonicosLeerTodos();

// Laterales mientras avanza (centrado). Llamar Actualizar una vez por ciclo de
// control: dispara un solo sensor, alternando izq/der (bloquea <= ~2 ms).
void ultrasonicosLateralReiniciar();   // al iniciar cada avance
// Para pistas donde la pared no esta pegada a la celda: cm que se suman al
// objetivo (US_LATERAL_OBJ_*) y al maximo (US_LATERAL_MAX_CM) de cada lado.
// 0, 0 = laberinto (lo de siempre).
void ultrasonicosLateralDesfase(float izqCm, float derCm);
void ultrasonicosLateralActualizar();
// Ultimas distancias aceptadas, o US_SIN_ECO si ese lado no tiene pared confiable
void ultrasonicosLateral(float& izq, float& der);
// Error lateral en cm, + = hay que irse a la IZQUIERDA. ambas = hay pared a los
// dos lados. false si ningun lado tiene pared confiable.
bool ultrasonicosErrorLateral(float& eCm, bool& ambas);
// Debug del centrado ('uc'), k: 0 = izq, 1 = der. Ultima lectura sin filtrar y por
// que se acepto o no: ok, sin eco, cerca, lejos, salto, espera (falta la 2a
// lectura parecida), vieja (nadie esta disparando ese lado)
float ultrasonicosLateralCrudo(uint8_t k);
const __FlashStringHelper* ultrasonicosLateralMotivo(uint8_t k);
