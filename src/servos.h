#pragma once
#include <Arduino.h>
#include "config.h"

// Servos espejeados  -- Autor: Carlos Parrilla
extern int servoAngulo;   // angulo logico actual

int servoMin();
int servoMax();
void servosConectar();
void servosSoltar();
void servosImprimir();
void servosMover(int a, bool imprimir = true);
void servoIndividual(uint8_t n, int crudo);
void servosBarrido();
