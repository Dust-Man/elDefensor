#pragma once
#include <Arduino.h>
#include "config.h"
#include "pid.h"

// Encoders, motores, MPU6050 y la cascada de control (velocidad por rueda,
// rumbo y giro). Lo comparten todos los main de src/apps/.

enum Estado : uint8_t { DETENIDO, AVANZANDO, GIRANDO, LIBRE };

extern Parametros P;
extern Estado estado;

extern float yaw;            // grados, continuo
extern float yawObjetivo;    // rumbo nominal (lo mueven los giros)
extern bool imuOk;
extern float velMedida[NUM_MOTORES];     // mm/s
extern float velObjetivo[NUM_MOTORES];   // mm/s

// true si el ultimo movimiento se corto por TIEMPO_MAX_MOV_MS (robot atorado).
// Se limpia al iniciar otro movimiento.
extern bool movExcedido;

void leerCuentas(long out[NUM_MOTORES]);
void encodersEnCero();
void motorAplicar(uint8_t i, float u);
void frenarTodos();
void soltarTodos();

// Motores, encoders, PID y MPU (calibra el giroscopio: robot QUIETO)
void movimientoIniciar();

float recorridoMM();
void iniciarAvance(float mm);
void iniciarGiro(float grados);   // relativo al rumbo nominal; + = izquierda
void detenerTodo();

// Corre un ciclo de control si ya pasaron CONTROL_PERIODO_MS. Llamar seguido.
// Devuelve true si corrio el ciclo.
bool controlActualizar();
// Tras una pausa larga sin llamar controlActualizar(): evita un dt enorme
void controlReiniciarReloj();
// Bloquea corriendo el control hasta que termine el avance o giro actual
void esperarMovimiento();
