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
extern float velYaw;         // grados/s
extern float yawObjetivo;    // rumbo nominal (lo mueven los giros y el re-alineo con paredes)
extern bool imuOk;

// Centrado con paredes laterales (solo mientras AVANZANDO)
extern float corrLateral;    // grados que se inclina el rumbo deseado: yawRef = yawObjetivo + corrLateral
extern float errorLateral;   // cm, + = hay que irse a la izquierda
extern bool hayLateral;      // el ultimo ciclo hubo pared confiable de algun lado
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
// Pasa las ganancias de P a los PID (tras cambiarlas en vivo)
void aplicarParametros();

float recorridoMM();
void iniciarAvance(float mm);
void iniciarGiro(float grados);   // relativo al rumbo nominal; + = izquierda
void detenerTodo();

// Trabajo corto que debe seguir corriendo mientras el robot espera un movimiento
// (leer el Serial, refrescar el LCD). controlActualizar() la llama en cada vuelta.
// Por defecto vacia; un main de src/apps/ la define si la necesita.
void tareaFondo();

// Corre un ciclo de control si ya pasaron CONTROL_PERIODO_MS. Llamar seguido.
// Devuelve true si corrio el ciclo.
bool controlActualizar();
// Tras una pausa larga sin llamar controlActualizar(): evita un dt enorme
void controlReiniciarReloj();
// Bloquea corriendo el control hasta que termine el avance o giro actual
void esperarMovimiento();
// Corre el control ms milisegundos con el robot frenado: termina de detenerse,
// el yaw registra lo que giro al frenar y se re-estima el sesgo del giroscopio
void asentar(uint16_t ms);
