#pragma once
#include <Arduino.h>

constexpr uint8_t NUM_MOTORES = 4; //En momento de compilacion
enum { M_DI = 0, M_TI = 1, M_DD = 2, M_TD = 3 }; //Crear constantes por motor

// Pines Motores -> DI, TI, DD, TD
const uint8_t PIN_INA[NUM_MOTORES] = { 22, 24, 26, 28 }; //IN1
const uint8_t PIN_INB[NUM_MOTORES] = { 23, 25, 27, 29 }; //IN2
const uint8_t PIN_EN[NUM_MOTORES]  = {  5,  6,  7,  8 }; //ENA

//Pines encoders con interrupcion -> DI, TI, DD, TD
const uint8_t PIN_ENC_A[NUM_MOTORES] = {  2,  3, 18, 19 }; //Cable Amarillo Señal A del Encoder
const uint8_t PIN_ENC_B[NUM_MOTORES] = { 30, 31, 32, 33 }; //Cable Negro Señal B del Encoder

//Para no recablear, cambiar valor aqui de signo si algo gira mal
const bool MOTOR_INVERTIDO[NUM_MOTORES]   = { false, false, true, true };
const bool ENCODER_INVERTIDO[NUM_MOTORES] = { true, true, false, false };

constexpr float DIAMETRO_RUEDA_MM = 68.0f;    // CALIBRADO
constexpr float TICKS_POR_VUELTA  = 985.0f;   // CALBIRADO

// Ajuste fino: pedir f100 (1 m), medir con cinta lo que avanzó en realidad y
// poner FACTOR_DISTANCIA = real_cm / 100. Ej: avanzó 96 cm -> 0.96
constexpr float FACTOR_DISTANCIA  = .98f;
constexpr float MM_POR_TICK = PI * DIAMETRO_RUEDA_MM / TICKS_POR_VUELTA * FACTOR_DISTANCIA;

// Seguridad: ningún movimiento puede durar más que esto
constexpr uint32_t TIEMPO_MAX_MOV_MS = 8000;

// Servos espejeados (pin 9 y pin 10) -- Autor: Carlos Parrilla
// El servo 2 esta montado al reves: cuando el 1 sube, el 2 baja (180 - angulo).
// Calibrar el cero de cada uno con 'z1<g>' / 'z2<g>' y poner aqui el valor.
const uint8_t PIN_SERVO_1 = 9;
const uint8_t PIN_SERVO_2 = 10;
constexpr uint16_t SERVO_PULSO_MIN = 544;    // us, configuracion probada
constexpr uint16_t SERVO_PULSO_MAX = 2400;   // us, configuracion probada
constexpr int SERVO_CERO_1 = 0;              // grados crudos del servo 1 en su cero
constexpr int SERVO_CERO_2 = 180;            // grados crudos del servo 2 en su cero (espejo)
constexpr uint8_t SERVO_MS_POR_GRADO = 15;   // velocidad del barrido 'zb'

// MPU6050 - giroscopio
constexpr uint8_t MPU_DIRECCION = 0x68;
constexpr float GIRO_LSB_POR_GPS = 65.5f;   // sensibilidad para el rango +-500 grados/s
constexpr float YAW_SIGNO = 1.0f;           // cambiar a -1.0f si el yaw crece al reves

// Multiplexor I2C TCA9548A (SDA 20, SCL 21). A0-A2 a GND = 0x70
constexpr uint8_t TCA_DIRECCION = 0x70;

// TCS34725 - sensores de color, cada uno en su canal del multiplexor
constexpr uint8_t NUM_SENSORES_COLOR = 2;
const uint8_t CANAL_COLOR[NUM_SENSORES_COLOR] = { 0, 1 };   // SD0/SC0, SD1/SC1
constexpr uint8_t TCS_DIRECCION = 0x29;
constexpr uint8_t TCS_ATIME = 0xEB;          // integracion 50 ms  (256 - 50/2.4)
constexpr uint8_t TCS_GANANCIA = 0x01;       // 0x00=1x 0x01=4x 0x02=16x 0x03=60x
constexpr uint16_t COLOR_PERIODO_MS = 300;   // cada cuanto imprime el modo continuo 'cc'
constexpr uint16_t COLOR_INTEGRACION_MS = 55; // espera entre muestras al calibrar (>= integracion)
constexpr uint8_t COLOR_MUESTRAS_CAL = 8;     // muestras promediadas al calibrar / aprender
constexpr float COLOR_DIST_MAX = 0.10f;      // distancia maxima para aceptar un color aprendido
constexpr float COLOR_BRILLO_MIN = 0.08f;    // por debajo de esto no hay nada que clasificar
constexpr int COLOR_EEPROM_DIR = 0;          // donde se guarda la calibracion

// Ganancias PID (se aplican en aplicarParametros(), solo desde setup())
struct Parametros {
  // Velocidad de cada motor: PID PURO, sin feedforward.
  // La integral ahora carga todo el PWM de regimen (antes lo daba el ff), por eso
  // NO hay limite propio de integral: su unica cota es salidaMax via anti-windup.
  float velKp = 0.4f; //Que tanto reacciona cada rueda frente al error
  float velKi = 1.5f; //Que tanto tiempo esta dispuesta a esperar si falla mucho
  float velErrorThreshold = 1.0f; // mm/s - banda muerta de la integral (solo la
                                  // integral; P sigue corrigiendo). CALIBRAR: con
                                  // esta calibracion 1 tick por ciclo de 20 ms son
                                  // ~10.6 mm/s, asi que 1.0 queda por debajo de la
                                  // resolucion del encoder y casi no actua. Si zumba
                                  // o caza en reposo, subir hacia ~5.
  float velFiltro = 0.3f;  //

  // Medido en lazo abierto con 'v<pwm>'. Ya no alimenta ningun feedforward;
  // queda como referencia para verificar que las 4 ruedas responden parecido.
  // PWM 100 -> mm/s: DI=635  TI=604  DD=606  TD=632
  // PWM 200 -> mm/s: DI=811  TI=819  DD=823  TD=830
  //DI=304  TI=308  DD=312  TD=318
  //DI=411  TI=416  DD=422  TD=426

  // Mantener rumbo recto
  //Esto es por lado
  float rumboKp = 2.0f;
  float rumboKi = 0.0f;
  float rumboKd = 0.0f;
  float rumboMax = 60.0f;    // limite de la correccion diferencial entre lados

  // Girar donde este
  float giroKp = 2.0f;
  float giroKd = 0.0f;
  float giroMax = 150.0f;
  float giroMin = 40.0f;     // Vencer la friccion
  float giroTol = 2.0f;      // Grados para dar el giro terminado

  // Distancia a recorrer
  float distKp = 1.0f;
  float vMax = 200.0f;       // mm/s
  float vMin = 40.0f;
  float distTol = 5.0f;      // mm de tolerancia
  float acel = 400.0f;       // mm/s^2, rampa solo al acelerar

  float maxPwm = 200.0f;     // Limite de PWM real
};
