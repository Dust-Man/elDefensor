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
constexpr float FACTOR_DISTANCIA  = .97f;
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
constexpr float YAW_SIGNO = 1.022f;           // cambiar a -1.0f si el yaw crece al reves
// El MPU6050 trae hasta +-3% de error de sensibilidad: cada giro de 90 deja ~2 grados
// de error. CALIBRAR: con 'l' (ruedas libres) girar el robot a mano 4 vueltas exactas
// contra una referencia en el piso y poner GIRO_ESCALA = 1440 / yaw leido.
constexpr float GIRO_ESCALA = 1.0f;

// Bus I2C (SDA 20, SCL 21): MPU, multiplexor y LCD
constexpr uint32_t I2C_RELOJ_HZ = 400000;

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
constexpr float COLOR_BLANCO_MIN = 0.7f;     // los 3 canales corregidos arriba de esto = blanco (cinta, franjas)
constexpr int COLOR_EEPROM_DIR = 0;          // donde se guarda la calibracion

// Ultrasonicos HC-SR04 -> FRENTE, IZQ, DER, ATRAS
// PINES POR DEFINIR: un sensor con US_SIN_PIN se ignora (no se toca ningun pin).
constexpr uint8_t NUM_ULTRASONICOS = 4;
constexpr uint8_t US_SIN_PIN = 255;
//const uint8_t PIN_US_TRIG[NUM_ULTRASONICOS] = { 39, 37,  41 };
//const uint8_t PIN_US_ECHO[NUM_ULTRASONICOS] = { 38, 36,  40 };
const uint8_t PIN_US_TRIG[NUM_ULTRASONICOS] = { 39,  41, 37, 12 };
const uint8_t PIN_US_ECHO[NUM_ULTRASONICOS] = { 38,  40, 36, 13 };
constexpr float US_DIST_MAX_CM = 200.0f;     // mas lejos que esto cuenta como "sin eco" (acota el bloqueo)
constexpr uint16_t US_PERIODO_MS = 150;      // cada cuanto imprime el modo continuo 'uu'
constexpr uint16_t US_PAUSA_MS = 30;         // entre disparos, para que no se crucen los ecos

// Centrado con las paredes laterales mientras avanza (lo aplica movimiento.cpp)
// CALIBRAR con 'um': robot centrado en una celda con pared a los dos lados.
constexpr float US_LATERAL_OBJ_IZQ_CM = 6.3f;    // lectura izq con el robot centrado
constexpr float US_LATERAL_OBJ_DER_CM = 5.8f;    // lectura der con el robot centrado
constexpr float US_LATERAL_MIN_CM = 2.0f;        // por debajo el HC-SR04 no es confiable
constexpr float US_LATERAL_MAX_CM = 14.0f;       // mas lejos no es pared pegada (~objetivo + 6)
constexpr float US_LATERAL_SALTO_CM = 2.0f;      // salto mayor entre lecturas = filo o hueco: se ignora
constexpr uint16_t US_LATERAL_VIGENCIA_MS = 100; // lectura mas vieja que esto ya no cuenta

// LCD 16x2 con backpack PCF8574, en el bus principal (NO en un canal del
// multiplexor). Si no se ve texto: potenciometro de contraste, o probar 0x3F.
constexpr uint8_t LCD_DIRECCION = 0x27;
constexpr uint32_t LCD_RELOJ_HZ = 100000;    // el PCF8574 solo garantiza 100 kHz
constexpr uint16_t LCD_PAUSA_MS = 2;         // entre caracteres: ~1.2 ms cada uno, no frenar el lazo

// Vision (celular con CVSquirtDeAga por el USB del Mega, Serial a 115200).
// Recibe M,<bola>,<n>,<id1>,...,<idn> y lo muestra en el LCD.
constexpr uint32_t VISION_TIMEOUT_MS = 10000;   // sin mensaje valido: "Sin enlace"

// Pin sin conectar (boton): el codigo lo ignora
constexpr uint8_t SIN_PIN = 255;

// Boton de arranque (a GND, usa INPUT_PULLUP). Con SIN_PIN arranca solo
// tras INICIO_ESPERA_MS.
constexpr uint8_t PIN_BOTON_INICIO = SIN_PIN;
constexpr uint32_t INICIO_ESPERA_MS = 10000;

// ---------------- Pista A (laberinto) ----------------
constexpr float CELDA_MM = 300.0f;           // una unidad
// iniciarGiro(+g) gira a la IZQUIERDA (yaw positivo); si el robot gira al
// lado contrario en la pista, cambiar el signo.
constexpr float GIRO_DERECHA_GRADOS = -90.0f;

// Paredes: hay pared si el ultrasonico lee menos que esto. MEDIR con 'um' con el
// robot centrado en una celda: pared pegada vs. la pared de la celda siguiente (~30 cm mas).
constexpr float US_PARED_CM = 20.0f;
constexpr uint8_t US_MUESTRAS = 3;           // mediana de cuantas lecturas por sensor

// Centrado con la pared de enfrente al llegar a una celda (corrige el error de encoders)
constexpr bool ALINEAR_CON_PARED = true;
constexpr float US_FRENTE_OBJETIVO_CM = 5.5f;   // MEDIR: lectura de frente con el robot centrado
constexpr float ALINEAR_MAX_CM = 8.0f;          // errores mayores se ignoran (lectura dudosa)
// Tras cada giro el robot queda corrido (no gira justo sobre su centro): se
// centra con la pared de enfrente y/o la de atras (con las dos, promedia).
// Tambien en la casilla de inicio.
constexpr bool ALINEAR_CON_ATRAS = true;
constexpr float US_ATRAS_OBJETIVO_CM = 6.48f;    // MEDIR con 'um': lectura de atras con el robot centrado
constexpr float ALINEAR_ATRAS_MAX_CM = 10.0f;   // el corrimiento del giro puede ser mayor que ALINEAR_MAX_CM

// Color del piso. El sensor va adelante del centro: lee la celda siguiente
// mientras avanza, asi detecta la meta roja antes de pisarla y regresa.
constexpr uint8_t SENSOR_COLOR_PISO = 0;     // cual TCS34725 mira al piso al frente
constexpr float COLOR_OFFSET_MM = 80.0f; //80   // MEDIR: del centro del robot al sensor, hacia adelante
constexpr float COLOR_MARGEN_MM = 10.0f;     // no leer cerca de la orilla entre celdas (franja blanca)
constexpr uint8_t COLOR_LECTURAS_MIN = 2  ;    // lecturas seguidas iguales para aceptar un color

// Bonus 1: true = regresa por TODAS las unidades recorridas en orden inverso
// (lo que pide el reglamento). false = solo por la ruta sin ciclos (mas rapido).
constexpr bool RETORNO_EXACTO = true;
// Despues de esto se deja de explorar y se va a la meta (si ya se conoce).
// La ronda dura 6 min y el regreso exacto tarda casi lo mismo que la ida.
constexpr uint32_t PISTA_A_LIMITE_EXPLORAR_MS = 180000;
// true = deja de explorar y va a la meta en cuanto ya vio todos los colores
// (incluida la meta roja). false = explora todo el laberinto.
constexpr bool PISTA_A_TERMINAR_CON_COLORES = true;
constexpr uint32_t PISTA_A_PAUSA_META_MS = 2000;   // quieto en la meta para que el juez la cuente

// ---------------- Pista B (lineas blancas y colores) ----------------
// Desde que seccion arranca (la 1, pelota, aun no esta). 2 = checkpoint IN de
// las lineas, 3 = checkpoint F (para probar la seccion 3 sola).
constexpr uint8_t PISTA_B_SECCION_INICIAL = 2;

// Seccion 2: 2 filas x B_COLUMNAS de verde. IN a la izquierda de la fila de
// abajo, F a la derecha de la fila de arriba. Sensores: S0 izquierdo, S1 derecho
// (ambos COLOR_OFFSET_MM adelante del centro).
constexpr uint8_t B_COLUMNAS = 4;
constexpr int8_t B_FIN_LADO = 1;             // fila de F: +1 = izquierda (arriba), -1 = derecha
constexpr float B_VEL_LINEAS = 100.0f;       // mm/s mientras busca lineas (frena antes de pisarlas)
constexpr uint8_t B_LECTURAS_BLANCO = 2;     // lecturas blancas seguidas para dar una linea
constexpr float B_MARGEN_SEPARACION_MM = 40.0f;   // ignora el blanco junto a la franja IN|verde
// Centrado con las paredes de los lados mientras avanza (usa la calibracion
// US_LATERAL_* del laberinto). En la seccion 2 las paredes estan a los lados de
// las 2 filas: lo que debe leer cada sensor sale de la fila en la que va.
// false si en la pista real no hay paredes ahi.
constexpr bool B_CENTRAR_PAREDES = true;

// Seccion 3: cada color dice a que celda ir (respecto a la pista; "arriba" =
// hacia donde ve el robot al salir de F). Verde = FIN.
constexpr uint8_t B_S3_LECTURAS = 5;         // lecturas por sensor en cada celda (voto)
constexpr uint8_t B_S3_MAX_PASOS = 12;       // seguridad: maximo de celdas

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
  float rumboKp = 5.0f;      // con rumboKd probar subirlo a 4-6
  float rumboKi = 0.0f;
  float rumboKd = 0.4f;      // mm/s por grado/s: amortigua con la velocidad angular del
                             // giroscopio (no con la derivada del error). Probar 0.3-0.5
  float rumboMax = 60.0f;    // limite de la correccion diferencial entre lados

  // Centrado con paredes laterales: el error lateral inclina el rumbo deseado y
  // el lazo de rumbo (giroscopio) lo sigue. Sin paredes queda solo el giroscopio.
  bool centrarParedes = true;
  float centradoKp = 2.0f;        // grados de rumbo por cm de error lateral
  float centradoMax = 8.0f;       // grados, inclinacion maxima del rumbo
  float centradoTasa = 30.0f;     // grados/s, que tan rapido cambia la inclinacion
  float centradoKi = 0.5f;        // grados/(cm*s): re-alinea yawObjetivo con las paredes
  float realineoMax = 3.0f;       // grados maximo de re-alineo por avance
  float centradoDistMin = 100.0f; // mm: avances mas cortos (ajustes) no centran

  // Quieto y frenado antes de leer paredes: el yaw registra lo que giro al
  // frenar y se re-estima el sesgo del giroscopio
  float asentarMs = 200.0f;

  // Girar donde este
  float giroKp = 2.0f;
  float giroKd = 0.0f;       // mm/s por grado/s, igual que rumboKd
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
