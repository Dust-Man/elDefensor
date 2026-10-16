#include <Wire.h>
#include "movimiento.h"
#include "ultrasonicos.h"

Parametros P;

Estado estado = DETENIDO;

//Encoders
volatile long cuentas[NUM_MOTORES] = { 0, 0, 0, 0 }; //Pulsos por encoder de motor
int8_t signoEnc[NUM_MOTORES]; //Signos + - por motor

uint32_t tInicioMov = 0;   // para el tiempo máximo de seguridad
bool movExcedido = false;

// Cada que canal A cambia de estado se ejecuta
static inline void flancoEncoder(uint8_t i) {
  bool a = digitalRead(PIN_ENC_A[i]); //Leer canal A de encoder i
  bool b = digitalRead(PIN_ENC_B[i]); //Leer canal B de encoder i
  cuentas[i] += (a != b) ? signoEnc[i] : -signoEnc[i]; //Invertir a partir de signo y comparar encoders
}
void isrDI() { flancoEncoder(M_DI); }
void isrTI() { flancoEncoder(M_TI); }
void isrDD() { flancoEncoder(M_DD); }
void isrTD() { flancoEncoder(M_TD); }

// Proporciona fuente confiable de cuanto han girado los motores
void leerCuentas(long out[NUM_MOTORES]) {
  noInterrupts();
  for (uint8_t i = 0; i < NUM_MOTORES; i++) out[i] = cuentas[i];
  interrupts();
}

//Leer HIGH hasta que haya cambios
void iniciarEncoders() {
  for (uint8_t i = 0; i < NUM_MOTORES; i++) {
    pinMode(PIN_ENC_A[i], INPUT_PULLUP);
    pinMode(PIN_ENC_B[i], INPUT_PULLUP);
    signoEnc[i] = ENCODER_INVERTIDO[i] ? -1 : 1;
  }

  // Cada que canal A cambia de estado se ejecuta para ver posicion
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A[M_DI]), isrDI, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A[M_TI]), isrTI, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A[M_DD]), isrDD, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A[M_TD]), isrTD, CHANGE);
}

//Motores
void iniciarMotores() {
  for (uint8_t i = 0; i < NUM_MOTORES; i++) {
    pinMode(PIN_INA[i], OUTPUT);
    pinMode(PIN_INB[i], OUTPUT);
    pinMode(PIN_EN[i], OUTPUT);
  }
}

// u con signo + = adelante pero si tiene - = atras
void motorAplicar(uint8_t i, float u) {
  if (MOTOR_INVERTIDO[i]) u = -u;
  int pwm = constrain((int)(fabs(u) + 0.5f), 0, (int)P.maxPwm); //Quedarse dentro de un rango de pwm
  if (pwm == 0) {
    //Giro sin resistencia
    digitalWrite(PIN_INA[i], LOW);
    digitalWrite(PIN_INB[i], LOW);
  } else if (u > 0) {
    //Giro con sentido
    digitalWrite(PIN_INA[i], HIGH);
    digitalWrite(PIN_INB[i], LOW);
  } else {
    //Giro con sentido contrario
    digitalWrite(PIN_INA[i], LOW);
    digitalWrite(PIN_INB[i], HIGH);
  }
  analogWrite(PIN_EN[i], pwm);
}

void frenarTodos() {
  for (uint8_t i = 0; i < NUM_MOTORES; i++) {
    digitalWrite(PIN_INA[i], HIGH);
    digitalWrite(PIN_INB[i], HIGH);
    analogWrite(PIN_EN[i], 255);   // freno activo, corriente se opne a su mov
  }
}

// Rueda libre: sin freno, para poder girar las ruedas con la mano
void soltarTodos() {
  for (uint8_t i = 0; i < NUM_MOTORES; i++) motorAplicar(i, 0);
}

//MPU eje z -- REVISAR
float yaw = 0;         // grados, continuo
float velYaw = 0;      // grados/s, ya sin sesgo
float sesgoGz = 0;     // sesgo del giroscopio en cuentas crudas
bool imuOk = false;
uint32_t tImu = 0;     // micros de la ultima lectura del giroscopio
uint32_t tUltimoMov = 0;   // millis de la ultima vez que el robot no estaba quieto

// Se lee mas rapido que el lazo de 20 ms: una sola muestra cada 20 ms pierde
// parte del giro (el MPU entrega 1 kHz con el filtro de 44 Hz)
constexpr uint32_t IMU_PERIODO_US = 2000;   // ~500 Hz
// El sesgo cambia al calentarse: con el robot quieto se re-estima poco a poco
constexpr uint32_t QUIETO_MS = 100;          // frenado y sin ticks este tiempo = quieto
constexpr float QUIETO_GPS = 1.0f;           // una lectura mas rapida que esto es giro, no sesgo
constexpr float SESGO_ALFA = 0.01f;          // por lectura (~0.2 s de constante a 500 Hz)

void mpuEscribir(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_DIRECCION);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

bool mpuLeerGiroZ(int16_t &gz) {
  Wire.beginTransmission(MPU_DIRECCION);
  Wire.write(0x47);   // GYRO_ZOUT_H
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)MPU_DIRECCION, (uint8_t)2) != 2) return false;
  uint8_t alto = Wire.read();
  uint8_t bajo = Wire.read();
  gz = (int16_t)((alto << 8) | bajo);
  return true;
}

bool mpuIniciar() {
  Wire.begin();
  Wire.setClock(400000);
  Wire.setWireTimeout(3000, true);   // si el ruido de motores traba el bus, no se cuelga
  mpuEscribir(0x6B, 0x01);   // PWR_MGMT_1: despertar
  delay(50);
  mpuEscribir(0x1A, 0x03);   // CONFIG: filtro pasa-bajas ~44Hz (quita vibracion de motores)
  mpuEscribir(0x1B, 0x08);   // GYRO_CONFIG: rango +-500 grados/s
  delay(50);
  int16_t prueba;
  return mpuLeerGiroZ(prueba);
}

// Robot QUIETO al llamar esto: promedia el giroscopio para hallar su sesgo.
void calibrarGiroscopio() {
  if (!imuOk) return;
  Serial.println(F("Calibrando giroscopio... no muevan el robot."));
  float suma = 0;
  int validas = 0;
  for (int k = 0; k < 300; k++) {
    int16_t gz;
    if (mpuLeerGiroZ(gz)) { suma += gz; validas++; }
    delay(3);
  }
  if (validas > 0) sesgoGz = suma / validas;
  yaw = 0;
  velYaw = 0;
  tImu = micros();
  Serial.println(F("Giroscopio listo."));
}

// Frenado y sin ticks en los encoders desde hace QUIETO_MS
bool robotQuieto() {
  return estado == DETENIDO && millis() - tUltimoMov >= QUIETO_MS;
}

// Se llama en cada controlActualizar(); lee a su propio ritmo (IMU_PERIODO_US)
void actualizarIMU() {
  if (!imuOk) return;
  uint32_t ahora = micros();
  if (ahora - tImu < IMU_PERIODO_US) return;
  int16_t gz;
  if (!mpuLeerGiroZ(gz)) return;
  float dt = (ahora - tImu) / 1e6f;
  tImu = ahora;
  if (robotQuieto() && fabs(gz - sesgoGz) < QUIETO_GPS * GIRO_LSB_POR_GPS)
    sesgoGz += SESGO_ALFA * (gz - sesgoGz);
  velYaw = (gz - sesgoGz) / GIRO_LSB_POR_GPS * YAW_SIGNO * GIRO_ESCALA;
  if (dt <= 0.1f) yaw += velYaw * dt;   // tras un bloqueo largo la muestra no representa el hueco
}

//Lazos de control
//Crear objetos PID
PID pidVel[NUM_MOTORES];
PID pidRumbo;
PID pidGiro;

float velMedida[NUM_MOTORES]   = { 0, 0, 0, 0 };   // velocidad real por rueda en mm/s
float velObjetivo[NUM_MOTORES] = { 0, 0, 0, 0 };   // target por rueda en mm/s
long cuentasPrevias[NUM_MOTORES] = { 0, 0, 0, 0 }; //conteo del encoder de la lectura pasada

float yawObjetivo = 0;
float vBase = 0;
float distObjetivoMM = 0;
long cuentasInicio[NUM_MOTORES];

// Centrado con paredes laterales
float corrLateral = 0;    // grados que se inclina el rumbo deseado
float errorLateral = 0;   // cm, + = hay que irse a la izquierda
bool hayLateral = false;  // este ciclo hubo pared confiable de algun lado
float realineo = 0;       // grados que se ha re-alineado yawObjetivo en este avance
constexpr float REALINEO_ERROR_CM = 1.5f;   // solo re-alinea ya casi centrado

void encodersEnCero() {
  noInterrupts();
  for (uint8_t i = 0; i < NUM_MOTORES; i++) cuentas[i] = 0;
  interrupts();
  leerCuentas(cuentasPrevias);
}

void aplicarParametros() {
  // OJO: el 5o argumento de configurar() es errorThreshold, NO un limite de integral.
  // Se pasa explicito en los tres para que ninguno herede un valor con otro significado.
  // unaDireccion=true solo en las ruedas: protege al driver de un cambio de signo brusco.
  // Rumbo y giro producen una correccion DIFERENCIAL que SI debe poder cambiar de signo
  // (asi corrigen hacia los dos lados) -> unaDireccion=false.
  // Su D (rumboKd, giroKd) no va en el PID: sale de la velocidad angular del
  // giroscopio en pasoAvanzar/pasoGirar, asi no da golpe cuando cambia el objetivo.
  for (uint8_t i = 0; i < NUM_MOTORES; i++)
    pidVel[i].configurar(P.velKp, P.velKi, 0.0f, P.maxPwm, P.velErrorThreshold, true);
  pidRumbo.configurar(P.rumboKp, P.rumboKi, 0.0f, P.rumboMax, 0.0f, false);
  pidGiro.configurar(P.giroKp,   0.0f,      0.0f, P.giroMax,  0.0f, false);
}

//Retroalimentacion aplicada a los 4 PID, cada 20ms. Devuelve true si alguna rueda se movio
bool medirVelocidades(float dt) {
  long c[NUM_MOTORES];
  leerCuentas(c);
  bool movio = false;
  for (uint8_t i = 0; i < NUM_MOTORES; i++) {
    if (labs(c[i] - cuentasPrevias[i]) > 1) movio = true;   // 1 tick puede ser vibracion
    float v = (c[i] - cuentasPrevias[i]) * MM_POR_TICK / dt;
    cuentasPrevias[i] = c[i];
    velMedida[i] += P.velFiltro * (v - velMedida[i]);
  }
  return movio;
}

void lazoVelocidad(float objIzq, float objDer, float dt) {
  velObjetivo[M_DI] = velObjetivo[M_TI] = objIzq;
  velObjetivo[M_DD] = velObjetivo[M_TD] = objDer;
  for (uint8_t i = 0; i < NUM_MOTORES; i++) {
    float obj = velObjetivo[i];
    // La direccion autorizada en este ciclo es la que pidio la capa de arriba.
    // Si el PID quisiera salir al otro lado (p.ej. al frenar al final del tramo,
    // con la integral todavia cargada), se fuerza a 0 en vez de invertir la
    // polaridad del puente con la rueda girando.
    pidVel[i].fijarDireccionPermitida(obj >= 0);
    float u = pidVel[i].calcular(obj, velMedida[i], dt);
    motorAplicar(i, u);
  }
}

// Cuánto ha avanzado el robot desde que empezó el movimiento actual promediando los 4 encoders
float recorridoMM() {
  long c[NUM_MOTORES];
  leerCuentas(c);
  float suma = 0;
  for (uint8_t i = 0; i < NUM_MOTORES; i++) suma += (c[i] - cuentasInicio[i]);
  return suma / NUM_MOTORES * MM_POR_TICK;
}

void terminarMovimiento() {
  frenarTodos();
  vBase = 0;
  estado = DETENIDO;
}

// Cascada: el error lateral (ultrasonicos) se convierte en cuantos grados inclinar
// el rumbo deseado, y el lazo de rumbo (giroscopio) lo sigue. Al ser un angulo, lo
// que se corrige por cm recorrido no depende de la velocidad.
// Solo hacia adelante: de reversa el mismo giro mueve al robot al lado contrario.
float centradoLateral(float dt) {
  bool ambas = false;
  hayLateral = false;
  if (P.centrarParedes && distObjetivoMM >= P.centradoDistMin) {
    ultrasonicosLateralActualizar();
    hayLateral = ultrasonicosErrorLateral(errorLateral, ambas);
  }
  float objetivo = 0;   // sin paredes la inclinacion regresa a 0 (solo giroscopio)
  if (hayLateral) objetivo = constrain(P.centradoKp * errorLateral, -P.centradoMax, P.centradoMax);
  float paso = P.centradoTasa * dt;
  corrLateral += constrain(objetivo - corrLateral, -paso, paso);

  // Re-alineo: con pared a los dos lados y ya casi centrado, lo que todavia empuja
  // hacia un lado es el desfase del giroscopio contra el laberinto (escala, sesgo).
  // Se pasa al rumbo nominal, asi el siguiente giro ya sale alineado con las paredes.
  if (ambas && fabs(errorLateral) < REALINEO_ERROR_CM && fabs(objetivo) < P.centradoMax) {
    float nuevo = constrain(realineo + P.centradoKi * errorLateral * dt, -P.realineoMax, P.realineoMax);
    yawObjetivo += nuevo - realineo;
    realineo = nuevo;
  }
  return corrLateral;
}

// Avanzar recto una distancia
void pasoAvanzar(float dt) {
  float restante = distObjetivoMM - recorridoMM();
  if (fabs(restante) <= P.distTol || restante * distObjetivoMM < 0) {
    terminarMovimiento();
    return;
  }
  float vDeseada = constrain(P.distKp * fabs(restante), P.vMin, P.vMax);
  if (restante < 0) vDeseada = -vDeseada;
  if (fabs(vDeseada) > fabs(vBase)) {
    float paso = P.acel * dt;
    vBase += constrain(vDeseada - vBase, -paso, paso);
  } else {
    vBase = vDeseada;
  }
  float yawRef = yawObjetivo + centradoLateral(dt);
  float dv = pidRumbo.calcular(yawRef, yaw, dt) - P.rumboKd * velYaw;
  dv = constrain(dv, -P.rumboMax, P.rumboMax);
  lazoVelocidad(vBase - dv, vBase + dv, dt);
}

//Girar en su lugar
void pasoGirar(float dt) {
  float error = yawObjetivo - yaw;
  if (fabs(error) <= P.giroTol) {
    terminarMovimiento();
    return;
  }
  float dv = pidGiro.calcular(yawObjetivo, yaw, dt) - P.giroKd * velYaw;
  dv = constrain(dv, -P.giroMax, P.giroMax);
  if (fabs(dv) < P.giroMin) dv = (error > 0) ? P.giroMin : -P.giroMin;
  lazoVelocidad(-dv, dv, dt);
}

void iniciarAvance(float mm) {
  for (uint8_t i = 0; i < NUM_MOTORES; i++) pidVel[i].reiniciar();
  leerCuentas(cuentasInicio);
  distObjetivoMM = mm;
  vBase = 0;
  pidRumbo.reiniciar();
  corrLateral = 0;
  realineo = 0;
  hayLateral = false;
  ultrasonicosLateralReiniciar();
  // yawObjetivo NO se toca: se mantiene el rumbo nominal que dejó el último
  // giro, así el error de un giro se corrige al avanzar en vez de acumularse.
  tInicioMov = millis();
  movExcedido = false;
  estado = AVANZANDO;
  Serial.print(F("Avanzando ")); Serial.print(mm / 10.0f, 1); Serial.println(F(" cm"));
}

void iniciarGiro(float grados) {
  for (uint8_t i = 0; i < NUM_MOTORES; i++) pidVel[i].reiniciar();
  yawObjetivo += grados;   // relativo al rumbo NOMINAL, no al yaw medido
  pidGiro.reiniciar();
  tInicioMov = millis();
  movExcedido = false;
  estado = GIRANDO;
  Serial.print(F("Girando ")); Serial.print(grados); Serial.println(F(" grados"));
}

void detenerTodo() {
  frenarTodos();
  vBase = 0;
  estado = DETENIDO;
  Serial.println(F("Detenido."));
}

void movimientoIniciar() {
  iniciarMotores();
  frenarTodos();
  iniciarEncoders();
  aplicarParametros();

  imuOk = mpuIniciar();
  if (imuOk) {
    calibrarGiroscopio();
  } else {
    Serial.println(F("!! MPU6050 no responde: revisar SDA->20, SCL->21, VCC, GND."));
    Serial.println(F("   Avanzar en linea recta no va a corregir rumbo; girar no funcionara."));
  }
  controlReiniciarReloj();
}

uint32_t tControl = 0;
const uint32_t CONTROL_PERIODO_MS = 20;   // 50 Hz para toda la cascada

void controlReiniciarReloj() {
  tControl = millis();
  leerCuentas(cuentasPrevias);
}

bool controlActualizar() {
  actualizarIMU();   // a su propio ritmo, fuera del candado de 20 ms
  uint32_t ahora = millis();
  if (ahora - tControl < CONTROL_PERIODO_MS) return false;
  float dt = (ahora - tControl) / 1000.0f;
  tControl = ahora;

  if (medirVelocidades(dt) || estado != DETENIDO) tUltimoMov = ahora;

  switch (estado) {
    case DETENIDO:  frenarTodos();   break;
    case LIBRE:     soltarTodos();   break;
    case AVANZANDO: pasoAvanzar(dt); break;
    case GIRANDO:   pasoGirar(dt);   break;
  }

  // Seguridad: si un movimiento tarda demasiado (signo de encoder mal,
  // robot atorado), se detiene en lugar de seguir para siempre.
  if ((estado == AVANZANDO || estado == GIRANDO) && millis() - tInicioMov > TIEMPO_MAX_MOV_MS) {
    detenerTodo();
    movExcedido = true;
    Serial.println(F("!! Tiempo maximo excedido. Revisar signos con 'p'."));
  }
  return true;
}

void esperarMovimiento() {
  while (estado == AVANZANDO || estado == GIRANDO) controlActualizar();
}

void asentar(uint16_t ms) {
  uint32_t t0 = millis();
  while (millis() - t0 < ms) controlActualizar();
}
