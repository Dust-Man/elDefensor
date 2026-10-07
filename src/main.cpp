#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>   // Autor: Carlos Parrilla
#include "config.h"
#include "pid.h"

Parametros P;

enum Estado : uint8_t { DETENIDO, AVANZANDO, GIRANDO, LIBRE };
Estado estado = DETENIDO;

//Encoders
volatile long cuentas[NUM_MOTORES] = { 0, 0, 0, 0 }; //Pulsos por encoder de motor
int8_t signoEnc[NUM_MOTORES]; //Signos + - por motor

uint32_t tInicioMov = 0;   // para el tiempo máximo de seguridad
bool rutinaActiva = false;   // Autor: Carlos Parrilla -- la rutina 'r' esta corriendo

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
float sesgoGz = 0;     // sesgo del giroscopio en cuentas crudas
bool imuOk = false;

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
  Serial.println(F("Giroscopio listo."));
}

void actualizarIMU(float dt) {
  if (!imuOk || dt > 0.1f) return;
  int16_t gz;
  if (!mpuLeerGiroZ(gz)) return;
  float velGrados = (gz - sesgoGz) / GIRO_LSB_POR_GPS * YAW_SIGNO;
  yaw += velGrados * dt;
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

void aplicarParametros() {
  for (uint8_t i = 0; i < NUM_MOTORES; i++)
    pidVel[i].configurar(P.velKp, P.velKi, 0.0f, P.maxPwm, P.velIntMax);
  pidRumbo.configurar(P.rumboKp, P.rumboKi, P.rumboKd, P.rumboMax, P.rumboMax * 0.5f);
  pidGiro.configurar(P.giroKp, 0.0f, P.giroKd, P.giroMax, 0.0f);
}

//Retroalimentacion aplicada a los 4 PID, cada 20ms
void medirVelocidades(float dt) {
  long c[NUM_MOTORES];
  leerCuentas(c);
  for (uint8_t i = 0; i < NUM_MOTORES; i++) {
    float v = (c[i] - cuentasPrevias[i]) * MM_POR_TICK / dt;
    cuentasPrevias[i] = c[i];
    velMedida[i] += P.velFiltro * (v - velMedida[i]);
  }
}

void lazoVelocidad(float objIzq, float objDer, float dt) {
  velObjetivo[M_DI] = velObjetivo[M_TI] = objIzq;
  velObjetivo[M_DD] = velObjetivo[M_TD] = objDer;
  for (uint8_t i = 0; i < NUM_MOTORES; i++) {
    float obj = velObjetivo[i];
    float ff = 0;
    if (obj > 0.5f)       ff =   P.pwmArranque[i] + P.ffGanancia[i] * obj;
    else if (obj < -0.5f) ff = -(P.pwmArranque[i] - P.ffGanancia[i] * obj);
    float u = pidVel[i].calcular(obj, velMedida[i], dt, ff);
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
  float dv = pidRumbo.calcular(yawObjetivo, yaw, dt);
  lazoVelocidad(vBase - dv, vBase + dv, dt);
}

//Girar en su lugar
void pasoGirar(float dt) {
  float error = yawObjetivo - yaw;
  if (fabs(error) <= P.giroTol) {
    terminarMovimiento();
    return;
  }
  float dv = pidGiro.calcular(yawObjetivo, yaw, dt);
  if (fabs(dv) < P.giroMin) dv = (error > 0) ? P.giroMin : -P.giroMin;
  lazoVelocidad(-dv, dv, dt);
}

void iniciarAvance(float mm) {
  for (uint8_t i = 0; i < NUM_MOTORES; i++) pidVel[i].reiniciar();
  leerCuentas(cuentasInicio);
  distObjetivoMM = mm;
  vBase = 0;
  pidRumbo.reiniciar();
  // yawObjetivo NO se toca: se mantiene el rumbo nominal que dejó el último
  // giro, así el error de un giro se corrige al avanzar en vez de acumularse.
  tInicioMov = millis();
  estado = AVANZANDO;
  Serial.print(F("Avanzando ")); Serial.print(mm / 10.0f, 1); Serial.println(F(" cm"));
}

void iniciarGiro(float grados) {
  for (uint8_t i = 0; i < NUM_MOTORES; i++) pidVel[i].reiniciar();
  yawObjetivo += grados;   // relativo al rumbo NOMINAL, no al yaw medido
  pidGiro.reiniciar();
  tInicioMov = millis();
  estado = GIRANDO;
  Serial.print(F("Girando ")); Serial.print(grados); Serial.println(F(" grados"));
}

void detenerTodo() {
  frenarTodos();
  vBase = 0;
  estado = DETENIDO;
  rutinaActiva = false;   // Autor: Carlos Parrilla -- 's' o tiempo maximo cancelan la rutina
  Serial.println(F("Detenido."));
}

// ---- Pruebas en lazo abierto (sin PID). Robot LEVANTADO, ruedas al aire ----
const char* const NOMBRE_MOTOR[NUM_MOTORES] = { "DI", "TI", "DD", "TD" };

void imprimirCuentas() {
  long c[NUM_MOTORES];
  leerCuentas(c);
  for (uint8_t i = 0; i < NUM_MOTORES; i++) {
    Serial.print(NOMBRE_MOTOR[i]); Serial.print('='); Serial.print(c[i]); Serial.print(F("  "));
  }
  Serial.println();
}

// 'p': cada motor solo, PWM 120 por 0.8 s. Debe girar ADELANTE y contar POSITIVO.
void pruebaSignos() {
  soltarTodos();
  for (uint8_t i = 0; i < NUM_MOTORES; i++) {
    long a[NUM_MOTORES], b[NUM_MOTORES];
    leerCuentas(a);
    motorAplicar(i, 120);
    delay(800);
    motorAplicar(i, 0);
    delay(400);
    leerCuentas(b);
    long d = b[i] - a[i];
    Serial.print(NOMBRE_MOTOR[i]); Serial.print(F(": ")); Serial.print(d);
    if (labs(d) < 20)  Serial.println(F("  SIN SENAL de encoder (o el motor no giro)"));
    else if (d > 0)    Serial.println(F("  positivo -> OK si la rueda giro ADELANTE"));
    else               Serial.println(F("  NEGATIVO -> si giro adelante: cambiar ENCODER_INVERTIDO"));
  }
  Serial.println(F("Si una rueda giro hacia ATRAS: cambiar MOTOR_INVERTIDO de ese motor."));
}

// 'v<pwm>': los 4 motores al mismo PWM fijo; imprime la velocidad real en mm/s.
// Sirve para calcular ffGanancia y pwmArranque.
void pruebaVelocidad(int pwm) {
  pwm = constrain(pwm, 0, (int)P.maxPwm);
  for (uint8_t i = 0; i < NUM_MOTORES; i++) motorAplicar(i, pwm);
  delay(700);                                   // dejar que se estabilice
  long a[NUM_MOTORES], b[NUM_MOTORES];
  leerCuentas(a);
  delay(1000);
  leerCuentas(b);
  soltarTodos();
  Serial.print(F("PWM ")); Serial.print(pwm); Serial.print(F(" -> mm/s: "));
  for (uint8_t i = 0; i < NUM_MOTORES; i++) {
    Serial.print(NOMBRE_MOTOR[i]); Serial.print('=');
    Serial.print((b[i] - a[i]) * MM_POR_TICK, 0); Serial.print(F("  "));
  }
  Serial.println();
}

// 10 s de espera antes de moverse, para alcanzar a desconectar el cable.
// Autor: Carlos Parrilla
void cuentaRegresiva() {
  for (uint8_t s = 10; s > 0; s--) {
    Serial.print(F("Arrancando en ")); Serial.print(s); Serial.println(F("..."));
    delay(1000);
  }
}

// ---------------- Servos espejeados ----------------  Autor: Carlos Parrilla
// Angulo "logico" a: grados desde el cero. Servo 1 = CERO_1 + a, servo 2 = CERO_2 - a.
Servo servo1, servo2;
int servoAngulo = 0;          // angulo logico actual
int servoCrudo[2] = { SERVO_CERO_1, SERVO_CERO_2 };   // ultimo valor escrito a cada servo
bool servosConectados = false;

// Rango logico permitido para que ninguno de los dos se salga de 0..180
// Autor: Carlos Parrilla
int servoMin() { return max(-SERVO_CERO_1, SERVO_CERO_2 - 180); }
int servoMax() { return min(180 - SERVO_CERO_1, SERVO_CERO_2); }

// Attach con la configuracion probada; escribe antes para que no brinque a 90
// Autor: Carlos Parrilla
void servosConectar() {
  if (servosConectados) return;
  servo1.write(servoCrudo[0]);
  servo2.write(servoCrudo[1]);
  servo1.attach(PIN_SERVO_1, SERVO_PULSO_MIN, SERVO_PULSO_MAX);
  servo2.attach(PIN_SERVO_2, SERVO_PULSO_MIN, SERVO_PULSO_MAX);
  servosConectados = true;
}

// Sin pulso: los servos quedan sueltos (se pueden mover con la mano)
// Autor: Carlos Parrilla
void servosSoltar() {
  servo1.detach();
  servo2.detach();
  servosConectados = false;
  Serial.println(F("Servos sueltos (sin pulso)"));
}

// Autor: Carlos Parrilla
void servosImprimir() {
  Serial.print(F("Servos: angulo=")); Serial.print(servoAngulo);
  Serial.print(F("  S1=")); Serial.print(servoCrudo[0]);
  Serial.print(F("  S2=")); Serial.print(servoCrudo[1]);
  Serial.print(F("  rango=[")); Serial.print(servoMin());
  Serial.print(F(", ")); Serial.print(servoMax()); Serial.print(']');
  Serial.println(servosConectados ? F("  conectados") : F("  sueltos"));
}

// Mueve los dos servos espejeados al angulo logico a (grados desde el cero)
// Autor: Carlos Parrilla
void servosMover(int a, bool imprimir = true) {
  a = constrain(a, servoMin(), servoMax());
  servoAngulo = a;
  servoCrudo[0] = SERVO_CERO_1 + a;
  servoCrudo[1] = SERVO_CERO_2 - a;
  servosConectar();
  servo1.write(servoCrudo[0]);
  servo2.write(servoCrudo[1]);
  if (imprimir) servosImprimir();
}

// Mueve un solo servo a un angulo crudo 0..180 (para encontrar su cero)
// Autor: Carlos Parrilla
void servoIndividual(uint8_t n, int crudo) {
  crudo = constrain(crudo, 0, 180);
  servoCrudo[n] = crudo;
  servosConectar();
  (n == 0 ? servo1 : servo2).write(crudo);
  servosImprimir();
}

// Barrido lento de todo el rango, ida y vuelta, y regresa a donde estaba
// Autor: Carlos Parrilla
void servosBarrido() {
  int inicio = servoAngulo;
  Serial.println(F("Barrido de servos..."));
  for (int a = servoAngulo; a >= servoMin(); a--) { servosMover(a, false); delay(SERVO_MS_POR_GRADO); }
  for (int a = servoMin(); a <= servoMax(); a++)  { servosMover(a, false); delay(SERVO_MS_POR_GRADO); }
  for (int a = servoMax(); a >= inicio; a--)      { servosMover(a, false); delay(SERVO_MS_POR_GRADO); }
  servosImprimir();
}

// ---------------- Rutina 'r' ----------------  Autor: Carlos Parrilla
// Cada paso espera a que el anterior termine (estado DETENIDO) y una pausa corta.
// 'f' = avanzar cm, 'g' = girar grados (relativo), 'n' = girar al rumbo absoluto
// (0 = el rumbo con el que empezo la rutina, "norte"), 'z' = servos a angulo logico.
// Si los giros salen al lado contrario, cambiar el signo de RUTINA_GIRO.
constexpr float RUTINA_GIRO = 90.0f;
struct PasoRutina { char tipo; float valor; };
const PasoRutina RUTINA[] = {
  { 'f', 60 }, { 'g', RUTINA_GIRO },
  { 'f', 60 }, { 'g', RUTINA_GIRO },
  { 'f', 60 }, { 'g', RUTINA_GIRO },
  { 'f', 30 }, { 'n', 0 },            // regresar a ver al norte
  { 'f', 30 },
  { 'z', 0 }, { 'z', 180 },
};
// olanda
constexpr uint8_t RUTINA_PASOS = sizeof(RUTINA) / sizeof(RUTINA[0]);
constexpr uint32_t RUTINA_PAUSA_MS = 300;         // entre movimientos, para que se asiente
constexpr uint32_t RUTINA_PAUSA_SERVO_MS = 1000;  // para que el servo llegue

uint8_t rutinaIndice = 0;
float rutinaRumboInicial = 0;
uint32_t rutinaTEspera = 0, rutinaPausa = 0;

// Autor: Carlos Parrilla
void iniciarRutina() {
  cuentaRegresiva();
  rutinaRumboInicial = yawObjetivo;   // "norte" = hacia donde ve al arrancar
  rutinaIndice = 0;
  rutinaPausa = 0;
  rutinaTEspera = millis();
  rutinaActiva = true;
  Serial.println(F("Rutina iniciada (s para cancelar)"));
}

// Se llama en cada loop: cuando el robot termina un paso, lanza el siguiente.
// Autor: Carlos Parrilla
void actualizarRutina() {
  if (!rutinaActiva) return;
  if (estado == LIBRE) { rutinaActiva = false; Serial.println(F("Rutina cancelada")); return; }
  if (estado != DETENIDO) { rutinaTEspera = millis(); return; }   // sigue moviendose
  if (millis() - rutinaTEspera < rutinaPausa) return;

  if (rutinaIndice >= RUTINA_PASOS) {
    rutinaActiva = false;
    Serial.println(F("Rutina terminada"));
    return;
  }
  const PasoRutina& paso = RUTINA[rutinaIndice++];
  Serial.print(F("Rutina paso ")); Serial.print(rutinaIndice);
  Serial.print('/'); Serial.print(RUTINA_PASOS); Serial.print(F(": "));
  Serial.print(paso.tipo); Serial.println(paso.valor, 0);
  rutinaPausa = RUTINA_PAUSA_MS;
  switch (paso.tipo) {
    case 'f': iniciarAvance(paso.valor * 10.0f); break;
    case 'g': iniciarGiro(paso.valor); break;
    case 'n': {
      // giro mas corto para quedar viendo al rumbo pedido
      float delta = fmod(rutinaRumboInicial + paso.valor - yawObjetivo, 360.0f);
      if (delta > 180.0f)  delta -= 360.0f;
      if (delta < -180.0f) delta += 360.0f;
      iniciarGiro(delta);
      break;
    }
    case 'z': servosMover((int)paso.valor); rutinaPausa = RUTINA_PAUSA_SERVO_MS; break;
  }
  rutinaTEspera = millis();
}

// Tecla 'z' para depurar los servos. linea[1] es el subcomando.
// Autor: Carlos Parrilla
void comandoServos(char* linea, bool quieto) {
  char sub = linea[1];
  int valor = (sub == '\0') ? 0 : atoi(linea + 2);
  switch (sub) {
    case 'a': servosMover(valor); break;                  // za<g>  ir a g grados desde el cero
    case 'm': servosMover(servoAngulo + valor); break;    // zm<g>  mover g grados (+/-) desde donde esta
    case '0': servosMover(0); break;                      // z0     regresar al cero
    case '1': servoIndividual(0, valor); break;           // z1<g>  solo servo 1, angulo crudo
    case '2': servoIndividual(1, valor); break;           // z2<g>  solo servo 2, angulo crudo
    case 'b': if (quieto) servosBarrido(); break;         // zb     barrido completo
    case 's': servosSoltar(); break;                      // zs     soltar (detach)
    case 'i': servosImprimir(); break;                    // zi     ver estado
    default:
      Serial.println(F("Servos: za<g> ir a g | zm<+-g> mover g | z0 cero | z1<g>/z2<g> un servo crudo | zb barrido | zs soltar | zi info"));
  }
}

void procesarComando(char* linea) {
  char cmd = linea[0];
  float valor = atof(linea + 1);
  bool quieto = (estado == DETENIDO || estado == LIBRE);
  switch (cmd) {
    case 'f': cuentaRegresiva(); iniciarAvance(valor * 10.0f); break;   // valor en CENTIMETROS
    case 'g': cuentaRegresiva(); iniciarGiro(valor); break;
    case 's': detenerTodo(); break;
    case 'l': soltarTodos(); estado = LIBRE;
              Serial.println(F("Ruedas LIBRES (s para frenar)")); break;
    case 'e': imprimirCuentas(); break;
    case 'x': noInterrupts();
              for (uint8_t i = 0; i < NUM_MOTORES; i++) cuentas[i] = 0;
              interrupts();
              leerCuentas(cuentasPrevias);
              Serial.println(F("Encoders en 0")); break;
    case 'p': if (quieto) pruebaSignos(); break;
    case 'v': if (quieto) pruebaVelocidad((int)valor); break;
    case 'z': comandoServos(linea, quieto); break;   // Autor: Carlos Parrilla
    case 'r': if (quieto && !rutinaActiva) iniciarRutina(); break;   // Autor: Carlos Parrilla
    default:  Serial.println(F("Comandos: f<cm> g<grados> s | l libre, e encoders, x cero, p signos, v<pwm>, z servos, r rutina"));
  }
}

void leerSerial() {
  static char linea[24];
  static uint8_t largo = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (largo > 0) { linea[largo] = '\0'; procesarComando(linea); largo = 0; }
    } else if (largo < sizeof(linea) - 1) {
      linea[largo++] = c;
    }
  }
}

void imprimirDebug() {
   static uint32_t tUltimo = 0;
  if (estado == DETENIDO) return;          // no llenar el monitor si está quieto
  if (millis() - tUltimo < 250) return;
  tUltimo = millis();
  if (estado == LIBRE) { imprimirCuentas(); return; }
  const char* nombre = (estado == AVANZANDO) ? "AVANZANDO" : "GIRANDO";
  Serial.print(F("estado=")); Serial.print(nombre);
  Serial.print(F("  cm="));   Serial.print(recorridoMM() / 10.0f, 1);
  Serial.print(F("  vObj="));  Serial.print(velObjetivo[M_DI], 0);
  Serial.print(F("  yaw="));  Serial.print(yaw, 1);
  Serial.print(F("  vDI="));  Serial.print(velMedida[M_DI], 0);
  Serial.print(F("  vTI="));  Serial.print(velMedida[M_TI], 0);
  Serial.print(F("  vDD="));  Serial.print(velMedida[M_DD], 0);
  Serial.print(F("  vTD="));  Serial.print(velMedida[M_TD], 0);
  Serial.println();
}

uint32_t tControl = 0;
const uint32_t CONTROL_PERIODO_MS = 20;   // 50 Hz para toda la cascada

void setup() {
  Serial.begin(115200);
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

  Serial.println(F("Listo. Comandos: f<cm> g<grados> s | l libre, e encoders, x cero, p signos, v<pwm>, z servos, r rutina"));   // Autor: Carlos Parrilla (z servos, r rutina)
  tControl = millis();
}

void loop() {
  uint32_t ahora = millis();
  if (ahora - tControl >= CONTROL_PERIODO_MS) {
    float dt = (ahora - tControl) / 1000.0f;
    tControl = ahora;

    actualizarIMU(dt);
    medirVelocidades(dt);

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
      Serial.println(F("!! Tiempo maximo excedido. Revisar signos con 'p'."));
    }
}

  leerSerial();
  actualizarRutina();   // Autor: Carlos Parrilla
  imprimirDebug();
}

