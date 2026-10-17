// Main de debugging: comandos por Serial para probar y calibrar cada parte.
// Compilar/subir con el entorno [env:debug].
#include <Arduino.h>
#include "config.h"
#include "movimiento.h"
#include "servos.h"
#include "color.h"
#include "ultrasonicos.h"

bool rutinaActiva = false;

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
// Ya no alimenta ningun feedforward: sirve para verificar que las 4 ruedas
// responden parecido al mismo PWM.
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
  controlReiniciarReloj();
}

// ---------------- Color ----------------
bool colorContinuo = false;   // 'cc' imprime cada COLOR_PERIODO_MS
uint8_t colorActivo = 0;      // sensor al que aplican cb, cn, ca, cx  ('cs<n>' lo cambia)

// Tecla 'c' para depurar los sensores de color. linea[1] es el subcomando.
// c, cc, ci actuan sobre todos; cb, cn, ca, cx sobre el sensor activo (cs<n>).
void comandoColor(char* linea, bool quieto) {
  uint8_t s = colorActivo;
  switch (linea[1]) {
    case '\0': colorImprimirTodos(); break;                   // c    una lectura de cada sensor
    case 'c':  colorContinuo = !colorContinuo;                // cc   continuo on/off
               Serial.println(colorContinuo ? F("Color continuo ON") : F("Color continuo OFF")); break;
    case 'i':  colorIniciarTodos(); break;                    // ci   reintentar inicio
    case 'e':  escanearI2C(); break;                          // ce   escanear bus I2C
    case 's': {                                               // cs<n> elegir sensor activo
      int n = atoi(linea + 2);
      if (n >= 0 && n < NUM_SENSORES_COLOR) colorActivo = n;
      Serial.print(F("Sensor activo: S")); Serial.println(colorActivo);
      break;
    }
    case 'b':  if (quieto) colorCalibrarExtremo(s, true); break;    // cb   calibrar blanco
    case 'n':  if (quieto) colorCalibrarExtremo(s, false); break;   // cn   calibrar negro
    case 'a':  if (quieto) colorAprender(s, linea[2] ? atoi(linea + 2) : -1); break;   // ca<n> aprender color n
    case 'x':  colorCalDefecto(s); colorCalGuardar(s);       // cx   borrar calibracion
               Serial.print(F("Calibracion de color borrada en S")); Serial.println(s); break;
    default:
      Serial.print(F("Color (activo S")); Serial.print(colorActivo);
      Serial.println(F("): c lectura | cc continuo | ci reiniciar | ce escanear I2C | cs<n> sensor activo | cb blanco | cn negro | ca<n> aprender (ca = lista) | cx borrar"));
  }
}

void imprimirColorContinuo() {
  static uint32_t tUltimo = 0;
  if (!colorContinuo || millis() - tUltimo < COLOR_PERIODO_MS) return;
  tUltimo = millis();
  colorImprimirTodos();
}

// ---------------- Ultrasonicos ----------------
bool usContinuo = false;   // 'uu' imprime cada US_PERIODO_MS
bool usCentrado = false;   // 'uc' debug del centrado con paredes

// Tecla 'u' para depurar los ultrasonicos. linea[1] es el subcomando.
void comandoUltrasonicos(char* linea) {
  switch (linea[1]) {
    case '\0': ultrasonicosLeerTodos(); break;               // u    una lectura de cada sensor
    case 'u':  usContinuo = !usContinuo;                     // uu   continuo on/off
               Serial.println(usContinuo ? F("Ultrasonicos continuo ON") : F("Ultrasonicos continuo OFF")); break;
    case 'i':                                                // ui   pines de cada sensor
      for (uint8_t i = 0; i < NUM_ULTRASONICOS; i++) {
        Serial.print(NOMBRE_US[i]); Serial.print(F(": "));
        if (!ultrasonicoAsignado(i)) { Serial.println(F("sin pin (definir en config.h)")); continue; }
        Serial.print(F("TRIG=")); Serial.print(PIN_US_TRIG[i]);
        Serial.print(F(" ECHO=")); Serial.println(PIN_US_ECHO[i]);
      }
      break;
    case 'm': {                                              // um   mediana (lo que usa la pista A)
      float d[NUM_ULTRASONICOS];
      for (uint8_t i = 0; i < NUM_ULTRASONICOS; i++) d[i] = ultrasonicoMedianaCm(i, US_MUESTRAS);
      ultrasonicosImprimir(d);
      break;
    }
    case 'c':                                                // uc   debug del centrado on/off
      usCentrado = !usCentrado;
      if (usCentrado) {
        usContinuo = false;   // dispararia los mismos sensores y se cruzan los ecos
        if (estado != AVANZANDO) ultrasonicosLateralReiniciar();
      }
      Serial.println(usCentrado ? F("Debug de centrado ON") : F("Debug de centrado OFF")); break;
    default:
      Serial.println(F("Ultrasonicos: u lectura | uu continuo | um mediana | uc centrado | ui pines"));
  }
}

// ---------------- Ajuste en vivo 't' ----------------
// t = lista, t <nombre> = ver uno, t <nombre> <valor> = asignar (p.ej. "t rkp 5").
// Se pierde al reiniciar: pasar a config.h los valores que funcionen.
struct Ajuste { const char* nombre; float* valor; };
const Ajuste AJUSTES[] = {
  { "rkp", &P.rumboKp }, { "rki", &P.rumboKi }, { "rkd", &P.rumboKd }, { "rmax", &P.rumboMax },
  { "gkp", &P.giroKp }, { "gkd", &P.giroKd },
  { "ckp", &P.centradoKp }, { "cki", &P.centradoKi }, { "cmax", &P.centradoMax }, { "ctasa", &P.centradoTasa },
  { "vmax", &P.vMax }, { "asen", &P.asentarMs },
};

void comandoAjuste(char* linea) {
  char* nombre = linea + 1;
  while (*nombre == ' ') nombre++;
  char* valor = nombre;
  while (*valor && *valor != ' ') valor++;
  if (*valor) *valor++ = '\0';
  while (*valor == ' ') valor++;

  for (const Ajuste& a : AJUSTES) {
    if (*nombre && strcmp(nombre, a.nombre) != 0) continue;
    if (*valor) { *a.valor = atof(valor); aplicarParametros(); }
    Serial.print(a.nombre); Serial.print('='); Serial.println(*a.valor, 3);
    if (*nombre) return;
  }
  if (*nombre) { Serial.print(F("No existe: ")); Serial.println(nombre); }
}

// Continuo: un sensor por llamada (no bloquea el lazo con los tres juntos) e imprime
// cuando completa la vuelta. Cada sensor se dispara cada US_PERIODO_MS.
void imprimirUltrasonicoContinuo() {
  static uint32_t tUltimo = 0;
  static uint8_t i = 0;
  static float d[NUM_ULTRASONICOS];
  if (!usContinuo || millis() - tUltimo < US_PERIODO_MS / NUM_ULTRASONICOS) return;
  tUltimo = millis();
  d[i] = ultrasonicoLeerCm(i);
  if (++i < NUM_ULTRASONICOS) return;
  i = 0;
  ultrasonicosImprimir(d);
}

// ---------------- Centrado con paredes 'uc' ----------------
// Todo el camino del centrado: lectura cruda de cada lado y si el filtro la acepto
// (o por que no), el error lateral, la inclinacion de rumbo que pide (obj) y la que
// va aplicando (corr, limitada por ctasa).
// Quieto (s o l): dispara los laterales al ritmo del lazo, igual que al avanzar, sin
// mover motores. Con 'l' empujar el robot entre dos paredes y revisar que el signo
// sea el correcto. Avanzando: los dispara el centrado mismo y esta linea reemplaza
// a la de imprimirDebug.
constexpr float CENTRADO_TOL_CM = 0.5f;   // solo para mostrar "centrado"

void imprimirLadoCentrado(const __FlashStringHelper* nombre, uint8_t k) {
  float d = ultrasonicosLateralCrudo(k);
  Serial.print(nombre);
  if (d > 0) Serial.print(d, 1); else Serial.print(F("---"));
  Serial.print(' '); Serial.print(ultrasonicosLateralMotivo(k));
}

void imprimirCentrado(bool ciclo) {
  static uint32_t tUltimo = 0;
  if (!usCentrado || estado == GIRANDO) return;
  bool quieto = (estado == DETENIDO || estado == LIBRE);
  if (quieto && ciclo) ultrasonicosLateralActualizar();
  if (millis() - tUltimo < US_PERIODO_MS) return;
  tUltimo = millis();

  Serial.print(F("CEN"));
  if (!quieto) {
    Serial.print(F("  cm="));   Serial.print(recorridoMM() / 10.0f, 1);
    Serial.print(F("  yaw="));  Serial.print(yaw, 1);
    Serial.print(F("  yObj=")); Serial.print(yawObjetivo, 1);
  }
  imprimirLadoCentrado(F("  izq="), 0);
  imprimirLadoCentrado(F("  der="), 1);

  float e, dIzq, dDer;
  bool ambas;
  bool hay = ultrasonicosErrorLateral(e, ambas);
  if (hay) {
    ultrasonicosLateral(dIzq, dDer);
    Serial.print(F("  e=")); Serial.print(e, 1);
    Serial.print(ambas ? F(" (ambas)") : dIzq > 0 ? F(" (solo izq)") : F(" (solo der)"));
    Serial.print(F("  obj=")); Serial.print(constrain(P.centradoKp * e, -P.centradoMax, P.centradoMax), 1);
  } else {
    Serial.print(F("  sin pared: solo giroscopio"));
  }
  if (!quieto) { Serial.print(F("  corr=")); Serial.print(corrLateral, 1); }
  if (hay) {
    if (fabs(e) < CENTRADO_TOL_CM) Serial.print(F("  centrado"));
    else Serial.print(e > 0 ? F("  <- ir IZQ") : F("  ir DER ->"));
  }
  if (!P.centrarParedes) Serial.print(F("  [centrado OFF: w]"));
  Serial.println();
}

// ---------------- Centrado adelante/atras 'a' ----------------
// Misma cuenta que alinearEje() de las pistas: cada pared dice cuanto avanzar para
// quedar centrado y con las dos se promedia. a = frente+atras, af = solo frente
// (sin pared atras), at = solo atras (sin pared enfrente), am = medir sin moverse.
// Imprime lo que dice cada pared (aunque no se use), avanza y al terminar vuelve a
// medir para ver cuanto falto.
bool ejeFrente = false, ejeAtras = false;   // paredes que usa el centrado en curso
bool ejeVerificar = false;                  // al terminar el avance, volver a medir

// Imprime una pared y lo que dice. true si se usa y la lectura sirve
bool paredEje(const __FlashStringHelper* nombre, float cm, float adelante, float maxCm, bool usar) {
  Serial.print(F("  ")); Serial.print(nombre); Serial.print('=');
  if (cm <= 0) Serial.print(F("---"));
  else { Serial.print(cm, 1); Serial.print(F("cm -> avanzar ")); Serial.print(adelante, 1); }
  bool pared = cm > 0 && cm < US_PARED_CM;
  bool sirve = pared && fabs(adelante) <= maxCm;
  if (!usar)       Serial.println(F("  (no se usa)"));
  else if (!pared) Serial.println(F("  ignorada: sin pared"));
  else if (!sirve) Serial.println(F("  ignorada: lectura dudosa (error > maximo)"));
  else             Serial.println(F("  ok"));
  return usar && sirve;
}

// Lee enfrente y atras. adelante = cm a avanzar para quedar centrado; false si
// ninguna pared sirve
bool medirEje(float& adelante) {
  float frente = ultrasonicoMedianaCm(US_FRENTE, US_MUESTRAS);
  float atras = ultrasonicoMedianaCm(US_ATRAS, US_MUESTRAS);
  float aF = frente - US_FRENTE_OBJETIVO_CM;
  float aA = US_ATRAS_OBJETIVO_CM - atras;
  bool f = paredEje(F("frente"), frente, aF, ALINEAR_MAX_CM, ejeFrente);
  bool a = paredEje(F("atras"), atras, aA, ALINEAR_ATRAS_MAX_CM, ejeAtras);
  if (f && a)  adelante = (aF + aA) / 2;
  else if (f)  adelante = aF;
  else if (a)  adelante = aA;
  else return false;
  return true;
}

void iniciarCentradoEje(bool frente, bool atras) {
  ejeFrente = frente;
  ejeAtras = atras;
  Serial.print(F("Centrado "));
  Serial.println(frente && atras ? F("frente+atras") : frente ? F("solo frente") : F("solo atras"));
  asentar(P.asentarMs);
  float adelante;
  if (!medirEje(adelante)) { Serial.println(F("  Ninguna pared sirve: no se mueve")); return; }
  if (fabs(adelante) < 1.0f) {
    Serial.print(F("  Ya centrado (")); Serial.print(adelante, 1); Serial.println(F(" cm)"));
    return;
  }
  controlReiniciarReloj();   // las medianas bloquearon el lazo
  iniciarAvance(adelante * 10.0f);
  ejeVerificar = true;
}

void comandoEje(char* linea) {
  switch (linea[1]) {
    case '\0': iniciarCentradoEje(true, true); break;    // a    frente + atras
    case 'f':  iniciarCentradoEje(true, false); break;   // af   solo frente (sin pared atras)
    case 't':  iniciarCentradoEje(false, true); break;   // at   solo atras (sin pared enfrente)
    case 'm': {                                          // am   medir las dos sin moverse
      ejeFrente = ejeAtras = true;
      float adelante;
      if (medirEje(adelante)) { Serial.print(F("  Avanzaria ")); Serial.print(adelante, 1); Serial.println(F(" cm")); }
      else Serial.println(F("  Ninguna pared sirve"));
      break;
    }
    default:
      Serial.println(F("Centrado adelante/atras: a frente+atras | af solo frente | at solo atras | am medir"));
  }
}

// Se llama en cada loop: al terminar el avance del centrado vuelve a medir
void actualizarCentradoEje() {
  if (!ejeVerificar) return;
  if (estado == LIBRE) { ejeVerificar = false; return; }
  if (estado != DETENIDO) return;
  ejeVerificar = false;
  asentar(P.asentarMs);
  Serial.println(F("Despues del centrado:"));
  float adelante;
  if (medirEje(adelante)) { Serial.print(F("  Falta ")); Serial.print(adelante, 1); Serial.println(F(" cm")); }
  else Serial.println(F("  Ninguna pared sirve"));
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
  { 'z', 10 }, { 'z', 110 },
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
    case 's': detenerTodo(); rutinaActiva = false; ejeVerificar = false; break;   // Autor: Carlos Parrilla -- 's' cancela la rutina
    case 'l': soltarTodos(); estado = LIBRE;
              Serial.println(F("Ruedas LIBRES (s para frenar)")); break;
    case 'e': imprimirCuentas(); break;
    case 'x': encodersEnCero();
              Serial.println(F("Encoders en 0")); break;
    case 'p': if (quieto) pruebaSignos(); break;
    case 'v': if (quieto) pruebaVelocidad((int)valor); break;
    case 'z': comandoServos(linea, quieto); break;   // Autor: Carlos Parrilla
    case 'r': if (quieto && !rutinaActiva) iniciarRutina(); break;   // Autor: Carlos Parrilla
    case 'c': comandoColor(linea, quieto); break;
    case 'u': comandoUltrasonicos(linea); break;
    case 'w': P.centrarParedes = !P.centrarParedes;
              Serial.println(P.centrarParedes ? F("Centrado con paredes ON") : F("Centrado con paredes OFF")); break;
    case 'a': if (quieto && !rutinaActiva) comandoEje(linea); break;
    case 't': comandoAjuste(linea); break;
    default:  Serial.println(F("Comandos: f<cm> g<grados> s | l libre, e encoders, x cero, p signos, v<pwm>, z servos, r rutina, c color, u ultrasonicos, w centrado, a centrar frente/atras, t ajustes"));
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
  if (usCentrado && estado != GIRANDO) return;   // lo imprime imprimirCentrado()
  if (millis() - tUltimo < 250) return;
  tUltimo = millis();
  if (estado == LIBRE) {   // yaw para calibrar GIRO_ESCALA girando a mano
    Serial.print(F("yaw=")); Serial.print(yaw, 1); Serial.print(F("  "));
    imprimirCuentas();
    return;
  }
  const char* nombre = (estado == AVANZANDO) ? "AVANZANDO" : "GIRANDO";
  Serial.print(F("estado=")); Serial.print(nombre);
  Serial.print(F("  cm="));   Serial.print(recorridoMM() / 10.0f, 1);
  Serial.print(F("  vObj="));  Serial.print(velObjetivo[M_DI], 0);
  Serial.print(F("  yaw="));  Serial.print(yaw, 1);
  Serial.print(F("  ref="));  Serial.print(yawObjetivo + corrLateral, 1);
  Serial.print(F("  w="));    Serial.print(velYaw, 0);
  if (estado == AVANZANDO) {
    float dIzq, dDer;
    ultrasonicosLateral(dIzq, dDer);
    Serial.print(F("  dI=")); if (dIzq > 0) Serial.print(dIzq, 1); else Serial.print('-');
    Serial.print(F("  dD=")); if (dDer > 0) Serial.print(dDer, 1); else Serial.print('-');
    Serial.print(F("  e="));  if (hayLateral) Serial.print(errorLateral, 1); else Serial.print('-');
    Serial.print(F("  corr=")); Serial.print(corrLateral, 1);
  }
  Serial.print(F("  vDI="));  Serial.print(velMedida[M_DI], 0);
  Serial.print(F("  vTI="));  Serial.print(velMedida[M_TI], 0);
  Serial.print(F("  vDD="));  Serial.print(velMedida[M_DD], 0);
  Serial.print(F("  vTD="));  Serial.print(velMedida[M_TD], 0);
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  movimientoIniciar();
  colorCalCargar();
  colorIniciarTodos();
  ultrasonicosIniciar();

  Serial.println(F("Listo. Comandos: f<cm> g<grados> s | l libre, e encoders, x cero, p signos, v<pwm>, z servos, r rutina, c color, u ultrasonicos, w centrado, a centrar frente/atras, t ajustes"));   // Autor: Carlos Parrilla (z servos, r rutina)
  controlReiniciarReloj();
}

void loop() {
  bool ciclo = controlActualizar();
  if (movExcedido) {   // el tiempo maximo tambien cancela la rutina
    movExcedido = false;
    rutinaActiva = false;
  }

  leerSerial();
  actualizarRutina();   // Autor: Carlos Parrilla
  actualizarCentradoEje();
  imprimirDebug();
  imprimirColorContinuo();
  imprimirUltrasonicoContinuo();
  imprimirCentrado(ciclo);
}
