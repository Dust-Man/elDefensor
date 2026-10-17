// Pista B: niveles. Compilar/subir con el entorno [env:pistaB].
// Seccion 1 (pelota en el 3x3) pendiente: depende de la vision. Por ahora se
// arranca en el checkpoint de las lineas (PISTA_B_SECCION_INICIAL = 2) o en el
// de los colores (= 3). Cada seccion es su propia funcion para que despues los
// botones de lack of progress puedan arrancar en cualquiera.
#include <Arduino.h>
#include "config.h"
#include "movimiento.h"
#include "color.h"
#include "ultrasonicos.h"
#include "arranque.h"

// Sensores de color: S0 izquierdo, S1 derecho
constexpr uint8_t S_IZQ = 0;
constexpr uint8_t S_DER = 1;

// Lados (y filas de la seccion 2): + = izquierda
constexpr int8_t IZQ = 1;
constexpr int8_t DER = -1;

constexpr float MEDIA = CELDA_MM / 2;

// ---------------------------------------------------------------------------
// Movimientos
// ---------------------------------------------------------------------------
// direcciones respecto a la pista, como en pista A:
// 0 = hacia donde ve el robot al arrancar ("arriba" en la seccion 3),
// 1 = derecha, 2 = abajo, 3 = izquierda
int8_t heading = 0;

void girarHacia(int8_t dir) {
  int8_t diff = (dir - heading + 4) % 4;
  if (diff == 0) return;
  float grados = GIRO_DERECHA_GRADOS;      // diff 1: derecha
  if (diff == 2) grados = 2 * GIRO_DERECHA_GRADOS;
  if (diff == 3) grados = -GIRO_DERECHA_GRADOS;   // izquierda
  iniciarGiro(grados);
  esperarMovimiento();
  heading = dir;
}

void avanzarMM(float mm) {
  iniciarAvance(mm);
  esperarMovimiento();
}

// ---------------------------------------------------------------------------
// Seccion 2: lineas blancas
// ---------------------------------------------------------------------------
// 2 filas x B_COLUMNAS de verde. IN a la izquierda de la fila de abajo, F a la
// derecha de la fila de arriba. Las lineas cruzan una fila y dejan libre la otra
// (hueco de 30 cm). Posicion por odometria:
//   posX   = mm a lo largo de la pista, 0 = centro de IN
//   posLat = mm de lado, 0 = costura entre las dos filas, + = izquierda (arriba)
float posX = 0;
float posLat = 0;

// Centrado con las paredes de los lados de la seccion 2. Con el robot al centro
// de una fila, la pared de ese lado queda como en el laberinto
// (US_LATERAL_OBJ_*); por cada mm que se aleja de ella, la lectura crece lo mismo.
void centrarSegunFila() {
  ultrasonicosLateralDesfase((MEDIA - posLat) / 10.0f, (MEDIA + posLat) / 10.0f);
}

struct Linea {
  float x;       // donde la vio el sensor
  int8_t fila;   // IZQ o DER: la fila que tapa
  bool izq, der; // que sensores la vieron
};

// Avanza (rumbo 0) hasta xObjetivo vigilando los dos sensores. true si llego;
// false si vio una linea blanca y freno antes de pisarla.
bool avanzarVigilando(float xObjetivo, Linea& linea) {
  float x0 = posX;
  centrarSegunFila();
  uint8_t racha[2] = { 0, 0 };
  float xPrimera[2] = { 0, 0 };   // x del sensor en la primera lectura blanca de la racha
  bool vista = false;
  uint32_t tColor = 0;
  uint32_t tLog = 0;

  iniciarAvance(xObjetivo - x0);
  while (estado == AVANZANDO) {
    controlActualizar();
    // Depuracion: que tanto lo esta moviendo de lado el centrado con paredes
    if (millis() - tLog >= 300) {
      tLog = millis();
      Serial.print(F("  x=")); Serial.print((x0 + recorridoMM()) / 10.0f, 1);
      Serial.print(F(" lat=")); Serial.print(posLat / 10.0f, 1);
      if (hayLateral) {
        Serial.print(F(" | pared err=")); Serial.print(errorLateral, 1);
        Serial.print(F("cm incl=")); Serial.print(corrLateral, 1); Serial.println(F("g"));
      } else {
        Serial.print(F(" | sin pared, incl=")); Serial.print(corrLateral, 1); Serial.println(F("g"));
      }
    }
    // Una lectura por integracion del sensor: las de en medio repiten el mismo dato
    if (millis() - tColor < COLOR_INTEGRACION_MS) continue;
    tColor = millis();
    float xSensor = x0 + recorridoMM() + COLOR_OFFSET_MM;
    // La franja blanca entre IN y el verde no es linea
    if (fabs(xSensor - MEDIA) < B_MARGEN_SEPARACION_MM) {
      racha[S_IZQ] = racha[S_DER] = 0;
      continue;
    }
    for (uint8_t s = 0; s < 2; s++) {
      int8_t c;
      bool blanco;
      if (!colorDetectar(s, c, &blanco)) continue;
      if (!blanco) { racha[s] = 0; continue; }
      if (racha[s] == 0) xPrimera[s] = xSensor;
      if (racha[s] < 255) racha[s]++;
    }
    if (racha[S_IZQ] >= B_LECTURAS_BLANCO || racha[S_DER] >= B_LECTURAS_BLANCO) {
      detenerTodo();
      vista = true;
    }
  }
  asentar(P.asentarMs);   // que termine de frenar antes de tomar la posicion
  posX = x0 + recorridoMM();
  if (!vista) return true;

  bool izq = racha[S_IZQ] >= B_LECTURAS_BLANCO;
  bool der = racha[S_DER] >= B_LECTURAS_BLANCO;
  linea.izq = izq;
  linea.der = der;
  if (izq && der) {
    // Las dos: si ya va en una fila, la linea es de esa fila; centrado, la que la vio primero
    if (posLat > MEDIA / 2)       linea.fila = IZQ;
    else if (posLat < -MEDIA / 2) linea.fila = DER;
    else                          linea.fila = (xPrimera[S_IZQ] <= xPrimera[S_DER]) ? IZQ : DER;
    linea.x = min(xPrimera[S_IZQ], xPrimera[S_DER]);
  } else {
    linea.fila = izq ? IZQ : DER;
    linea.x = izq ? xPrimera[S_IZQ] : xPrimera[S_DER];
  }
  Serial.print(F("Linea en x=")); Serial.print(linea.x / 10.0f, 1);
  Serial.print(F(" cm, fila ")); Serial.print(linea.fila == IZQ ? F("izquierda") : F("derecha"));
  Serial.print(F(" (sensores: izq ")); Serial.print(izq ? F("blanco") : F("verde"));
  Serial.print(F(", der ")); Serial.print(der ? F("blanco") : F("verde"));
  Serial.print(F("; lat=")); Serial.print(posLat / 10.0f, 1); Serial.println(F(" cm)"));
  return false;
}

// Sin vigilar (retroceder o entrar a F)
void irAX(float xObjetivo) {
  float x0 = posX;
  centrarSegunFila();
  avanzarMM(xObjetivo - x0);
  posX = x0 + recorridoMM();
}

// Lo que debe leer el ultrasonico de enfrente con el robot en latObjetivo viendo
// hacia la pared de ese lado (las paredes estan en las orillas de las 2 filas):
// al centro de la fila de esa pared es US_FRENTE_OBJETIVO_CM, como en el laberinto.
float frenteEsperadoCm(float latObjetivo, int8_t lado) {
  return US_FRENTE_OBJETIVO_CM + (MEDIA - lado * latObjetivo) / 10.0f;
}

// De lado: gira 90 hacia alla, avanza y regresa al rumbo 0. Moverse de lado no
// cruza lineas (van en la misma direccion), pero el giro en su lugar solo es
// seguro en la mitad de una columna.
// Va viendo de frente a la pared de ese lado: frena si se acerca de mas (la
// odometria se paso) y al final se acomoda a la distancia exacta, que tambien
// corrige posLat. Sin pared enfrente queda solo la odometria.
void moverLateral(float latObjetivo) {
  float d = latObjetivo - posLat;
  if (fabs(d) < 5) return;
  int8_t lado = d > 0 ? IZQ : DER;
  girarHacia(lado == IZQ ? 3 : 1);
  float esperado = frenteEsperadoCm(latObjetivo, lado);
  Serial.print(F("De lado a lat=")); Serial.print(latObjetivo / 10.0f, 1);
  Serial.print(F(" cm: espero leer enfrente ")); Serial.print(esperado, 1); Serial.println(F(" cm"));

  // De lado, los ultrasonicos laterales ven a lo largo de la pista: no centran
  bool centrar = P.centrarParedes;
  P.centrarParedes = false;
  iniciarAvance(fabs(d));
  uint8_t cerca = 0;
  uint32_t tUs = 0;
  while (estado == AVANZANDO) {
    controlActualizar();
    if (millis() - tUs < US_PAUSA_MS) continue;   // que se apague el eco anterior
    tUs = millis();
    float cm = ultrasonicoLeerCm(US_FRENTE, esperado + ALINEAR_MAX_CM);   // timeout corto
    // Dos lecturas seguidas: una sola puede ser ruido y lo dejaria corto
    cerca = (cm > US_LATERAL_MIN_CM && cm <= esperado) ? cerca + 1 : 0;
    if (cerca >= 2) {
      detenerTodo();
      Serial.print(F("  Pared enfrente a ")); Serial.print(cm, 1);
      Serial.print(F(" cm: freno con ")); Serial.print(recorridoMM() / 10.0f, 1);
      Serial.print(F(" de ")); Serial.print(fabs(d) / 10.0f, 1); Serial.println(F(" cm"));
    }
  }
  P.centrarParedes = centrar;

  asentar(P.asentarMs);
  float cm = ultrasonicoMedianaCm(US_FRENTE, US_MUESTRAS);
  float error = cm - esperado;
  Serial.print(F("  Enfrente al final: "));
  if (cm > 0) { Serial.print(cm, 1); Serial.print(F(" cm")); } else Serial.print(F("sin eco"));
  bool ajusta = cm > 0 && fabs(error) >= 1.0f && fabs(error) <= ALINEAR_MAX_CM;
  if (ajusta) { Serial.print(F(", ajusto ")); Serial.print(error, 1); Serial.print(F(" cm")); }
  Serial.println();
  if (ajusta) avanzarMM(error * 10.0f);
  girarHacia(0);
  posLat = latObjetivo;
}

// Regresa a la mitad de la columna anterior, se pasa a la fila libre (el hueco
// es de 30 cm: el robot va centrado en el) y cruza. Si vuelve a ver blanco:
//  - un solo sensor: quedo cargado hacia la linea (o la linea es mas larga): se
//    recorre B_PASO_AJUSTE_MM alejandose de ese sensor y lo vuelve a intentar.
//  - los dos: la linea tapa toda esa fila, se pasa a la otra.
bool esquivar(const Linea& primera, float xUltima) {
  const float latMax = MEDIA + B_LAT_EXTRA_MM;
  float xLinea = primera.x;
  float latObj = -primera.fila * MEDIA;   // centro de la fila libre
  for (uint8_t intento = 1; intento <= B_INTENTOS_LINEA; intento++) {
    Serial.print(F("Esquivar, intento ")); Serial.print(intento);
    Serial.print(F(": lat=")); Serial.print(latObj / 10.0f, 1); Serial.println(F(" cm"));
    irAX(xLinea - MEDIA);
    moverLateral(latObj);
    Linea otra;
    if (avanzarVigilando(min(xLinea + MEDIA, xUltima), otra)) {
      // La siguiente linea iria una columna mas adelante. Si ahi ya se acabo el
      // verde, esta era la ultima: directo a la fila de F, sin pasar por la costura.
      const float xFinVerde = CELDA_MM * B_COLUMNAS + MEDIA;
      bool ultima = xLinea + CELDA_MM > xFinVerde - B_MARGEN_SEPARACION_MM;
      moverLateral(ultima ? B_FIN_LADO * MEDIA : 0);
      return true;
    }
    xLinea = otra.x;
    float nueva;
    if (otra.izq && otra.der) nueva = latObj > 0 ? -MEDIA : MEDIA;
    else nueva = constrain(latObj + (otra.izq ? -B_PASO_AJUSTE_MM : B_PASO_AJUSTE_MM), -latMax, latMax);
    // Ya no se puede recorrer mas hacia la pared: la otra fila
    if (fabs(nueva - latObj) < 1) nueva = latObj > 0 ? -MEDIA : MEDIA;
    latObj = nueva;
  }
  Serial.println(F("!! No encontre el hueco de la linea"));
  return false;
}

bool seccion2() {
  Serial.println(F("== Seccion 2: lineas blancas =="));
  heading = 0;
  posX = 0;
  posLat = DER * MEDIA;   // IN: centrado en la fila de abajo
  float vMaxNormal = P.vMax;
  P.vMax = B_VEL_LINEAS;

  const float xUltima = CELDA_MM * B_COLUMNAS;   // centro de la ultima columna
  bool centrado = false;
  bool ok = true;
  Linea linea;
  while (ok && posX < xUltima - 10) {
    // Primero al centro de la columna 1 para centrarse entre las dos filas
    float objetivo = centrado ? xUltima : CELDA_MM;
    if (avanzarVigilando(objetivo, linea)) {
      if (!centrado) { moverLateral(0); centrado = true; }
      continue;
    }
    ok = esquivar(linea, xUltima);
    centrado = true;
  }

  if (ok) {
    // A la fila de F y al centro de F
    moverLateral(B_FIN_LADO * MEDIA);
    irAX(CELDA_MM * (B_COLUMNAS + 1));
    Serial.println(F("Checkpoint F"));
  }
  P.vMax = vMaxNormal;
  return ok;
}

// ---------------------------------------------------------------------------
// Seccion 3: avanzar con colores
// ---------------------------------------------------------------------------
// Color de la celda donde esta parado: los dos sensores votan. Como van
// COLOR_OFFSET_MM adelante del centro, siguen dentro de la misma celda.
int8_t leerColorCelda() {
  uint8_t votos[NUM_COLORES] = { 0 };
  for (uint8_t k = 0; k < B_S3_LECTURAS; k++) {
    asentar(COLOR_INTEGRACION_MS);   // lectura nueva; el control sigue frenando
    for (uint8_t s = 0; s < 2; s++) {
      int8_t c;
      if (colorDetectar(s, c) && c >= 0) votos[c]++;
    }
  }
  int8_t mejor = COLOR_NINGUNO;
  uint8_t maxVotos = 0;
  for (int8_t c = 0; c < NUM_COLORES; c++) {
    if (votos[c] > maxVotos) { maxVotos = votos[c]; mejor = c; }
  }
  // Por lo menos la mitad de todas las lecturas
  return maxVotos >= B_S3_LECTURAS ? mejor : COLOR_NINGUNO;
}

// Hacia donde manda cada color (respecto a la pista), -1 si no es de flecha
int8_t direccionDeColor(int8_t c) {
  switch (c) {
    case COLOR_NARANJA:  return 0;   // arriba
    case COLOR_CYAN:     return 1;   // derecha
    case COLOR_ROSA:     return 2;   // abajo
    case COLOR_AMARILLO: return 3;   // izquierda
    default:             return -1;
  }
}

bool seccion3() {
  Serial.println(F("== Seccion 3: colores =="));
  heading = 0;   // "arriba" = hacia donde ve al salir de F
  // Celdas como las del laberinto: centra solo con paredes pegadas a la celda
  ultrasonicosLateralDesfase(0, 0);
  for (uint8_t paso = 0; paso < B_S3_MAX_PASOS; paso++) {
    avanzarMM(CELDA_MM);
    asentar(P.asentarMs);
    int8_t c = leerColorCelda();
    if (c == COLOR_NINGUNO) {
      // Quedo en la orilla o el sensor vio la franja blanca: un poco mas adelante
      avanzarMM(30);
      c = leerColorCelda();
      avanzarMM(-30);
    }
    Serial.print(F("Celda ")); Serial.print(paso + 1); Serial.print(F(": "));
    Serial.println(c >= 0 ? NOMBRE_COLOR[c] : "?");

    if (c == COLOR_VERDE) {
      Serial.println(F("FIN"));
      return true;
    }
    int8_t dir = direccionDeColor(c);
    if (dir < 0) {
      Serial.println(F("!! Color sin direccion, me detengo"));
      return false;
    }
    girarHacia(dir);
  }
  Serial.println(F("!! Demasiadas celdas sin llegar al FIN"));
  return false;
}

// ---------------------------------------------------------------------------
// Pista
// ---------------------------------------------------------------------------
void correrPistaB() {
  uint32_t t0 = millis();
  bool ok = true;
  if (PISTA_B_SECCION_INICIAL <= 2) ok = seccion2();
  if (ok) ok = seccion3();
  Serial.print(ok ? F("Pista B terminada en ") : F("Pista B detenida a los "));
  Serial.print((millis() - t0) / 1000); Serial.println(F(" s"));
}

void setup() {
  Serial.begin(115200);
  movimientoIniciar();   // calibra el giroscopio: robot QUIETO
  P.centrarParedes = B_CENTRAR_PAREDES;
  colorCalCargar();      // calibracion hecha con el main de debug (EEPROM)
  colorIniciarTodos();
  if (!colorOk[S_IZQ] || !colorOk[S_DER])
    Serial.println(F("!! Pista B necesita los dos sensores de color"));
  ultrasonicosIniciar();
  esperarArranque();
  correrPistaB();
}

void loop() {
  controlActualizar();   // DETENIDO: mantiene el freno
}
