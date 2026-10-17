// Pista B: niveles. Compilar/subir con el entorno [env:pistaB].
// La pelota no se atrapa (no hay mecanismo confiable): en la seccion 1 el robot
// solo explora el 3x3 hasta el checkpoint de las lineas. PISTA_B_SECCION_INICIAL dice
// en que seccion arranca (1, 2 o 3); cada seccion es su propia funcion para que
// despues los botones de lack of progress puedan arrancar en cualquiera.
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
// Seccion 1 sin pelota: explorar el 3x3 hasta el checkpoint IN
// ---------------------------------------------------------------------------
// La casilla de INICIO cambia por ronda y no se puede pre-mapear. Lo fijo es que
// IN esta pegado al 3x3, a la derecha de su casilla central. Se explora como el
// laberinto de la pista A: cuadricula virtual con el inicio al centro (el 3x3
// cabe empiece donde empiece), paredes con los ultrasonicos y siempre hacia la
// celda sin visitar mas cercana. Cada avance vigila el piso: a IN solo se entra
// desde el 3x3 y avanzando hacia las lineas, asi que al ver su rojo con los DOS
// sensores ya va entrando, en la direccion de la seccion 2 (la pelota naranja
// es angosta: a lo mas la ve uno).
constexpr int8_t S1_N = 9;
constexpr int8_t S1_INICIO = S1_N / 2;
const int8_t dx[4] = { 0, 1, 0, -1 };
const int8_t dy[4] = { 1, 0, -1, 0 };

struct Celda { int8_t x, y; };

bool s1Pared[S1_N][S1_N][4];
bool s1Visitada[S1_N][S1_N];
int8_t s1Dist[S1_N][S1_N];
int8_t s1x = S1_INICIO, s1y = S1_INICIO;

bool s1Dentro(int8_t x, int8_t y) {
  return x >= 0 && x < S1_N && y >= 0 && y < S1_N;
}

void s1AgregarPared(int8_t x, int8_t y, int8_t dir) {
  s1Pared[x][y][dir] = true;
  int8_t nx = x + dx[dir];
  int8_t ny = y + dy[dir];
  if (s1Dentro(nx, ny)) s1Pared[nx][ny][(dir + 2) % 4] = true;
}

bool s1Accesible(int8_t x, int8_t y, int8_t dir) {
  return !s1Pared[x][y][dir] && s1Dentro(x + dx[dir], y + dy[dir]);
}

// Distancia de cada celda a la celda sin visitar mas cercana
void s1FloodFill() {
  static Celda cola[S1_N * S1_N];
  uint8_t ini = 0, fin = 0;
  for (int8_t x = 0; x < S1_N; x++)
    for (int8_t y = 0; y < S1_N; y++) {
      s1Dist[x][y] = s1Visitada[x][y] ? -1 : 0;
      if (!s1Visitada[x][y]) cola[fin++] = { x, y };
    }
  while (ini < fin) {
    Celda act = cola[ini++];
    for (int8_t dir = 0; dir < 4; dir++) {
      if (!s1Accesible(act.x, act.y, dir)) continue;
      int8_t nx = act.x + dx[dir];
      int8_t ny = act.y + dy[dir];
      if (s1Dist[nx][ny] != -1) continue;
      s1Dist[nx][ny] = s1Dist[act.x][act.y] + 1;
      cola[fin++] = { nx, ny };
    }
  }
}

// Prefiere seguir derecho cuando hay empate para ahorrar giros
int8_t s1MejorDireccion() {
  int8_t mejor = -1;
  int8_t menor = 127;
  for (int8_t k = 0; k < 4; k++) {
    int8_t dir = (heading + k) % 4;
    if (!s1Accesible(s1x, s1y, dir)) continue;
    int8_t d = s1Dist[s1x + dx[dir]][s1y + dy[dir]];
    if (d >= 0 && d < menor) { menor = d; mejor = dir; }
  }
  return mejor;
}

// (Igual que en la pista A)
bool hayPared(float cm) {
  return cm > 0 && cm < US_PARED_CM;   // sin eco = no hay pared cerca
}

// (Igual que en la pista A) Si hay pared enfrente, se acerca o aleja hasta
// quedar centrado en la celda: el error de los encoders no se acumula.
void alinearConFrente(float cm) {
  if (!ALINEAR_CON_PARED || !hayPared(cm)) return;
  float error = cm - US_FRENTE_OBJETIVO_CM;
  if (fabs(error) < 1.0f || fabs(error) > ALINEAR_MAX_CM) return;
  avanzarMM(error * 10.0f);
}

void s1LeerParedes() {
  asentar(P.asentarMs);
  float frente = ultrasonicoMedianaCm(US_FRENTE, US_MUESTRAS);
  float izq = ultrasonicoMedianaCm(US_IZQ, US_MUESTRAS);
  float der = ultrasonicoMedianaCm(US_DER, US_MUESTRAS);
  if (hayPared(frente)) s1AgregarPared(s1x, s1y, heading);
  if (hayPared(der))    s1AgregarPared(s1x, s1y, (heading + 1) % 4);
  if (hayPared(izq))    s1AgregarPared(s1x, s1y, (heading + 3) % 4);
  alinearConFrente(frente);
}

// Gira hacia dir con las mismas protecciones de la pista A. false si ahi hay
// pared y hay que volver a planear.
bool s1GirarHacia(int8_t dir) {
  int8_t diff = (dir - heading + 4) % 4;
  if (diff == 0) return true;
  // Antes de un giro de 90 confirma ese lado: si se escapo una pared, el robot
  // giraria, la veria y tendria que girar otros 90 (un 180 en dos partes)
  if (diff != 2 && hayPared(ultrasonicoMedianaCm(diff == 1 ? US_DER : US_IZQ, US_MUESTRAS))) {
    s1AgregarPared(s1x, s1y, dir);
    return false;
  }
  girarHacia(dir);
  // La pared que ahora queda enfrente pudo no haberse leido (la de atras del inicio)
  asentar(P.asentarMs);
  float frente = ultrasonicoMedianaCm(US_FRENTE, US_MUESTRAS);
  if (!hayPared(frente)) return true;
  s1AgregarPared(s1x, s1y, dir);
  alinearConFrente(frente);
  return false;
}

enum S1Avance : uint8_t { S1_LLEGO, S1_ROJO, S1_ATORADO };

// Avanza una celda vigilando el rojo de IN con los dos sensores (cada uno
// COLOR_LECTURAS_MIN lecturas seguidas). desdeRojo = lo que avanzo desde la
// primera lectura roja.
S1Avance s1AvanzarCelda(float& desdeRojo) {
  uint8_t racha[2] = { 0, 0 };
  float sPrimera[2] = { 0, 0 };
  bool rojo = false;
  uint32_t tColor = 0;

  iniciarAvance(CELDA_MM);
  while (estado == AVANZANDO) {
    controlActualizar();
    if (millis() - tColor < COLOR_INTEGRACION_MS) continue;
    tColor = millis();
    float s = recorridoMM();
    for (uint8_t k = 0; k < 2; k++) {
      int8_t c;
      if (!colorDetectar(k, c)) continue;
      if (c != COLOR_ROJO) { racha[k] = 0; continue; }
      if (racha[k] == 0) sPrimera[k] = s;
      if (racha[k] < 255) racha[k]++;
    }
    if (racha[S_IZQ] >= COLOR_LECTURAS_MIN && racha[S_DER] >= COLOR_LECTURAS_MIN) {
      detenerTodo();
      rojo = true;
    }
  }
  asentar(P.asentarMs);
  if (rojo) {
    desdeRojo = recorridoMM() - min(sPrimera[S_IZQ], sPrimera[S_DER]);
    return S1_ROJO;
  }
  // Atorado (p. ej. empujando la pelota contra la pared): si no paso la mitad, regresa
  if (movExcedido) {
    float hecho = recorridoMM();
    if (hecho < MEDIA) {
      avanzarMM(-hecho);
      return S1_ATORADO;
    }
  }
  return S1_LLEGO;
}

// xFinal: donde queda el centro del robot respecto al centro de IN
bool seccion1(float& xFinal) {
  Serial.println(F("== Seccion 1: buscar el checkpoint IN (sin pelota) =="));
  ultrasonicosLateralDesfase(0, 0);   // celdas como las del laberinto
  heading = 0;
  s1Visitada[s1x][s1y] = true;

  for (uint8_t paso = 0; paso < B_S1_MAX_PASOS; paso++) {
    s1LeerParedes();
    s1FloodFill();
    int8_t dir = s1MejorDireccion();
    if (dir < 0) {
      Serial.println(F("!! Ya no hay a donde explorar y no vi IN"));
      return false;
    }
    if (!s1GirarHacia(dir)) continue;   // pared recien vista: volver a planear

    float desdeRojo;
    S1Avance r = s1AvanzarCelda(desdeRojo);
    if (r == S1_ROJO) {
      // Primera lectura roja: los sensores en la orilla de IN (MEDIA antes de su
      // centro) y el centro del robot COLOR_OFFSET_MM mas atras
      xFinal = desdeRojo - MEDIA - COLOR_OFFSET_MM;
      Serial.print(F("Rojo de IN: el robot quedo a ")); Serial.print(-xFinal / 10.0f, 1);
      Serial.println(F(" cm antes de su centro"));
      float x0 = xFinal;
      avanzarMM(B_EXTRA_CHECKPOINT_MM);
      xFinal = x0 + recorridoMM();
      return true;
    }
    if (r == S1_ATORADO) {
      Serial.println(F("!! No se pudo pasar: se marca como pared"));
      s1AgregarPared(s1x, s1y, dir);
      continue;
    }
    s1x += dx[dir];
    s1y += dy[dir];
    s1Visitada[s1x][s1y] = true;
    Serial.print(F("S1 pos ")); Serial.print(s1x - S1_INICIO); Serial.print(',');
    Serial.println(s1y - S1_INICIO);
  }
  Serial.println(F("!! Demasiados pasos sin encontrar IN"));
  return false;
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

// xInicial: donde esta el centro del robot respecto al centro de IN (0 si
// arranca puesto en IN; la seccion 1 lo deja un poco antes)
bool seccion2(float xInicial) {
  Serial.println(F("== Seccion 2: lineas blancas =="));
  heading = 0;
  posX = xInicial;
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
  float xIn = 0;   // donde queda el robot respecto al centro de IN
  if (PISTA_B_SECCION_INICIAL <= 1) ok = seccion1(xIn);
  if (ok && PISTA_B_SECCION_INICIAL <= 2) ok = seccion2(xIn);
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
