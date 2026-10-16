// Pista A: laberinto de 5x5 unidades. Compilar/subir con el entorno [env:pistaA].
// Explora todo el laberinto mostrando los colores del piso, llega a la meta roja
// y regresa al inicio por las mismas unidades (bonus 1).
// La logica es la de exploreColor.cpp (probada en mms); lo que cambia es el robot:
// paredes con los ultrasonicos, color con el TCS34725 mientras avanza y
// movimientos con el control de movimiento.cpp.
#include <Arduino.h>
#include "config.h"
#include "movimiento.h"
#include "color.h"
#include "ultrasonicos.h"
#include "arranque.h"

// La casilla de inicio cambia por ronda y no se puede pre-mapear: se usa una
// cuadricula virtual de 9x9 con el inicio al centro, asi el laberinto real de 5x5
// cabe sin importar donde empiece. Con lo visitado se va cerrando la ventana de
// 5x5 donde puede estar el laberinto (posible()).
constexpr int8_t LADO = 5;
constexpr int8_t N = 2 * LADO - 1;
constexpr int8_t INICIO = LADO - 1;

// direcciones:
// 0 = norte (hacia donde ve el robot al arrancar), 1 = este, 2 = sur, 3 = oeste
const int8_t dx[4] = { 0, 1, 0, -1 };
const int8_t dy[4] = { 1, 0, -1, 0 };
const char dirChar[4] = { 'N', 'E', 'S', 'O' };

struct Celda { int8_t x, y; };

const int8_t SIN_LEER = -2;   // colorVisto: el sensor aun no la ha visto

bool walls[N][N][4];
bool visited[N][N];
// Celdas que el floodFill no puede atravesar (la meta roja mientras se explora)
bool bloqueada[N][N];
int8_t colorVisto[N][N];      // SIN_LEER, COLOR_NINGUNO o el color
bool colorEncontrado[NUM_COLORES];
int8_t dist[N][N];
int8_t minX, maxX, minY, maxY;   // ventana de lo visitado

int8_t px = INICIO, py = INICIO;
int8_t heading = 0;

Celda salida;
bool salidaConocida = false;

// Trayectoria completa: dirección de cada avance
constexpr uint8_t MAX_MOVS = 250;
uint8_t movimientos[MAX_MOVS];
uint8_t numMovs = 0;
bool movsLlenos = false;   // ya no cabe: el regreso usa la ruta sin ciclos
// Trayectoria sin ciclos desde el inicio hasta la posición actual
Celda ruta[N * N];
uint8_t largoRuta = 0;

uint32_t tInicioRonda = 0;

// ---------------------------------------------------------------------------
// Mapa
// ---------------------------------------------------------------------------
bool inside(int8_t x, int8_t y) {
  return x >= 0 && x < N && y >= 0 && y < N;
}

// Puede ser parte del laberinto: cabe en una ventana de 5x5 junto con todo lo visitado
bool posible(int8_t x, int8_t y) {
  return inside(x, y) && x > maxX - LADO && x < minX + LADO && y > maxY - LADO && y < minY + LADO;
}

void marcarVisitada(int8_t x, int8_t y) {
  visited[x][y] = true;
  minX = min(minX, x); maxX = max(maxX, x);
  minY = min(minY, y); maxY = max(maxY, y);
}

void addWall(int8_t x, int8_t y, int8_t dir) {
  walls[x][y][dir] = true;
  int8_t nx = x + dx[dir];
  int8_t ny = y + dy[dir];
  // Colocar pared en la dirección opuesta
  if (inside(nx, ny)) walls[nx][ny][(dir + 2) % 4] = true;
}

// Validar si me puedo mover a una celda en una dirección
bool isAccesible(int8_t x, int8_t y, int8_t dir) {
  if (walls[x][y][dir]) return false;
  int8_t nx = x + dx[dir];
  int8_t ny = y + dy[dir];
  if (!posible(nx, ny)) return false;
  return !bloqueada[nx][ny];
}

void floodFill(const Celda* goals, uint8_t numGoals) {
  static Celda tasks[N * N];
  uint8_t ini = 0, fin = 0;
  for (int8_t x = 0; x < N; x++)
    for (int8_t y = 0; y < N; y++) dist[x][y] = -1;

  for (uint8_t i = 0; i < numGoals; i++) {
    dist[goals[i].x][goals[i].y] = 0;
    tasks[fin++] = goals[i];
  }
  while (ini < fin) {
    Celda act = tasks[ini++];
    int8_t d = dist[act.x][act.y];
    for (int8_t dir = 0; dir < 4; dir++) {
      if (!isAccesible(act.x, act.y, dir)) continue;
      int8_t nx = act.x + dx[dir];
      int8_t ny = act.y + dy[dir];
      if (dist[nx][ny] != -1) continue;
      dist[nx][ny] = d + 1;
      tasks[fin++] = { nx, ny };
    }
  }
}

// Prefiere seguir derecho cuando hay empate para ahorrar giros
int8_t bestDirection() {
  int8_t best = -1;
  int8_t menor = 127;
  for (int8_t k = 0; k < 4; k++) {
    int8_t dir = (heading + k) % 4;
    if (!isAccesible(px, py, dir)) continue;
    int8_t d = dist[px + dx[dir]][py + dy[dir]];
    if (d == -1) continue;
    if (d < menor) { menor = d; best = dir; }
  }
  return best;
}

// Celdas pendientes de visitar (la meta no cuenta, no se puede pisar)
uint8_t unvisited(Celda* pend) {
  uint8_t n = 0;
  for (int8_t x = 0; x < N; x++)
    for (int8_t y = 0; y < N; y++)
      if (!visited[x][y] && !bloqueada[x][y] && posible(x, y)) pend[n++] = { x, y };
  return n;
}

int8_t direccionHacia(const Celda& c) {
  for (int8_t dir = 0; dir < 4; dir++)
    if (px + dx[dir] == c.x && py + dy[dir] == c.y) return dir;
  return -1;
}

// ---------------------------------------------------------------------------
// Colores
// ---------------------------------------------------------------------------
// Unico lugar donde se le muestra el color al juez. TODO: pantalla.
void mostrarColor(int8_t c) {
  Serial.print(F(">> COLOR ")); Serial.println(NOMBRE_COLOR[c]);
}

void registrarColor(int8_t x, int8_t y, int8_t c) {
  if (c == SIN_LEER || colorVisto[x][y] >= 0) return;   // ya tiene color: no se cambia
  colorVisto[x][y] = c;
  if (c < 0) return;
  colorEncontrado[c] = true;

  Serial.print(F("Color ")); Serial.print(NOMBRE_COLOR[c]);
  Serial.print(F(" en (")); Serial.print(x - INICIO); Serial.print(',');
  Serial.print(y - INICIO); Serial.println(')');
  // Solo los 4 colores del laberinto dan puntos
  if (c == COLOR_CYAN || c == COLOR_AMARILLO || c == COLOR_NARANJA || c == COLOR_ROSA) mostrarColor(c);

  if (c == COLOR_ROJO && !salidaConocida) {
    salida = { x, y };
    salidaConocida = true;
    // No se entra a la meta hasta terminar de explorar
    bloqueada[x][y] = true;
    Serial.println(F("Meta encontrada, se guarda y se sigue explorando"));
  }
}

// Incluye la meta roja: si es true ya se conoce la salida
bool todosLosColores() {
  for (int8_t c = 0; c < NUM_COLORES; c++)
    if (!colorEncontrado[c]) return false;
  return true;
}

// ---------------------------------------------------------------------------
// Robot
// ---------------------------------------------------------------------------
bool hayPared(float cm) {
  return cm > 0 && cm < US_PARED_CM;   // sin eco = no hay pared cerca
}

// Si hay pared enfrente, se acerca o aleja hasta quedar centrado en la celda:
// asi el error de los encoders no se acumula de celda en celda.
void alinearConFrente(float cm) {
  if (!ALINEAR_CON_PARED || !hayPared(cm)) return;
  float error = cm - US_FRENTE_OBJETIVO_CM;
  if (fabs(error) < 1.0f || fabs(error) > ALINEAR_MAX_CM) return;
  iniciarAvance(error * 10.0f);
  esperarMovimiento();
}

void readWalls() {
  asentar(P.asentarMs);
  float frente = ultrasonicoMedianaCm(US_FRENTE, US_MUESTRAS);
  float izq = ultrasonicoMedianaCm(US_IZQ, US_MUESTRAS);
  float der = ultrasonicoMedianaCm(US_DER, US_MUESTRAS);
  if (hayPared(frente)) addWall(px, py, heading);
  if (hayPared(der))    addWall(px, py, (heading + 1) % 4);
  if (hayPared(izq))    addWall(px, py, (heading + 3) % 4);
  alinearConFrente(frente);
}

// Tras girar: la pared que ahora queda enfrente pudo no haberse leido
// (la de atras del inicio nunca se lee)
bool paredEnfrente() {
  asentar(P.asentarMs);
  float frente = ultrasonicoMedianaCm(US_FRENTE, US_MUESTRAS);
  if (!hayPared(frente)) return false;
  addWall(px, py, heading);
  alinearConFrente(frente);
  return true;
}

// Devuelve true si tuvo que girar
bool orientateTo(int8_t dir) {
  int8_t diff = (dir - heading + 4) % 4;
  if (diff == 0) return false;
  float grados = GIRO_DERECHA_GRADOS;      // diff 1: derecha
  if (diff == 2) grados = 2 * GIRO_DERECHA_GRADOS;
  if (diff == 3) grados = -GIRO_DERECHA_GRADOS;   // izquierda
  iniciarGiro(grados);
  esperarMovimiento();
  heading = dir;
  return true;
}

enum Avance : uint8_t { LLEGO, META, ATORADO };

// Avanza una unidad leyendo el piso de la celda siguiente. El sensor va
// COLOR_OFFSET_MM adelante del centro: ve la celda siguiente cuando el robot
// todavia esta en la actual, asi si es la meta roja (y evitarMeta) se regresa
// al centro sin haberla pisado.
// colorLeido queda en SIN_LEER, COLOR_NINGUNO o el color de la celda siguiente.
Avance avanzarCelda(bool evitarMeta, int8_t& colorLeido) {
  colorLeido = SIN_LEER;
  int8_t ultimo = SIN_LEER;
  uint8_t racha = 0;
  uint32_t tColor = 0;

  iniciarAvance(CELDA_MM);
  while (estado == AVANZANDO) {
    controlActualizar();
    // Una lectura por integracion del sensor: las de en medio repiten el mismo dato
    if (millis() - tColor < COLOR_INTEGRACION_MS) continue;
    tColor = millis();
    // Posicion del sensor medida desde el centro de la celda de salida
    float sensor = recorridoMM() + COLOR_OFFSET_MM;
    if (sensor < CELDA_MM / 2 + COLOR_MARGEN_MM || sensor > CELDA_MM * 1.5f - COLOR_MARGEN_MM) continue;
    int8_t c;
    if (!colorDetectar(SENSOR_COLOR_PISO, c)) continue;
    racha = (c == ultimo) ? racha + 1 : 1;
    ultimo = c;
    if (racha < COLOR_LECTURAS_MIN) continue;
    if (colorLeido < 0) colorLeido = c;   // un color nunca se cambia por "ninguno"

    if (evitarMeta && colorLeido == COLOR_ROJO) {
      float hecho = recorridoMM();
      detenerTodo();
      iniciarAvance(-hecho);
      esperarMovimiento();
      return META;
    }
  }

  // Atorado (escalera que no sube, choque): si no paso la mitad, regresa al centro
  if (movExcedido) {
    float hecho = recorridoMM();
    if (hecho < CELDA_MM / 2) {
      iniciarAvance(-hecho);
      esperarMovimiento();
      return ATORADO;
    }
    Serial.println(F("!! Atorado despues de la mitad: se toma como celda alcanzada"));
  }
  return LLEGO;
}

// Avanzar sin leer piso ni registrar (regreso al inicio)
void avanzar() {
  iniciarAvance(CELDA_MM);
  esperarMovimiento();
  px += dx[heading];
  py += dy[heading];
  asentar(P.asentarMs);
  alinearConFrente(ultrasonicoMedianaCm(US_FRENTE, US_MUESTRAS));
}

// Registrar el avance de la ida (exploración y camino a la meta)
void mover() {
  px += dx[heading];
  py += dy[heading];
  marcarVisitada(px, py);
  if (numMovs < MAX_MOVS) movimientos[numMovs++] = heading;
  else movsLlenos = true;

  // Si ya estaba en la ruta se cerró un ciclo: se recorta
  for (uint8_t i = 0; i < largoRuta; i++) {
    if (ruta[i].x == px && ruta[i].y == py) { largoRuta = i; break; }
  }
  ruta[largoRuta++] = { px, py };

  Serial.print(F("pos ")); Serial.print(px - INICIO); Serial.print(',');
  Serial.print(py - INICIO); Serial.print(F(" h=")); Serial.println(dirChar[heading]);
}

// Gira hacia dir y avanza una celda registrando color. false si no se movio
// (pared recien vista, meta roja enfrente o atorado): hay que volver a planear.
bool pasoHacia(int8_t dir, bool evitarMeta) {
  if (orientateTo(dir) && paredEnfrente()) return false;
  int8_t c;
  Avance r = avanzarCelda(evitarMeta, c);
  registrarColor(px + dx[dir], py + dy[dir], c);
  if (r == META) return false;
  if (r == ATORADO) {
    Serial.println(F("!! No se pudo pasar: se marca como pared"));
    addWall(px, py, dir);
    return false;
  }
  mover();
  return true;
}

// ---------------------------------------------------------------------------
// Pista
// ---------------------------------------------------------------------------
void explorar() {
  static Celda goals[N * N];
  while (true) {
    readWalls();

    if (salidaConocida && millis() - tInicioRonda > PISTA_A_LIMITE_EXPLORAR_MS) {
      Serial.println(F("Tiempo de explorar agotado, voy a la meta"));
      return;
    }
    if (PISTA_A_TERMINAR_CON_COLORES && todosLosColores()) {
      Serial.println(F("Todos los colores encontrados, voy a la meta"));
      return;
    }
    uint8_t n = unvisited(goals);
    if (n == 0) {
      Serial.println(F("Laberinto explorado por completo"));
      return;
    }
    floodFill(goals, n);
    if (dist[px][py] == -1) {
      Serial.print(F("Exploración terminada: ")); Serial.print(n);
      Serial.println(F(" celdas inaccesibles sin pasar por la meta"));
      return;
    }
    pasoHacia(bestDirection(), true);
  }
}

bool irALaMeta() {
  bloqueada[salida.x][salida.y] = false;
  while (px != salida.x || py != salida.y) {
    readWalls();
    floodFill(&salida, 1);
    int8_t dir = bestDirection();
    if (dir == -1) {
      Serial.println(F("No hay camino a la meta"));
      return false;
    }
    pasoHacia(dir, false);
  }
  return true;
}

void regresarAlInicio() {
  if (RETORNO_EXACTO && !movsLlenos) {
    // Mismos movimientos en orden inverso y sentido contrario
    for (int16_t i = numMovs - 1; i >= 0; i--) {
      orientateTo((movimientos[i] + 2) % 4);
      avanzar();
    }
  } else {
    if (RETORNO_EXACTO) Serial.println(F("!! Trayectoria demasiado larga, regreso por la ruta sin ciclos"));
    for (int16_t i = largoRuta - 2; i >= 0; i--) {
      orientateTo(direccionHacia(ruta[i]));
      avanzar();
    }
  }
}

void resumenColores() {
  Serial.println(F("---- Colores encontrados ----"));
  for (int8_t c = 0; c < NUM_COLORES; c++) {
    bool alguno = false;
    for (int8_t x = 0; x < N; x++)
      for (int8_t y = 0; y < N; y++) {
        if (colorVisto[x][y] != c) continue;
        if (!alguno) { Serial.print(NOMBRE_COLOR[c]); Serial.print(':'); alguno = true; }
        Serial.print(F(" (")); Serial.print(x - INICIO); Serial.print(',');
        Serial.print(y - INICIO); Serial.print(')');
      }
    if (alguno) Serial.println();
  }
}

// Mapa de lo explorado (coordenadas relativas al inicio, norte arriba)
void imprimirMapa() {
  for (int8_t y = maxY; y >= minY; y--) {
    for (int8_t x = minX; x <= maxX; x++) Serial.print(walls[x][y][0] ? F("+---") : F("+   "));
    Serial.println('+');
    for (int8_t x = minX; x <= maxX; x++) {
      Serial.print(walls[x][y][3] ? '|' : ' ');
      char centro = ' ';
      if (x == INICIO && y == INICIO) centro = 'I';
      else if (salidaConocida && x == salida.x && y == salida.y) centro = 'M';
      else if (colorVisto[x][y] >= 0) centro = NOMBRE_COLOR[colorVisto[x][y]][0];
      else if (!visited[x][y]) centro = '?';
      Serial.print(' '); Serial.print(centro); Serial.print(' ');
    }
    Serial.println(walls[maxX][y][1] ? '|' : ' ');
  }
  for (int8_t x = minX; x <= maxX; x++) Serial.print(walls[x][minY][2] ? F("+---") : F("+   "));
  Serial.println('+');
}

void correrPistaA() {
  for (int8_t x = 0; x < N; x++)
    for (int8_t y = 0; y < N; y++) colorVisto[x][y] = SIN_LEER;
  minX = maxX = minY = maxY = INICIO;
  marcarVisitada(INICIO, INICIO);
  ruta[largoRuta++] = { INICIO, INICIO };
  // La celda de inicio siempre es verde
  registrarColor(INICIO, INICIO, COLOR_VERDE);

  tInicioRonda = millis();
  explorar();

  if (!salidaConocida) {
    Serial.println(F("No se encontró la celda roja"));
  } else if (irALaMeta()) {
    Serial.println(F("Llegué a la meta"));
    frenarTodos();
    delay(PISTA_A_PAUSA_META_MS);
    controlReiniciarReloj();
    regresarAlInicio();
    Serial.println(F("De regreso en el inicio"));
  }
  Serial.print(F("Tiempo: ")); Serial.print((millis() - tInicioRonda) / 1000); Serial.println(F(" s"));
  resumenColores();
  imprimirMapa();
}

void setup() {
  Serial.begin(115200);
  movimientoIniciar();   // calibra el giroscopio: robot QUIETO
  colorCalCargar();      // calibracion hecha con el main de debug (EEPROM)
  colorIniciarTodos();
  ultrasonicosIniciar();
  esperarArranque();
  correrPistaA();
}

void loop() {
  controlActualizar();   // DETENIDO: mantiene el freno
}
