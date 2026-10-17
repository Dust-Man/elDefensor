#include "vision.h"
#include "pantalla.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

VisionMsg visionUltimo = { false, 0, {} };
bool visionEnlace = false;

static char linea[64];
static uint8_t largo = 0;
static bool descartando = false;   // linea demasiado larga: ignorar hasta el siguiente '\n'
static uint32_t tUltimoRx = 0;
static bool vencido = false;
static bool unaFila = false;      // ya se mostro "Sin enlace"

// ---- analisis de la linea (sin sscanf: la lista de ids es de longitud variable) ----
static bool comer(const char*& p, char c) {
  if (*p != c) return false;
  ++p;
  return true;
}

static bool leerEntero(const char*& p, int& v) {
  char* fin;
  long x = strtol(p, &fin, 10);
  if (fin == p) return false;
  v = (int)x;
  p = fin;
  return true;
}

// M,<bola>,<n>,<id>...   Rechaza todo lo que no cuadre exactamente.
static bool parsearM(const char* s, VisionMsg& m) {
  const char* p = s;
  int bola, n;
  if (!comer(p, 'M') || !comer(p, ',') || !leerEntero(p, bola) || (bola != 0 && bola != 1)) return false;
  if (!comer(p, ',') || !leerEntero(p, n) || n < 0 || n > VISION_MAX_IDS) return false;
  for (int i = 0; i < n; i++) {
    int id;
    if (!comer(p, ',') || !leerEntero(p, id) || id < 0 || id > 999) return false;
    m.ids[i] = id;
  }
  if (*p != '\0') return false;       // sobra algo: linea corrupta
  m.bola = (bola == 1);
  m.n = (uint8_t)n;
  return true;
}

// D,<bx>,<by>,<br>,<aid>  (formato v1)
static bool parsearD(const char* s, VisionMsg& m) {
  int bx, by, br, aid;
  if (sscanf(s, "D,%d,%d,%d,%d", &bx, &by, &br, &aid) != 4) return false;
  m.bola = (bx >= 0);
  m.n = (aid >= 0) ? 1 : 0;
  if (m.n) m.ids[0] = aid;
  return true;
}

bool visionParsear(const char* s, VisionMsg& m) {
  if (s[0] == 'M') return parsearM(s, m);
  if (s[0] == 'D') return parsearD(s, m);
  return false;
}

// Ids separados por espacio en `ancho` columnas. Si no caben todos, los que quepan y '+'.
static void formatoIds(const VisionMsg& m, char* out /* ancho + 1 */, uint8_t ancho) {
  if (m.n == 0) { snprintf(out, ancho + 1, "-"); return; }
  char tmp[8];
  uint8_t total = 0;
  for (uint8_t i = 0; i < m.n; i++) {
    total += snprintf(tmp, sizeof(tmp), "%d", m.ids[i]) + (i ? 1 : 0);
  }
  uint8_t limite = (total <= ancho) ? ancho : ancho - 1;     // si hay que cortar, se reserva sitio para '+'
  uint8_t pos = 0, mostrados = 0;
  for (uint8_t i = 0; i < m.n; i++) {
    uint8_t necesita = snprintf(tmp, sizeof(tmp), "%d", m.ids[i]) + (i ? 1 : 0);
    if (pos + necesita > limite) break;
    if (i) out[pos++] = ' ';
    for (const char* t = tmp; *t; ++t) out[pos++] = *t;
    mostrados++;
  }
  if (mostrados < m.n) out[pos++] = '+';
  out[pos] = '\0';
}

void visionFormatoFilas(const VisionMsg& m, char* fila1, char* fila2) {
  snprintf(fila1, 17, "ArUco x%d Bola:%s", m.n, m.bola ? "si" : "no");
  formatoIds(m, fila2, PANTALLA_COLUMNAS);
}

// "Ar:7 12 3   B:si": ids en medio, la bola siempre en las ultimas 4 columnas
static const char PREFIJO_UNA_FILA[] = "Ar:";
constexpr uint8_t ANCHO_BOLA_UNA_FILA = 5;   // " B:si"
constexpr uint8_t ANCHO_IDS_UNA_FILA =
    PANTALLA_COLUMNAS - (sizeof(PREFIJO_UNA_FILA) - 1) - ANCHO_BOLA_UNA_FILA;

void visionFormatoUnaFila(const VisionMsg& m, char* fila /* 17 */) {
  char ids[ANCHO_IDS_UNA_FILA + 1];
  formatoIds(m, ids, ANCHO_IDS_UNA_FILA);
  // Sin "%-*s": el printf de avr-libc no soporta '*' como ancho (cortaba la linea tras "Ar:")
  // y rellena a mano los ids hasta ANCHO_IDS_UNA_FILA columnas.
  char relleno[ANCHO_IDS_UNA_FILA + 1];
  memset(relleno, ' ', ANCHO_IDS_UNA_FILA);
  relleno[ANCHO_IDS_UNA_FILA] = '\0';
  memcpy(relleno, ids, strnlen(ids, ANCHO_IDS_UNA_FILA));
  snprintf(fila, 17, "%s%s B:%s", PREFIJO_UNA_FILA, relleno, m.bola ? "si" : "no");
}

static bool mismosIds(const VisionMsg& a, const VisionMsg& b) {
  if (a.n != b.n) return false;
  for (uint8_t i = 0; i < a.n; i++)
    if (a.ids[i] != b.ids[i]) return false;
  return true;
}

static bool manejarLinea(const char* s) {
  VisionMsg m;
  if (!visionParsear(s, m)) return false;   // basura: se ignora
  tUltimoRx = millis();
  // En la consola solo cuando cambian los ids (llega un mensaje por foto)
  if (!visionEnlace || !mismosIds(m, visionUltimo)) {
    Serial.print(F(">> ArUco"));
    if (m.n == 0) Serial.print(F(" -"));
    for (uint8_t i = 0; i < m.n; i++) { Serial.print(' '); Serial.print(m.ids[i]); }
    Serial.println();
  }
  visionUltimo = m;
  visionEnlace = true;
  vencido = false;
  char a[17], b[17];
  if (unaFila) {
    visionFormatoUnaFila(m, b);
  } else {
    visionFormatoFilas(m, a, b);
    pantallaFila(0, a);
  }
  pantallaFila(1, b);
  return true;
}

void visionIniciar(bool soloFila2) {
  unaFila = soloFila2;
  if (unaFila) {
    pantallaFila(1, "ArUco: esperando");
  } else {
    pantallaFila(0, "Enlace USB");
    pantallaFila(1, "esperando datos");
  }
  tUltimoRx = millis();
  visionEnlace = false;
  vencido = false;
  Serial.println(F("B"));   // anuncia el arranque: el celular reinicia la vision
}

bool visionActualizar() {
  bool nuevo = false;
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      if (!descartando && largo > 0) {
        linea[largo] = '\0';
        if (manejarLinea(linea)) nuevo = true;
      }
      largo = 0;
      descartando = false;
      continue;
    }
    if (descartando) continue;
    if (largo < sizeof(linea) - 1) linea[largo++] = c;
    else { descartando = true; largo = 0; }
  }

  if (!vencido && millis() - tUltimoRx > VISION_TIMEOUT_MS) {
    vencido = true;
    visionEnlace = false;
    Serial.println(F("!! Vision: sin enlace"));
    if (unaFila) {
      pantallaFila(1, "ArUco:sin enlace");
    } else {
      pantallaFila(0, "Sin enlace");
      pantallaFila(1, "sin datos nuevos");
    }
  }
  return nuevo;
}
