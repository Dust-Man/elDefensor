#pragma once
#include <Arduino.h>
#include "config.h"

// Enlace con el celular (CVSquirtDeAga) por el USB del Mega: el mismo Serial de
// los mensajes de depuracion. Port de arduino/src/main.cpp de ese repo.
//
// Protocolo v2 (ASCII, una linea por mensaje, 115200 baud, fin de linea '\n'):
//     M,<bola>,<n>,<id1>,...,<idn>
//   bola : 1 si se ve la pelota, 0 si no (solo como indicador de depuracion)
//   n    : cuantos ArUco se reportan (0..VISION_MAX_IDS)
//   idk  : ids de los ArUco, el mas grande (el mas cercano) primero
// Tambien acepta el formato anterior  D,<bx>,<by>,<br>,<aid>
// Mega -> celular: visionIniciar() manda  B\n  ("arranque"); el celular reinicia
// la vision. Ningun otro mensaje de depuracion puede ser una linea "B" sola.
//
// Lo que llega por USB NO es confiable: linea con tope de longitud, se descarta
// lo que no parsea, y tras VISION_TIMEOUT_MS sin mensaje valido el LCD dice
// "Sin enlace" (un valor viejo en pantalla pareceria un dato actual).

constexpr uint8_t VISION_MAX_IDS = 8;   // debe coincidir con MAX_IDS de link.py (CVSquirtDeAga)

struct VisionMsg {
  bool bola;
  uint8_t n;
  int ids[VISION_MAX_IDS];
};

extern VisionMsg visionUltimo;   // ultimo mensaje valido
extern bool visionEnlace;        // false si nunca llego uno o paso VISION_TIMEOUT_MS

// Texto inicial del LCD y anuncio "B". Llamar al final de la inicializacion.
// soloFila2 = false: las 2 filas como el firmware original ("ArUco x3 Bola:si" / "7 12 3").
// soloFila2 = true: solo la fila 2 ("Ar:7 12 3   B:si"); la fila 1 queda libre (Pista A: color).
void visionIniciar(bool soloFila2 = false);
// Lee lo que haya en el Serial sin bloquear y actualiza el LCD (pantallaFila).
// true si llego un mensaje valido nuevo.
bool visionActualizar();

// Sin Serial ni LCD: tambien sirven para probar el protocolo en la PC
bool visionParsear(const char* s, VisionMsg& m);
void visionFormatoFilas(const VisionMsg& m, char* fila1 /* 17 */, char* fila2 /* 17 */);
void visionFormatoUnaFila(const VisionMsg& m, char* fila /* 17 */);
