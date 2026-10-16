#pragma once
#include <Arduino.h>
#include "config.h"

// Espera para arrancar la ronda. Con boton (PIN_BOTON_INICIO): a que lo
// presionen y lo suelten. Sin boton: INICIO_ESPERA_MS.
void esperarArranque();
