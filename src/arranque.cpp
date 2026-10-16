#include "arranque.h"
#include "movimiento.h"

void esperarArranque() {
  if (PIN_BOTON_INICIO != SIN_PIN) {
    pinMode(PIN_BOTON_INICIO, INPUT_PULLUP);
    Serial.println(F("Listo. Presiona el boton para arrancar."));
    while (digitalRead(PIN_BOTON_INICIO) == HIGH) {}
    delay(30);
    while (digitalRead(PIN_BOTON_INICIO) == LOW) {}
    delay(500);   // que la mano se aleje
  } else {
    Serial.print(F("Arrancando en ")); Serial.print(INICIO_ESPERA_MS / 1000); Serial.println(F(" s"));
    delay(INICIO_ESPERA_MS);
  }
  controlReiniciarReloj();
}
