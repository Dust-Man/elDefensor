// Banco de vision: recibe del celular (CVSquirtDeAga) y lo muestra en el LCD,
// igual que el firmware 'uno' de ese repo pero en el Mega del robot.
// Compilar/subir con el entorno [env:vision]. No mueve motores.
#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "pantalla.h"
#include "vision.h"

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(I2C_RELOJ_HZ);
  Wire.setWireTimeout(3000, true);
  pantallaIniciar();
  visionIniciar();
}

void loop() {
  visionActualizar();
  pantallaActualizar();
}
