#include <Servo.h>
#include "servos.h"

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

// Un pin compartido con un ultrasonico: el servo le meteria pulsos a su eco
bool servoPinOcupado() {
  for (uint8_t i = 0; i < NUM_ULTRASONICOS; i++) {
    const uint8_t pines[2] = { PIN_US_TRIG[i], PIN_US_ECHO[i] };
    for (uint8_t p : pines)
      if (p == PIN_SERVO_1 || p == PIN_SERVO_2) return true;
  }
  return false;
}

// Attach con la configuracion probada; escribe antes para que no brinque a 90
// Autor: Carlos Parrilla
void servosConectar() {
  if (servosConectados) return;
  if (servoPinOcupado()) {
    Serial.println(F("!! Pines de servos ocupados por un ultrasonico (config.h): no se conectan"));
    return;
  }
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
void servosMover(int a, bool imprimir) {
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
