#pragma once
#include <Arduino.h>

// u(k) = Kp*e(k) + Ki*Ts*sum(e(i)) + Kd*(e(k)-e(k-1))/Ts
// Sin feedforward: estos tres terminos son TODA la salida.

class PID {
  public:
    // errorThreshold: banda muerta de la INTEGRAL (en las unidades del error).
    //   Error por debajo de esto no se acumula: es ruido del sensor, no una
    //   desviacion real. Ojo: solo congela la integral, P y D siguen respondiendo.
    // unaDireccion: si true, la salida nunca cruza de signo (se fuerza a 0) sin
    //   que fijarDireccionPermitida() lo autorice explicitamente.
    void configurar(float kp, float ki, float kd, float salidaMax,
                    float errorThreshold = 0.0f, bool unaDireccion = false) {
      Kp = kp; Ki = ki; Kd = kd;
      this->salidaMax = salidaMax;
      this->errorThreshold = errorThreshold;   // Calibrar
      this->unaDireccion = unaDireccion;
    }

    // Llamar ANTES de calcular() en cada ciclo si unaDireccion esta activo:
    // le dice al PID cual es la direccion que pidio el movimiento actual.
    void fijarDireccionPermitida(bool soloPositivos) {
      permitirPositivos = soloPositivos;
    }

    float calcular(float setpoint, float medido, float dt) {
      // dt invalido: mantener la ultima salida en lugar de inventar una nueva.
      if (dt <= 0.0f) return constrain(salidaPrevia, -salidaMax, salidaMax);
      float error = setpoint - medido;

      // ---- first run ----
      // Tras reiniciar() no hay historial: errorPrevio seria un 0 falso y la
      // derivada daria un golpe enorme que no corresponde a nada fisico.
      // Primer ciclo: SOLO proporcional, y se siembra el historial con datos reales.
      if (primeraVez) {
        primeraVez = false;
        errorPrevio = error;
        salidaPrevia = limitarDireccion(constrain(Kp * error, -salidaMax, salidaMax));
        return salidaPrevia;
      }

      // Banda muerta: error chico no acumula integral, no cuenta.
      bool dentroDeBanda = fabs(error) < errorThreshold;

      // Anti-windup: si la salida ya topo y el error empuja HACIA el tope,
      // congelar la integral. Si el error va al otro lado, sigue libre para
      // descargarse de inmediato.
      bool saturadoArriba = (salidaPrevia >=  salidaMax) && (error > 0);
      bool saturadoAbajo  = (salidaPrevia <= -salidaMax) && (error < 0);

      if (!dentroDeBanda && !saturadoArriba && !saturadoAbajo) {
        integral += Ki * error * dt;   // Ki*Ts*sum(e(i)), acumulado paso a paso
      }

      // Derivada sobre el ERROR, no sobre la medicion (forma de la ecuacion de arriba).
      float derivada = (error - errorPrevio) / dt;
      errorPrevio = error;

      float salida = Kp * error + integral + Kd * derivada;
      salida = constrain(salida, -salidaMax, salidaMax);
      salida = limitarDireccion(salida);

      salidaPrevia = salida;
      return salida;
    }

    // Llamar SIEMPRE al iniciar un movimiento nuevo.
    void reiniciar() {
      integral = 0;
      errorPrevio = 0;
      salidaPrevia = 0;
      primeraVez = true;
    }

  private:
    // Una sola direccion: protege al driver de un cambio de signo brusco que el
    // movimiento actual no autorizo. Invertir la polaridad del puente H con el
    // motor todavia girando lo vuelve generador peleando contra la fuente: pico
    // de corriente, driver caliente y posible caida del riel de 5V.
    // Forzar a 0 deja la rueda libre, o sea frena por friccion.
    // Se aplica en los DOS caminos de calcular(), incluido el de first run.
    float limitarDireccion(float salida) {
      if (!unaDireccion) return salida;
      if ( permitirPositivos && salida < 0) return 0.0f;
      if (!permitirPositivos && salida > 0) return 0.0f;
      return salida;
    }

    float Kp = 0, Ki = 0, Kd = 0;
    float salidaMax = 255;
    float errorThreshold = 0.0f;
    bool unaDireccion = false;
    bool permitirPositivos = true;

    float integral = 0;        // ya multiplicada por Ki
    float errorPrevio = 0;
    float salidaPrevia = 0;    // ya limitada, es la que vio el motor
    bool primeraVez = true;
};

// first run = cada que el PID corre tras reiniciar() no tiene historial:
// errorPrevio = 0, la integral no tiene base. En el primer ciclo solo usamos la
// proporcional y sembramos errorPrevio y salidaPrevia con valores reales.

// Anti windup = cuando el output ya esta en su maximo, el motor no puede ir mas
// rapido. Si el error aun es + la integral nada mas infla un numero sin efecto,
// y cuando el error se invierte tarda muchisimo en descargarse -> sobrepaso tardio.
// Si el output anterior estaba saturado y el error actual va pal mismo lado,
// congelamos la integral. Pero si el signo va pa otro lado, la integral sigue.

// La integral se guarda YA multiplicada por Ki, asi cambiar Ki no reescala el
// historial que ya se acumulo.
