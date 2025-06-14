#include "ControlPIDMotores.h"
#include <Wire.h>
#include <math.h>

ControlPIDMotores::ControlPIDMotores(double kp, double ki, double kd, double tiempoMuestreo, ModoPID m, double constanteFiltroD)
  : Kp(kp), Ki(ki), Kd(kd), T(tiempoMuestreo), modo(m), tau_d(constanteFiltroD),
    referencia(0), entrada(0), error(0),
    error_anterior(0), error_pasado(0), salida(0), salida_anterior(0), derivada_filtrada(0), ultimoTiempo(0)
{
  a0 = Kp + Ki * T + Kd / T;
  a1 = -Kp - 2.0 * Kd / T;
  a2 = Kd / T;
}

void ControlPIDMotores::configurarMotores(int pwma, int ain1, int ain2, int pwmb, int bin1, int bin2, int stby) {
  pinPWMA = pwma; pinAIN1 = ain1; pinAIN2 = ain2;
  pinPWMB = pwmb; pinBIN1 = bin1; pinBIN2 = bin2;
  pinSTBY = stby;
}

void ControlPIDMotores::configurarPinesMotores() {
  pinMode(pinPWMA, OUTPUT); pinMode(pinAIN1, OUTPUT); pinMode(pinAIN2, OUTPUT);
  pinMode(pinPWMB, OUTPUT); pinMode(pinBIN1, OUTPUT); pinMode(pinBIN2, OUTPUT);
  pinMode(pinSTBY, OUTPUT);
  digitalWrite(pinSTBY, HIGH);
}

void ControlPIDMotores::iniciar() {
  Wire.begin();
  mpu.initialize();
  configurarPinesMotores();
  ultimoTiempo = millis();
}

void ControlPIDMotores::leerAngulo() {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  double anguloAcelerometro = atan2(ay, az) * 180.0 / PI;
  double velocidadGyro = gx / 131.0;
  double dt = (millis() - ultimoTiempo) / 1000.0;

  entrada = 0.96 * (entrada + velocidadGyro * dt) + 0.04 * anguloAcelerometro;
}

void ControlPIDMotores::actualizar() {
  unsigned long ahora = millis();
  if ((ahora - ultimoTiempo) >= T * 1000.0) {
    leerAngulo();
    switch (modo) {
      case PID_IIR: actualizarIIR(); break;
      case PID_DISCRETO: actualizarDiscreto(); break;
      case PID_LPF_D: actualizarLPF_D(); break;
    }
    ultimoTiempo = ahora;
  }
}

void ControlPIDMotores::actualizarIIR() {
  error = referencia - entrada;
  salida = salida_anterior + a0 * error + a1 * error_anterior + a2 * error_pasado;
  salida = constrain(salida, -255, 255);

  moverMotor(pinPWMA, pinAIN1, pinAIN2, salida);
  moverMotor(pinPWMB, pinBIN1, pinBIN2, salida);

  error_pasado = error_anterior;
  error_anterior = error;
  salida_anterior = salida;
}

void ControlPIDMotores::actualizarDiscreto() {
  error = referencia - entrada;
  static double I = 0;
  I += Ki * error * T;
  double D = Kd * (error - error_anterior) / T;
  double P = Kp * error;

  salida = P + I + D;
  salida = constrain(salida, -255, 255);

  moverMotor(pinPWMA, pinAIN1, pinAIN2, salida);
  moverMotor(pinPWMB, pinBIN1, pinBIN2, salida);

  error_anterior = error;
}

void ControlPIDMotores::actualizarLPF_D() {
  error = referencia - entrada;
  static double I = 0;
  I += Ki * error * T;
  double P = Kp * error;

  double derivada_bruta = (error - error_anterior) / T;
  double alfa = tau_d / (tau_d + T);
  derivada_filtrada = alfa * derivada_filtrada + (1 - alfa) * derivada_bruta;
  double D = Kd * derivada_filtrada;

  salida = P + I + D;
  salida = constrain(salida, -255, 255);

  moverMotor(pinPWMA, pinAIN1, pinAIN2, salida);
  moverMotor(pinPWMB, pinBIN1, pinBIN2, salida);

  error_anterior = error;
}

void ControlPIDMotores::moverMotor(int pwm, int in1, int in2, int velocidad) {
  velocidad = constrain(velocidad, -255, 255);
  if (velocidad > 0) {
    digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
    analogWrite(pwm, velocidad);
  } else if (velocidad < 0) {
    digitalWrite(in1, LOW); digitalWrite(in2, HIGH);
    analogWrite(pwm, -velocidad);
  } else {
    digitalWrite(in1, LOW); digitalWrite(in2, LOW);
    analogWrite(pwm, 0);
  }
}

void ControlPIDMotores::definirReferencia(double ref) {
  referencia = ref;
}
