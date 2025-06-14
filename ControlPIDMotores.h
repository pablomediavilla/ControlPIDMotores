#ifndef CONTROLPIDMOTORES_H
#define CONTROLPIDMOTORES_H

#include <Arduino.h>
#include <MPU6050.h>

enum ModoPID {
  PID_DISCRETO,
  PID_IIR,
  PID_LPF_D
};

class ControlPIDMotores {
public:
  ControlPIDMotores(double kp, double ki, double kd, double tiempoMuestreo, ModoPID modo = PID_IIR, double constanteFiltroD = 0.01);

  void iniciar();
  void actualizar();
  void definirReferencia(double referencia);
  void configurarMotores(int pwma, int ain1, int ain2, int pwmb, int bin1, int bin2, int stby);

private:
  void leerAngulo();
  void moverMotor(int pwm, int in1, int in2, int velocidad);
  void configurarPinesMotores();

  void actualizarDiscreto();
  void actualizarIIR();
  void actualizarLPF_D();

  MPU6050 mpu;

  // Parametros PID
  double Kp, Ki, Kd, T;
  double tau_d; // constante del filtro pasa baja
  ModoPID modo;

  // Estados PID
  double referencia, entrada, error;
  double error_anterior, error_pasado, salida, salida_anterior;
  double derivada_filtrada;

  // Coeficientes IIR
  double a0, a1, a2;

  // Tiempo
  unsigned long ultimoTiempo;

  // Pines del puente H
  int pinPWMA, pinAIN1, pinAIN2, pinPWMB, pinBIN1, pinBIN2, pinSTBY;
};

#endif
