#include <ControlPIDMotores.h>

ControlPIDMotores pid(4.0, 0.8, 0.5, 0.05, PID_LPF_D, 0.02);  // Kp, Ki, Kd, T, modo, constante filtro D

void setup() {
  pid.configurarMotores(9, 7, 8, 10, 5, 6, 4);  // PWMA, AIN1, AIN2, PWMB, BIN1, BIN2, STBY
  pid.definirReferencia(0.0);                  // Mantener ángulo en 0°
  pid.iniciar();
}

void loop() {
  pid.actualizar();
}
