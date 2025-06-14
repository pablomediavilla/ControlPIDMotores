#include <ControlPIDMotores.h>

// Kp, Ki, Kd, sampleTime, modo
ControlPIDMotores pid(4.0, 0.8, 0.5, 0.05, PID_DISCRETO);

void setup() {
  pid.configurarMotores(9, 7, 8, 10, 5, 6, 4);  // PWMA, AIN1, AIN2, PWMB, BIN1, BIN2, STBY
  pid.definirReferencia(0.0);  // Mantener el ángulo en 0 grados
  pid.iniciar();
}

void loop() {
  pid.actualizar();
}