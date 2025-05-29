#include "PIDControl.h"

// PID Discreto
PID_Discreto::PID_Discreto(float Kp, float Ki, float Kd, float dt)
    : Kp(Kp), Ki(Ki), Kd(Kd), dt(dt), integral(0.0f), prev_error(0.0f) {}

float PID_Discreto::compute(float setpoint, float measured_value) {
    float error = setpoint - measured_value;
    integral += error * dt;
    float derivative = (error - prev_error) / dt;
    prev_error = error;
    return Kp * error + Ki * integral + Kd * derivative;
}

// PID IIR
PID_IIR::PID_IIR(float Kp, float Ki, float Kd, float dt)
    : dt(dt) {
    A0 = Kp + Ki * dt + Kd / dt;
    A1 = -Kp - 2.0f * Kd / dt;
    A2 = Kd / dt;
    error[0] = error[1] = error[2] = 0.0f;
}

float PID_IIR::compute(float setpoint, float measured_value) {
    error[2] = error[1];
    error[1] = error[0];
    error[0] = setpoint - measured_value;
    return A0 * error[0] + A1 * error[1] + A2 * error[2];
}

// PID con derivada filtrada (pasabaixa)
PID_Filtrado::PID_Filtrado(float Kp, float Ki, float Kd, float dt, float N)
    : Kp(Kp), Ki(Ki), Kd(Kd), dt(dt), N(N),
      output(0.0f), d0(0.0f), d1(0.0f), fd0(0.0f), fd1(0.0f) {
    
    A0 = Kp + Ki * dt;
    A1 = -Kp;

    A0d = Kd / dt;
    A1d = -2.0f * Kd / dt;
    A2d = Kd / dt;

    tau = Kd / (Kp * N);
    alpha = dt / (2.0f * tau);
    alpha_1 = alpha / (alpha + 1.0f);
    alpha_2 = (alpha - 1.0f) / (alpha + 1.0f);

    error[0] = error[1] = error[2] = 0.0f;
}

float PID_Filtrado::compute(float setpoint, float measured_value) {
    error[2] = error[1];
    error[1] = error[0];
    error[0] = setpoint - measured_value;

    output += A0 * error[0] + A1 * error[1];

    d1 = d0;
    d0 = A0d * error[0] + A1d * error[1] + A2d * error[2];

    fd1 = fd0;
    fd0 = alpha_1 * (d0 + d1) - alpha_2 * fd1;

    return output + fd0;
}
