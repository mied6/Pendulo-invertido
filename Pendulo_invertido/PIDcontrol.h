#ifndef PIDCONTROL_H
#define PIDCONTROL_H

// Clase base abstracta para PID
class PID {
public:
    virtual float compute(float setpoint, float measured_value) = 0;
    virtual ~PID() {}
};

// PID Discreto clásico
class PID_Discreto : public PID {
public:
    PID_Discreto(float Kp, float Ki, float Kd, float dt);
    float compute(float setpoint, float measured_value);

private:
    float Kp, Ki, Kd, dt;
    float integral;
    float prev_error;
};

// PID con implementación IIR (según diferencias de errores)
class PID_IIR : public PID {
public:
    PID_IIR(float Kp, float Ki, float Kd, float dt);
    float compute(float setpoint, float measured_value);

private:
    float dt;
    float A0, A1, A2;
    float error[3];
};

// PID con derivada filtrada (filtro pasabajo)
class PID_Filtrado : public PID {
public:
    PID_Filtrado(float Kp, float Ki, float Kd, float dt, float N);
    float compute(float setpoint, float measured_value);

private:
    float Kp, Ki, Kd, dt, N;
    float output;
    float d0, d1;
    float fd0, fd1;
    float A0, A1;
    float A0d, A1d, A2d;
    float tau, alpha, alpha_1, alpha_2;
    float error[3];
};

#endif // PIDCONTROL_H

