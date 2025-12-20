#ifndef STM32PID_H
#define STM32PID_H

class PID
{
private:
    float Kp, Ki, Kd;       // Katsayılar
    float dt;               // Örnekleme zamanı
    float satMax, satMin;   // Saturasyon üst ve alt değerler

    float Kaw;              // Integral anti-windup için
    float tau;

    float integrator;
    float dFilter;          // Türev alçak geçirgen filtre
    float prevMeasurement;
    bool firstRun;

public:
    PID(float Kp_d,
        float Ki_d,
        float Kd_d,
        float dt_d,
        float saturationMax,
        float saturationMin,
        float antiWindupGain,
        float derivativeTau);

    float update(float reference, float measurement);
    void reset();
};

#endif // STM32PID_H
