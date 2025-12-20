#include "PID.h"

PID::PID(float Kp_d,
         float Ki_d,
         float Kd_d,
         float dt_d,
         float saturationMax,
         float saturationMin,
         float antiWindupGain,
         float derivativeTau)
    : Kp(Kp_d),
      Ki(Ki_d),
      Kd(Kd_d),
      dt(dt_d),
      satMax(saturationMax),
      satMin(saturationMin),
      Kaw(antiWindupGain),
      tau(derivativeTau),
      integrator(0.0f),
      dFilter(0.0f),
      prevMeasurement(0.0f),
      firstRun(true)
{
}

float PID::update(float reference, float measurement)
{
    float error = reference - measurement;

    if (firstRun)                       // İlk çalıştırmada iç değerleri matematiksel olarak işlenebilir hale getir
    {
        prevMeasurement = measurement;
        dFilter = 0.0f;
        firstRun = false;
    }

    dFilter += (dt / (tau + dt)) * (((measurement - prevMeasurement) / dt) - dFilter);  // Alçak geçirgen filtreden geçirilmiş türev
    float derivative = -Kd * dFilter;   // Türev kazancı

    float proportional = Kp * error;    // Oransal kazanç

    float unsatOutput = proportional + integrator + derivative; // Saturasyonsuz çıktı

    // Saturasyon koruması
    float output = unsatOutput;
    if (output > satMax)
        output = satMax;
    else if (output < satMin)
        output = satMin;

    integrator += (Ki * error + Kaw * (output - unsatOutput)) * dt; // İntegral kazancı, anti-windup koruması

    prevMeasurement = measurement;  // Sonraki döngü için güncelleme

    return output;
}

void PID::reset()   // Sıfırlama
{
    integrator = 0.0f;
    dFilter = 0.0f;
    prevMeasurement = 0.0f;
    firstRun = true;
}
