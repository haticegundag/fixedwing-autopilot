#include "AttitudeEstimator.h"
#include <math.h>

#define ALPHA 0.98f
#define SAMPLE_COUNT 200

using namespace atabey::utils;

namespace atabey {
    namespace estimation {

        AttitudeEstimator::AttitudeEstimator(atabey::drivers::ImuSensor& imuSensor) : imu(&imuSensor) {}

        bool AttitudeEstimator::init() {
            roll = pitch = yaw = 0.0f;
            pitchAcc = rollAcc = 0.0f;

            for (int i = 0; i < 10; i++) {
                sampleSum += imu->update();
                delay(100);
            }
            sample = sampleSum * SAMPLE_COUNT; // Ortalama alınarak ilk örnek değeri hesaplanır
            return true;
        }

        void AttitudeEstimator::update() {
            Vec3f accel = normalize(imu->getAccel()); // Akselometre verilerini normalize ederek kullanıyoruz
            Vec3f gyro = imu->getGyro();

            prevMicros = micros();
            nowMicros = micros();

            dt = (nowMicros - prevMicros) / 1000000.0f; // Saniyeye dönüştürmek için
            prevMicros = nowMicros;

            if (dt <= 0.0f) { dt = 0.01f; } // dt'nin sıfır veya negatif gelmesi durumunu engelledik
            else if (dt > 0.1f) { dt = 0.1f; }

            pitchAcc = atan2f(-accel.x, sqrtf(accel.y * accel.y + accel.z * accel.z)); // Pitch açısını hesaplamak için akselometre verilerini kullandık
            rollAcc  = atan2f(accel.y, accel.z); // Roll açısını hesaplamak için akselometre verilerini kullandık

            roll = lerp(rollAcc, roll + gyro.x * dt, ALPHA);
            pitch = lerp(pitchAcc, pitch + gyro.y * dt, ALPHA);
            yaw   += gyro.z * dt;

            // Açıları -180..180 aralığında tutar
            roll  = wrapPi(roll);
            pitch = wrapPi(pitch);
            yaw   = wrapPi(yaw);
        }

        Vec3f AttitudeEstimator::getAttitude() const {
            return {roll, pitch, yaw};
        }

    }
}