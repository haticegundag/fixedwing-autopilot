#pragma once
namespace atb::control {
struct PID {
  float kp = 0, ki = 0, kd = 0;
  float Update(float err, float dt);
};
}
