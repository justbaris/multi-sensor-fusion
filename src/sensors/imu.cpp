#include "sensors/imu.hpp"
#include <random>

static std::default_random_engine gen;
static std::normal_distribution<double> noise(0.0, 0.1);

Eigen::Vector3d IMUSensor::readAccelerometer() {
    return {1.0 + noise(gen), 0.5 + noise(gen), 0.0 + noise(gen)};
}

Eigen::Vector3d IMUSensor::readGyroscope() {
    return {0.01 + noise(gen), 0.02 + noise(gen), 0.0 + noise(gen)};
}
