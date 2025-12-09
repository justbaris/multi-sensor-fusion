#include "sensors/gps.hpp"
#include <random>

static std::default_random_engine gen2;
static std::normal_distribution<double> gpsNoise(0.0, 0.5);

Eigen::Vector3d GPSSensor::readPosition() {
    static double x = 0, y = 0, z = 0;
    x += 0.1;
    y += 0.05;

    return {x + gpsNoise(gen2), y + gpsNoise(gen2), z + gpsNoise(gen2)};
}
