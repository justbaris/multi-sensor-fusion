#pragma once
#include <Eigen/Dense>

class IMUSensor {
public:
    Eigen::Vector3d readAccelerometer();
    Eigen::Vector3d readGyroscope();
};
