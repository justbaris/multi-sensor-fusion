#pragma once
#include <Eigen/Dense>

class GPSSensor {
public:
    Eigen::Vector3d readPosition();
};
