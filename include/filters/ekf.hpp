#pragma once
#include <Eigen/Dense>

class EKF {
public:
    EKF();

    // Predict step: accel in world frame, dt in seconds
    void predict(const Eigen::Vector3d& accel, double dt);

    // Update step: gps gives position (px,py,pz)
    void update(const Eigen::Vector3d& gps);

    // Current estimated position (px,py,pz)
    Eigen::Vector3d getStatePos() const;

    // (Optional) full state getter
    Eigen::VectorXd getState() const;

private:
    Eigen::VectorXd x;   // 6x1 state
    Eigen::MatrixXd P;   // 6x6 covariance
    Eigen::MatrixXd Q;   // 6x6 process noise
    Eigen::MatrixXd R;   // 3x3 measurement noise

    Eigen::Matrix<double,3,6> H; // measurement matrix
};
