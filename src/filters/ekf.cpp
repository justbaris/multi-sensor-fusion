#include "filters/ekf.hpp"
#include <cmath>

EKF::EKF() {
    x = Eigen::VectorXd::Zero(6); 
    P = Eigen::MatrixXd::Identity(6,6) * 0.1;

    // Small process noise by default
    Q = Eigen::MatrixXd::Zero(6,6);
    double pos_q = 1e-3;   // position noise
    double vel_q = 1e-2;   // velocity noise
    Q.block<3,3>(0,0) = Eigen::Matrix3d::Identity() * pos_q;
    Q.block<3,3>(3,3) = Eigen::Matrix3d::Identity() * vel_q;

    // Measurement noise (GPS)
    R = Eigen::Matrix3d::Identity() * 0.5; // tune as needed

    // Measurement matrix: z = H * x  (z is position)
    H.setZero();
    H.block<3,3>(0,0) = Eigen::Matrix3d::Identity();
    H.block<3,3>(0,3) = Eigen::Matrix3d::Zero();
}

void EKF::predict(const Eigen::Vector3d& accel, double dt) {
    if (dt <= 0) return;

    // State transition matrix A
    Eigen::Matrix<double,6,6> A = Eigen::Matrix<double,6,6>::Identity();
    A.block<3,3>(0,3) = Eigen::Matrix3d::Identity() * dt;

    // Control input matrix B: maps acceleration to state
    Eigen::Matrix<double,6,3> B = Eigen::Matrix<double,6,3>::Zero();
    B.block<3,3>(0,0) = Eigen::Matrix3d::Identity() * 0.5 * dt * dt; // position contribution
    B.block<3,3>(3,0) = Eigen::Matrix3d::Identity() * dt;           // velocity contribution

    // Predict state
    x = A * x + B * accel;

    // Predict covariance
    P = A * P * A.transpose() + Q;
}

void EKF::update(const Eigen::Vector3d& gps) {
    // Innovation
    Eigen::Vector3d z = gps;
    Eigen::Vector3d z_pred = H * x;
    Eigen::Vector3d y = z - z_pred;

    Eigen::Matrix3d S = H * P * H.transpose() + R;
    Eigen::Matrix<double,6,3> K = P * H.transpose() * S.inverse();

    x = x + K * y;
    Eigen::Matrix<double,6,6> I = Eigen::Matrix<double,6,6>::Identity();
    P = (I - K * H) * P;
}

Eigen::Vector3d EKF::getStatePos() const {
    return x.head<3>();
}

Eigen::VectorXd EKF::getState() const {
    return x;
}