#include "sensors/imu.hpp"
#include "sensors/gps.hpp"
#include "filters/ekf.hpp"
#include <iostream>
#include <thread>
#include <chrono>

int main() {
    IMUSensor imu;
    GPSSensor gps;
    EKF ekf;

    // Basit bir loop; gerçek sistemde dt gerçek timestamp'ten hesaplanmalı
    double dt = 0.2; // 200 ms (main loopunuzda sleep ile eşlenik)

    for (int i = 0; i < 200; i++) {
        auto accel = imu.readAccelerometer(); // Eigen::Vector3d
        auto pos   = gps.readPosition();      // Eigen::Vector3d

        // Predict using accel and dt
        ekf.predict(accel, dt);

        // Optionally: update when GPS is available (simulate dropout)
        // Here we update every loop for simplicity
        ekf.update(pos);

        auto est = ekf.getStatePos();

        std::cout << "GPS  : " << pos.transpose() << "\n";
        std::cout << "EST  : " << est.transpose() << "\n";
        std::cout << "-------------------------\n";

        std::this_thread::sleep_for(std::chrono::milliseconds((int)(dt*1000)));
    }

    return 0;
}
