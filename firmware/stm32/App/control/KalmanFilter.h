#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include "Eigen/Dense"

class KalmanFilter {
public:
    KalmanFilter();

    // Initialize
    void init(const Eigen::Matrix<double, 13, 1>& x0,
              const Eigen::Matrix<double, 13, 13>& P0,
              const Eigen::Matrix<double, 13, 13>& A,
              const Eigen::Matrix<double, 13, 13>& Q,
              const Eigen::Matrix<double, 12, 13>& H,
              const Eigen::Matrix<double, 12, 12>& R);

    void predict();
    void update(const Eigen::Matrix<double, 12, 1>& z);

    // Mevcut durum
    Eigen::Matrix<double, 13, 1> getState() const;

private:
    Eigen::Matrix<double, 13, 1> x;   // State vector
    Eigen::Matrix<double, 13, 13> P;  // Covariance matrix
    Eigen::Matrix<double, 13, 13> A;  // State transition matrix
    Eigen::Matrix<double, 13, 13> Q;  // Process noise covariance
    Eigen::Matrix<double, 12, 13> H;  // Measurement matrix
    Eigen::Matrix<double, 12, 12> R;  // Measurement noise covariance
};

#endif
