#include "KalmanFilter.h"

void KalmanFilter::init(const Eigen::Matrix<double, 13, 1>& x0,
                             const Eigen::Matrix<double, 13, 13>& P0,
                             const Eigen::Matrix<double, 13, 13>& A_,
                             const Eigen::Matrix<double, 13, 13>& Q_,
                             const Eigen::Matrix<double, 12, 13>& H_,
                             const Eigen::Matrix<double, 12, 12>& R_) {
    x = x0;
    P = P0;
    A = A_;
    Q = Q_;
    H = H_;
    R = R_;
}

KalmanFilter::KalmanFilter() {
    x.setZero();
    P.setIdentity();
    A.setIdentity();
    Q.setIdentity();
    H.setZero();
    R.setIdentity();
}

void KalmanFilter::predict() {
    x = A * x;
    P = A * P * A.transpose() + Q;
}

void KalmanFilter::update(const Eigen::Matrix<double, 12, 1>& z) {
    Eigen::Matrix<double, 12, 1> y = z - H * x;  // Innovation
    Eigen::Matrix<double, 12, 12> S = H * P * H.transpose() + R; // Innovation covariance
    Eigen::Matrix<double, 13, 12> K = P * H.transpose() * S.inverse(); // Kalman gain
    x = x + K * y;
    P = (Eigen::Matrix<double, 13, 13>::Identity() - K * H) * P;
}

Eigen::Matrix<double, 13, 1> KalmanFilter::getState() const {
    return x;
}

