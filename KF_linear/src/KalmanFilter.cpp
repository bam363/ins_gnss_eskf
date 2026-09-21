//
// Created by Barin A. Moghimi on 8/10/26.
//
#include "../include/KalmanFilter.hpp"

KalmanFilter::KalmanFilter(const ICConfig& icConfig,
                           const ModelConfig& modelConfig
) : x_(icConfig.xh0),
    P_(icConfig.P0),
    v_(0.0),
    S_(0.0),
    F_(modelConfig.F),
    H_(modelConfig.H),
    Q_(modelConfig.Q),
    B_(modelConfig.B),
    R_(modelConfig.R)
{};

void KalmanFilter::update(const double z) {
    v_ = z - (H_ * x_)(0);
    S_ = (H_ * P_ * H_.transpose())(0,0) + R_;
    Eigen::Vector3d K = P_ * H_.transpose() / S_;
    x_ = x_ + K * v_;

    Eigen::Matrix3d A = (Eigen::Matrix3d::Identity() - K * H_);
    P_ = A * P_ * A.transpose() + K * R_ * K.transpose();
    P_ = 0.5 * (P_ + P_.transpose());
}

void KalmanFilter::predict(const double am) {
    x_ = F_ * x_ + B_ * am;
    P_ = F_ * P_ * F_.transpose() + Q_;
}

const Eigen::Vector3d& KalmanFilter::state() const {
    return x_;
}

const Eigen::Matrix3d& KalmanFilter::covariance() const {
    return P_;
}

const double KalmanFilter::innovation() const {
    return v_;
}

const double KalmanFilter::innovationCovariance() const {
    return S_;
}