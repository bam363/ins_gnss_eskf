//
// Created by Barin A. Moghimi on 8/10/26.
//
#include "KalmanFilter.hpp"

KalmanFilter::KalmanFilter(const ModelConfig& config,
                           const Eigen::Vector2d& x0,
                           const Eigen::Matrix2d& P0
                           )
                           : x_(x0),
                             P_(P0),
                             F_(config.F),
                             H_(config.H),
                             Q_(config.Q),
                             R_(config.R)
{};

void KalmanFilter::update(const double z) {
    v_ = z - (H_ * x_)(0);
    S_ = (H_ * P_ * H_.transpose())(0,0) + R_;
    Eigen::Vector2d K = P_ * H_.transpose() / S_;
    x_ = x_ + K * v_;

    Eigen::Matrix2d A = (Eigen::Matrix2d::Identity() - K * H_);
    P_ = A * P_ * A.transpose() + K * R_ * K.transpose();
    P_ = 0.5 * (P_ + P_.transpose());
}

void KalmanFilter::predict() {
    x_ = F_ * x_;
    P_ = F_ * P_ * F_.transpose() + Q_;
}

const Eigen::Vector2d& KalmanFilter::state() const {
    return x_;
}

const Eigen::Matrix2d& KalmanFilter::covariance() const {
    return P_;
}

const double KalmanFilter::innovation() const {
    return v_;
}

const double KalmanFilter::innovationCovariance() const {
    return S_;
}