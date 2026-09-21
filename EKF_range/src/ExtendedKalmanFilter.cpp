//
// Created by Barin A. Moghimi on 9/6/26.
//

#include "../include/ExtendedKalmanFilter.h"

#include <cmath>

#include "ICConfigEKF.h"
#include "../include/ModelConfigEKF.h"

ExtendedKalmanFilter::ExtendedKalmanFilter(const ICConfigEKF& icConfig,
                                           const ModelConfigEKF& modelConfig) :
                                           x_(icConfig.xh0),
                                           P_(icConfig.P0),
                                           v_(0.0),
                                           S_(0.0),
                                           F_(modelConfig.F),
                                           // H_(modelConfig.H),
                                           Q_(modelConfig.Q),
                                           R_(modelConfig.R),
                                           anchor1_(icConfig.anchor1),
                                           anchor2_(icConfig.anchor2)
{}

double ExtendedKalmanFilter::predictRange(const Eigen::Vector2d& anchor) const
{
    const auto lx = anchor.x();
    const auto ly = anchor.y();
    const auto px = x_[0];
    const auto py = x_[1];
    const double rho = std::sqrt((px - lx)*(px - lx) + (py - ly) * (py - ly));

    return rho;
}

Eigen::Matrix<double, 1, 4>
ExtendedKalmanFilter::rangeJacobian(const Eigen::Vector2d& anchor) const
{
    Eigen::Matrix<double, 1, 4> H;
    const auto lx = anchor.x();
    const auto ly = anchor.y();
    const auto px = x_[0];
    const auto py = x_[1];
    auto rho = predictRange(anchor);
    H << (px - lx) / rho, (py - ly) / rho, 0.0, 0.0;

    return H;
}

void ExtendedKalmanFilter::updateRange(double z, const Eigen::Vector2d& anchor)
{
    double rho = predictRange(anchor);
    v_ = z - rho;
    auto H = rangeJacobian(anchor);
    auto S = H * P_ * H.transpose() + R_;
    S_ = S;
    auto K = P_ * H.transpose() / S;

    // Update state
    x_ = x_ + K * v_;

    // Update uncertainty
    Eigen::Matrix4d A = (Eigen::Matrix4d::Identity() - K * H);
    P_ = A * P_ * A.transpose() + K * R_ * K.transpose();
    P_ = 0.5 * (P_ + P_.transpose());
}

void ExtendedKalmanFilter::predict() {
    x_ = F_ * x_;
    P_ = F_ * P_ * F_.transpose() + Q_;
}

const Eigen::Vector4d& ExtendedKalmanFilter::state() const {
    return x_;
}

const Eigen::Matrix4d& ExtendedKalmanFilter::covariance() const {
    return P_;
}

const double ExtendedKalmanFilter::innovation() const {
    return v_;
}

const double ExtendedKalmanFilter::innovationCovariance() const {
    return S_;
}

const Eigen::Vector2d& ExtendedKalmanFilter::anchor1() const {
    return anchor1_;
}

const Eigen::Vector2d& ExtendedKalmanFilter::anchor2() const {
    return anchor2_;
}
