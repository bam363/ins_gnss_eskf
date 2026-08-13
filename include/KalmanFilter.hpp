//
// Created by Barin A. Moghimi on 8/8/26.
//

#ifndef KALMANFILTER_KALMANFILTER_H
#define KALMANFILTER_KALMANFILTER_H

#include <Eigen/Dense>
#include "ModelConfig.h"

class KalmanFilter {
public:
    KalmanFilter(const ModelConfig& config,
                 const Eigen::Vector2d& x0,
                 const Eigen::Matrix2d& P0
                 );

    void predict();
    void update(double z);

    const Eigen::Vector2d& state() const;
    const Eigen::Matrix2d& covariance() const;
    const double innovation() const;
    const double innovationCovariance() const;

private:
    Eigen::Vector2d x_;
    Eigen::Matrix2d P_;
    double v_;
    double S_;

    Eigen::Matrix2d F_;
    Eigen::RowVector2d H_;
    Eigen::Matrix2d Q_;
    double R_;

};

#endif //KALMANFILTER_KALMANFILTER_H
