//
// Created by Barin A. Moghimi on 8/8/26.
//

#ifndef KALMANFILTER_KALMANFILTER_H
#define KALMANFILTER_KALMANFILTER_H

#include <Eigen/Dense>

#include "ICConfig.h"
#include "../include/ModelConfig.h"

class KalmanFilter {
public:
    KalmanFilter(const ICConfig& icConfig,
                 const ModelConfig& config
                 );

    void predict(double am);
    void update(double z);

    const Eigen::Vector3d& state() const;
    const Eigen::Matrix3d& covariance() const;
    const double innovation() const;
    const double innovationCovariance() const;

private:
    Eigen::Vector3d x_;
    Eigen::Matrix3d P_;
    double v_;
    double S_;

    Eigen::Matrix3d F_;
    Eigen::RowVector3d H_;
    Eigen::Matrix3d Q_;
    Eigen::Vector3d B_;
    double R_;
};

#endif //KALMANFILTER_KALMANFILTER_H
