//
// Created by Barin A. Moghimi on 9/6/26.
//

#ifndef ESTIMATION_LAB_EXTENDEDKALMANFILTER_H
#define ESTIMATION_LAB_EXTENDEDKALMANFILTER_H

#include <Eigen/Dense>
#include "../include/ICConfigEKF.h"
#include "ModelConfigEKF.h"

class ExtendedKalmanFilter {

public:
    ExtendedKalmanFilter(const ICConfigEKF& icConfig,
                         const ModelConfigEKF& config);

    double predictRange(const Eigen::Vector2d& anchor) const;
    void updateRange(double z, const Eigen::Vector2d& anchor);

    void predict();

    Eigen::Matrix<double, 1, 4>
    rangeJacobian(const Eigen::Vector2d& anchor) const;

    const Eigen::Vector4d& state() const;
    const Eigen::Matrix4d& covariance() const;
    const double innovation() const;
    const double innovationCovariance() const;
    const Eigen::Vector2d& anchor1() const;
    const Eigen::Vector2d& anchor2() const;


private:
    Eigen::Vector4d x_;
    Eigen::Matrix4d P_;
    double v_;
    double S_;

    Eigen::Matrix4d F_;
    Eigen::Matrix4d Q_;
    // Eigen::Vector4d H_;
    double R_;
    Eigen::Vector2d anchor1_;
    Eigen::Vector2d anchor2_;

};


#endif //ESTIMATION_LAB_EXTENDEDKALMANFILTER_H
