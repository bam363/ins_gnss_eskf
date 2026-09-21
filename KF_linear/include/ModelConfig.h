//
// Created by Barin A. Moghimi on 8/11/26.
//

#ifndef ESTIMATION_LAB_MODELCONFIG_H
#define ESTIMATION_LAB_MODELCONFIG_H

#include <Eigen/Dense>

struct ModelConfig {
    Eigen::Matrix3d F;
    Eigen::Vector3d B;
    Eigen::Matrix3d Q;
    Eigen::RowVector3<double> H;
    double R;
    double sigma_z; // measurement noise on position
    double sigma_accel; // per sample standard deviations
    double sigma_bias; // per sample standard deviations
};

#endif //ESTIMATION_LAB_MODELCONFIG_H
