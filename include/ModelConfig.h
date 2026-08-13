//
// Created by Barin A. Moghimi on 8/11/26.
//

#ifndef ESTIMATION_LAB_MODELCONFIG_H
#define ESTIMATION_LAB_MODELCONFIG_H

#include <Eigen/Dense>

struct ModelConfig {
    Eigen::Matrix2d F;
    Eigen::Matrix2d Q;
    Eigen::RowVector2<double> H;
    double R;
    double sigma_z;
};

#endif //ESTIMATION_LAB_MODELCONFIG_H
