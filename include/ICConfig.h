//
// Created by Barin A. Moghimi on 8/11/26.
//

#ifndef ESTIMATION_LAB_ICCONFIG_H
#define ESTIMATION_LAB_ICCONFIG_H

#include <Eigen/Dense>

struct ICConfig {
    Eigen::Vector3d x0;
    Eigen::Matrix3d P0;
    Eigen::Vector3d xh0;
};

#endif //ESTIMATION_LAB_ICCONFIG_H
