//
// Created by Barin A. Moghimi on 8/11/26.
//

#ifndef ESTIMATION_LAB_ICCONFIG_H
#define ESTIMATION_LAB_ICCONFIG_H

#include <Eigen/Dense>

struct ICConfig {
    Eigen::Vector2d x0;
    Eigen::Matrix2d P0;
    Eigen::Vector2d xh0;
};

#endif //ESTIMATION_LAB_ICCONFIG_H
