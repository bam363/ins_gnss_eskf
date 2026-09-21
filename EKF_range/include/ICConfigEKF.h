//
// Created by Barin A. Moghimi on 9/7/26.
//

#ifndef ESTIMATION_LAB_ICCONFIGEKF_H
#define ESTIMATION_LAB_ICCONFIGEKF_H

#include <Eigen/Dense>

struct ICConfigEKF {
    Eigen::Vector4d x0;
    Eigen::Matrix4d P0;
    Eigen::Vector4d xh0;
    Eigen::Vector2d anchor1;
    Eigen::Vector2d anchor2;
};

#endif //ESTIMATION_LAB_ICCONFIGEKF_H
