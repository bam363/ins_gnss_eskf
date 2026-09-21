//
// Created by Barin A. Moghimi on 9/7/26.
//

#ifndef ESTIMATION_LAB_MODELCONFIGEKF_H
#define ESTIMATION_LAB_MODELCONFIGEKF_H

#include <Eigen/Dense>

struct ModelConfigEKF {
    Eigen::Matrix4d F;
    // Eigen::Vector4d B;
    Eigen::Matrix4d Q;
    // Eigen::RowVector4<double> H;
    double R;
    double sigmaRange; // measurement noise on position
};

#endif //ESTIMATION_LAB_MODELCONFIGEKF_H
