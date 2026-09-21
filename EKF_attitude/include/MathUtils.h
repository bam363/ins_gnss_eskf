//
// Created by Barin A. Moghimi on 9/15/26.
//

#ifndef EKF_ATTITUDE_MATHUTILS_H
#define EKF_ATTITUDE_MATHUTILS_H

#include <Eigen/Dense>

inline Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
    Eigen::Matrix3d A;
    A << 0.0, -v.z(), v.y(),
         v.z(), 0.0, -v.x(),
        -v.y(), v.x(), 0.0;

    return A;
};

#endif //EKF_ATTITUDE_MATHUTILS_H
