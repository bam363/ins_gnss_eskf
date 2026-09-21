//
// Created by Barin A. Moghimi on 9/21/26.
//

#pragma once

#include <Eigen/Dense>

using Vector15d    = Eigen::Matrix<double, 15, 1>;
using Matrix15d    = Eigen::Matrix<double, 15, 15>;

using Vector6d     = Eigen::Matrix<double, 6, 1>;
using Matrix6d     = Eigen::Matrix<double, 6, 6>;

using Matrix6x15d  = Eigen::Matrix<double, 6, 15>;
using Matrix15x6d  = Eigen::Matrix<double, 15, 6>;

using Matrix12d    = Eigen::Matrix<double, 12, 12>;
using Matrix15x12d = Eigen::Matrix<double, 15, 12>;