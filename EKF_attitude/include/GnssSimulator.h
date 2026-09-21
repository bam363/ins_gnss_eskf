//
// Created by Barin A. Moghimi on 9/17/26.
//

#ifndef EKF_ATTITUDE_GNSSSIMULATOR_H
#define EKF_ATTITUDE_GNSSSIMULATOR_H

#include <Eigen/Dense>
#include "VehicleSimulator.h"

struct GnssMeasurement {
    double time;
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
};

struct GnssNoiseConfig {
    double positionStd;
    double velocityStd;
};

class GnssSimulator {

public:
    GnssSimulator(const std::normal_distribution<>& normal,
                  const std::mt19937& rng,
                  const GnssNoiseConfig& config);

    GnssMeasurement measure(const VehicleTruth& truth);

private:
    std::normal_distribution<> normal_;
    std::mt19937 rng_;
    double positionStd_;
    double velocityStd_;
};


#endif //EKF_ATTITUDE_GNSSSIMULATOR_H
