//
// Created by Barin A. Moghimi on 9/14/26.
//

#ifndef EKF_ATTITUDE_IMUSIMULATOR_H
#define EKF_ATTITUDE_IMUSIMULATOR_H

#include <random>
#include "EstimationConfig.h"
#include "VehicleSimulator.h"

struct ImuMeasurement {
    double time;

    Eigen::Vector3d angularRate;
    Eigen::Vector3d specificForce;
};

class ImuSimulator {
    public:
    ImuSimulator(const double dt,
                 const std::mt19937& rng,
                 std::normal_distribution<double>& normal,
                 Eigen::Vector3d biasGyro,
                 Eigen::Vector3d biasAccel,
                 EskfNoiseConfig& noiseConfig,
                 Eigen::Vector3d gravity
        );

    ImuMeasurement measure(const VehicleTruth& vehicle_truth);

    const Eigen::Vector3d& biasGyro() const;
    const Eigen::Vector3d& biasAccel() const;
    const double biasGyroRandomWalk() const;
    const double biasAccelRandomWalk() const;

    private:
    double dt_;
    std::mt19937 rng_;
    std::normal_distribution<double> normal_;
    Eigen::Vector3d biasGyro_;
    Eigen::Vector3d biasAccel_;
    double biasGyroRandomWalk_;
    double biasAccelRandomWalk_;
    double gyroNoiseDensity_;
    double accelNoiseDensity_;
    Eigen::Vector3d gravity_;
};

#endif //EKF_ATTITUDE_IMUSIMULATOR_H



