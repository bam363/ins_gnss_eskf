//
// Created by Barin A. Moghimi on 9/14/26.
//


#include "ImuSimulator.h"

ImuSimulator::ImuSimulator(const double dt,
                const std::mt19937& rng,
                std::normal_distribution<double>& normal,
                Eigen::Vector3d biasGyro,
                Eigen::Vector3d biasAccel,
                EskfNoiseConfig& noiseConfig,
                Eigen::Vector3d gravity
                ) :
                dt_(dt),
                rng_(rng),
                normal_(normal),
                biasGyro_(biasGyro),
                biasAccel_(biasAccel),
                biasGyroRandomWalk_(noiseConfig.gyroBiasRandomWalk),
                biasAccelRandomWalk_(noiseConfig.accelBiasRandomWalk),
                gyroNoiseDensity_(noiseConfig.gyroNoiseDensity),
                accelNoiseDensity_(noiseConfig.accelNoiseDensity),
                gravity_(gravity)
{};

ImuMeasurement ImuSimulator::measure(const VehicleTruth &vehicle_truth) {

    ImuMeasurement measurement = {};
    auto omega = vehicle_truth.angularRate;
    auto q = vehicle_truth.attitude;
    auto accel = vehicle_truth.acceleration;
    measurement.time = vehicle_truth.time;

    // 1. Generate measurement using CURRENT biases b_k
    // Gyro
    Eigen::Vector3d zeta1(
    normal_(rng_),
    normal_(rng_),
    normal_(rng_)
    );
    const double sigmaGyro = gyroNoiseDensity_ / std::sqrt(dt_);
    auto n1 = sigmaGyro * zeta1;
    measurement.angularRate = omega + biasGyro_ + n1;

    // Accelerometer
    auto C_BI = q.toRotationMatrix();
    Eigen::Vector3d zeta2(
   normal_(rng_),
   normal_(rng_),
   normal_(rng_)
    );
    const double sigmaAccel = accelNoiseDensity_ / std::sqrt(dt_);
    auto n2 = sigmaAccel * zeta2;
    measurement.specificForce = C_BI * (accel - gravity_) + biasAccel_ + n2;

    // 2. Evolve biases for NEXT sample
    biasAccel_ += biasAccelRandomWalk_ * std::sqrt(dt_) *
        Eigen::Vector3d(
            normal_(rng_),
            normal_(rng_),
            normal_(rng_));

    biasGyro_ += biasGyroRandomWalk_ * std::sqrt(dt_) *
        Eigen::Vector3d(
            normal_(rng_),
            normal_(rng_),
            normal_(rng_));

    return measurement;
}

const Eigen::Vector3d& ImuSimulator::biasAccel() const {
    return biasAccel_;
}

const Eigen::Vector3d& ImuSimulator::biasGyro() const {
    return biasGyro_;
}

const double ImuSimulator::biasAccelRandomWalk() const {
    return biasAccelRandomWalk_;
}

const double ImuSimulator::biasGyroRandomWalk() const {
    return biasGyroRandomWalk_;
}