//
// Created by Barin A. Moghimi on 9/17/26.
//

#include "GnssSimulator.h"

GnssSimulator::GnssSimulator(const std::normal_distribution<>& normal,
                  const std::mt19937& rng,
                  const GnssNoiseConfig& config) :
                  normal_(normal),
                  rng_(rng)
{
    positionStd_ = config.positionStd;
    velocityStd_ = config.velocityStd;
};

GnssMeasurement GnssSimulator::measure(const VehicleTruth& truth) {

    GnssMeasurement measurement;

    measurement.time = truth.time;

    Eigen::Vector3d positionNoise(
        normal_(rng_),
        normal_(rng_),
        normal_(rng_));

    Eigen::Vector3d velocityNoise(
        normal_(rng_),
        normal_(rng_),
        normal_(rng_));

    measurement.position =
        truth.position + positionStd_ * positionNoise;

    measurement.velocity =
        truth.velocity + velocityStd_ * velocityNoise;

    return measurement;
}