//
// Created by Barin A. Moghimi on 9/15/26.
//

#ifndef EKF_ATTITUDE_INSESKF_H
#define EKF_ATTITUDE_INSESKF_H

#include "InsMechanization.h"
#include "EstimationTypes.h"
#include <Eigen/Dense>

#include "GnssSimulator.h"
#include "EstimationConfig.h"

class InsEskf {
public:
    InsEskf(
        const EskfNoiseConfig &noiseConfig,
        const EskfInitialCovariance &initialCovariance);

    void propagateCovariance(const InsData& ins,
                             double dt);

    Vector15d updateGnss(
        const GnssMeasurement& gnss,
        const GnssNoiseConfig& noiseConfig,
        const Eigen::Vector3d& p_hat,
        const Eigen::Vector3d& v_hat);

    const Eigen::Matrix<double, 15, 15>& covariance() const;

private:
    Eigen::Matrix<double, 15, 15> P_;
    Eigen::Matrix<double, 12, 12> Qc_;
    double accelNoiseDensity_;
    double gyroNoiseDensity_;
    double accelBiasRandomWalk_;
    double gyroBiasRandomWalk_;

};

#endif //EKF_ATTITUDE_INSESKF_H
