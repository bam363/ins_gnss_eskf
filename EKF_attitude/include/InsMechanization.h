//
// Created by Barin A. Moghimi on 9/14/26.
//

#ifndef EKF_ATTITUDE_INSMECHANIZATION_H
#define EKF_ATTITUDE_INSMECHANIZATION_H

#include <Eigen/Dense>
#include "ImuSimulator.h"
#include "EstimationTypes.h"

struct InsData {
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::Quaterniond attitude;

    Eigen::Vector3d cSpecificForce;
    Eigen::Vector3d cAngularVelocity;
    Eigen::Matrix3d C_IB;

    Eigen::Vector3d biasAccel;
    Eigen::Vector3d biasGyro;
};

class InsMechanization {
    public:
    InsMechanization(const double dt,
                     Eigen::Vector3d position,
                     Eigen::Vector3d velocity,
                     Eigen::Quaterniond attitude,
                     Eigen::Vector3d biasAccel,
                     Eigen::Vector3d biasGyro,
                     Eigen::Vector3d gravity);

    InsData propagate(const ImuMeasurement& imu);

    InsData injectCorrection(const Vector15d& dx, InsData& data);

private:
    double dt_;
    Eigen::Vector3d ph_;
    Eigen::Vector3d vh_;
    Eigen::Quaterniond qBIh_;
    Eigen::Vector3d biasAccel_;
    Eigen::Vector3d biasGyro_;
    Eigen::Vector3d gravity_;
};


#endif //EKF_ATTITUDE_INSMECHANIZATION_H
