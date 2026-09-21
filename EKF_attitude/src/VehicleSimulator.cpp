//
// Created by Barin A. Moghimi on 9/12/26.
//

#include "VehicleSimulator.h"
#include <cmath>

VehicleSimulator::VehicleSimulator(double dt,
                                   const Eigen::Vector3d& p0,
                                   const Eigen::Vector3d& v0,
                                   const Eigen::Quaterniond& q0,
                                   const Eigen::Vector3d& aTrue,
                                   const Eigen::Vector3d& omegaTrue) :
                                   dt_(dt),
                                   pTrue_(p0),
                                   vTrue_(v0),
                                   qTrue_(q0),
                                   aTrue_(aTrue),
                                   omegaTrue_(omegaTrue)
{};

VehicleTruth VehicleSimulator::step(double time) {
    // Truth propagation
    pTrue_ = pTrue_ + dt_ * vTrue_ + 0.5 * dt_ * dt_ * aTrue_;
    vTrue_ = vTrue_ + dt_ * aTrue_;
    Eigen::Vector3d dtheta = omegaTrue_ * dt_;
    double dthetam = dtheta.norm();

    if (dthetam > 0.0) {
        Eigen::Vector3d u = dtheta / dthetam;
        Eigen::Quaterniond dq(
            std::cos(dthetam / 2.0),
            -u.x() * std::sin(dthetam / 2.0),
            -u.y() * std::sin(dthetam / 2.0),
            -u.z() * std::sin(dthetam / 2.0)
        );
        qTrue_ = dq * qTrue_;
        qTrue_.normalize();
    }

    VehicleTruth truth = {};
    truth.position = pTrue_;
    truth.velocity = vTrue_;
    truth.acceleration = aTrue_;
    truth.attitude = qTrue_;
    truth.angularRate = omegaTrue_;

    truth.time = time;
    return truth;
}

const Eigen::Quaterniond& VehicleSimulator::quaternion() const {
    return qTrue_;
}

const Eigen::Vector3d& VehicleSimulator::position() const {
    return pTrue_;
}

const Eigen::Vector3d& VehicleSimulator::velocity() const {
    return vTrue_;
}

const Eigen::Vector3d& VehicleSimulator::acceleration() const {
    return aTrue_;
}

const Eigen::Vector3d& VehicleSimulator::angularRate() const {
    return omegaTrue_;
}

