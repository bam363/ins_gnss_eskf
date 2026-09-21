//
// Created by Barin A. Moghimi on 9/14/26.
//

#include "InsMechanization.h"

InsMechanization::InsMechanization(const double dt,
                                   Eigen::Vector3d position,
                                   Eigen::Vector3d velocity,
                                   Eigen::Quaterniond attitude,
                                   Eigen::Vector3d biasAccel,
                                   Eigen::Vector3d biasGyro,
                                   Eigen::Vector3d gravity) :
                                dt_(dt),
                                ph_(position),
                                vh_(velocity),
                                qBIh_(attitude),
                                biasAccel_(biasAccel),
                                biasGyro_(biasGyro),
                                gravity_(gravity)
{};

InsData InsMechanization::propagate(const ImuMeasurement& imu) {

    InsData data;
    auto omegam = imu.angularRate;
    auto sfm = imu.specificForce;
    auto omegah = omegam - biasGyro_;
    auto sfh = sfm - biasAccel_;
    auto C_IB = qBIh_.toRotationMatrix().transpose();
    auto ah_I = C_IB * sfh + gravity_;

    ph_ = ph_ + dt_ * vh_ + 0.5 * dt_ * dt_ * ah_I;
    vh_ = vh_ + dt_ * ah_I;

    Eigen::Vector3d dtheta = omegah * dt_;
    double dthetam = dtheta.norm();
    if (dthetam > 0.0) {
        Eigen::Vector3d u = dtheta / dthetam;
        Eigen::Quaterniond dq(
            std::cos(dthetam / 2.0),
            -u.x() * std::sin(dthetam / 2.0),
            -u.y() * std::sin(dthetam / 2.0),
            -u.z() * std::sin(dthetam / 2.0)
        );
        qBIh_ = dq * qBIh_;
        qBIh_.normalize();
    }

    data.attitude = qBIh_;
    data.position = ph_;
    data.velocity = vh_;

    data.cSpecificForce = sfh;
    data.cAngularVelocity = omegah;
    data.C_IB = C_IB;

    return data;
}

InsData InsMechanization::injectCorrection(const Vector15d& dx, InsData& data) {

    auto dp = dx.segment<3>(0);
    auto dv = dx.segment<3>(3);
    auto dth = dx.segment<3>(6);
    auto dba = dx.segment<3>(9);
    auto dbg = dx.segment<3>(12);

    ph_ += dp;
    vh_ += dv;

    biasAccel_ += dba;
    biasGyro_ += dbg;

    // attitude correction
    double dthm = dth.norm();
    if (dthm > 0.0) {
        Eigen::Vector3d u = dth / dthm;
        Eigen::Quaterniond dq(
            std::cos(dthm / 2.0),
            -u.x() * std::sin(dthm / 2.0),
            -u.y() * std::sin(dthm / 2.0),
            -u.z() * std::sin(dthm / 2.0)
        );
        qBIh_ = dq * qBIh_;
        qBIh_.normalize();
    }

    data.attitude = qBIh_;
    data.position = ph_;
    data.velocity = vh_;

    data.biasAccel = biasAccel_;
    data.biasGyro = biasGyro_;

    return data;

}