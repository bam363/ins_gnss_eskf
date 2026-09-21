//
// Created by Barin A. Moghimi on 9/12/26.
//

#ifndef EKF_ATTITUDE_SIMULATORATTITUDE_H
#define EKF_ATTITUDE_SIMULATORATTITUDE_H

#include <random>
#include <Eigen/Geometry>

struct VehicleTruth {
    double time;

    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::Quaterniond attitude;

    Eigen::Vector3d acceleration;
    Eigen::Vector3d angularRate;
};

class VehicleSimulator {
    public:
    VehicleSimulator(double dt,
                     const Eigen::Vector3d& p0,
                     const Eigen::Vector3d& v0,
                     const Eigen::Quaterniond& q0,
                     const Eigen::Vector3d& aTrue,
                     const Eigen::Vector3d& omegaTrue
                    );

    VehicleTruth step(double time);

    const Eigen::Vector3d& position() const;
    const Eigen::Vector3d& velocity() const;
    const Eigen::Quaterniond& quaternion() const;

    const Eigen::Vector3d& acceleration() const;
    const Eigen::Vector3d& angularRate() const;

private:
    double dt_;

    Eigen::Vector3d pTrue_;
    Eigen::Vector3d vTrue_;
    Eigen::Quaterniond qTrue_;

    Eigen::Vector3d aTrue_;
    Eigen::Vector3d omegaTrue_;
};

#endif //EKF_ATTITUDE_SIMULATORATTITUDE_H
