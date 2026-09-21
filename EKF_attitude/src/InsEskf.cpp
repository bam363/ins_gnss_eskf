//
// Created by Barin A. Moghimi on 9/15/26.
//

#include "../include/InsEskf.h"
#include "MathUtils.h"

InsEskf::InsEskf(
            const EskfNoiseConfig &noiseConfig,
            const EskfInitialCovariance &initialCovariance) {

    Vector15d variances;
    variances <<
        Eigen::Vector3d::Constant(initialCovariance.position * initialCovariance.position),
        Eigen::Vector3d::Constant(initialCovariance.velocity * initialCovariance.velocity),
        Eigen::Vector3d::Constant(initialCovariance.attitude * initialCovariance.attitude),
        Eigen::Vector3d::Constant(initialCovariance.biasAccel * initialCovariance.biasAccel),
        Eigen::Vector3d::Constant(initialCovariance.biasGyro * initialCovariance.biasGyro);

    P_ = variances.asDiagonal();

    accelNoiseDensity_ = noiseConfig.accelNoiseDensity;
    gyroNoiseDensity_= noiseConfig.gyroNoiseDensity;
    accelBiasRandomWalk_= noiseConfig.accelBiasRandomWalk;
    gyroBiasRandomWalk_= noiseConfig.gyroBiasRandomWalk;

    const Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();

    Qc_.setZero();

    Qc_.block<3,3>(0,0) =
        std::pow(accelNoiseDensity_, 2) * I3;

    Qc_.block<3,3>(3,3) =
        std::pow(gyroNoiseDensity_, 2) * I3;

    Qc_.block<3,3>(6,6) =
        std::pow(accelBiasRandomWalk_, 2) * I3;

    Qc_.block<3,3>(9,9) =
        std::pow(gyroBiasRandomWalk_, 2) * I3;

};

void InsEskf::propagateCovariance(
                                  const InsData& ins,
                                  double dt) {

    // C_IB
    auto C_IB = ins.C_IB;
    auto f_hat = ins.cSpecificForce;
    auto omega_hat = ins.cAngularVelocity;

    // F Matrix
    Eigen::Matrix<double, 15, 15> F =
        Eigen::Matrix<double, 15, 15>::Zero();

    const Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();

    F.block<3,3>(0,3) = I3;
    F.block<3,3>(3,6) = -C_IB * skew(f_hat);
    F.block<3,3>(3,9) = -C_IB;
    F.block<3,3>(6,6) = -skew(omega_hat);
    F.block<3,3>(6,12) = -I3;

    // G Matrix
    Eigen::Matrix<double, 15, 12> G =
        Eigen::Matrix<double, 15, 12>::Zero();

    G.block<3,3>(3,0) = -C_IB;  // accel measurement noise
    G.block<3,3>(6,3) = -I3;    // gyro measurement noise
    G.block<3,3>(9,6) =  I3;    // accel bias random walk
    G.block<3,3>(12,9) = I3;    // gyro bias random walk

    // Q Matrix
    const Matrix15d Qd =
        G * Qc_ * G.transpose() * dt;

    // Propagate P
    using Matrix15d = Eigen::Matrix<double, 15, 15>;
    const Matrix15d I15 = Matrix15d::Identity();
    const Matrix15d Phi = I15 + F * dt;
    P_ = Phi * P_ * Phi.transpose() + Qd;
    // Symmetrize
    P_ = 0.5 * (P_ + P_.transpose());

}

Vector15d InsEskf::updateGnss(
    const GnssMeasurement& gnss,
    const GnssNoiseConfig& noiseConfig,
    const Eigen::Vector3d& p_hat,
    const Eigen::Vector3d& v_hat) {

    Matrix6x15d H = Matrix6x15d::Zero();

    H.block<3,3>(0,0) =  Eigen::Matrix3d::Identity();
    H.block<3,3>(3,3) = Eigen::Matrix3d::Identity();

    // Form residual
    Vector6d r;
    r.segment<3>(0) = gnss.position - p_hat;
    r.segment<3>(3) = gnss.velocity - v_hat;

    // Measurement covariance
    Matrix6d R = Matrix6d::Zero();
    R.block<3,3>(0,0) = Eigen::Matrix3d::Identity() * noiseConfig.positionStd *
                                        noiseConfig.positionStd;
    R.block<3,3>(3,3) = Eigen::Matrix3d::Identity() * noiseConfig.velocityStd *
                                        noiseConfig.velocityStd;

    // Innovation covariance
    Matrix6d S = H * P_ * H.transpose()  + R;

    // Kalman Gain
    Matrix15x6d PHt = P_ * H.transpose();
    Matrix15x6d K =
        S.ldlt().solve(PHt.transpose()).transpose();

    // Correction
    Vector15d dx = K * r;

    // Update uncertainty
    Matrix15d A = Matrix15d::Identity() - K * H;
    P_ = A * P_ * A.transpose() + K * R * K.transpose();

    P_ = 0.5 * (P_ + P_.transpose());

    return dx;
}