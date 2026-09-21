//
// Created by Barin A. Moghimi on 9/21/26.
//


#pragma once

struct EskfInitialCovariance {
    double position;
    double velocity;
    double attitude;
    double biasAccel;
    double biasGyro;
};

struct EskfNoiseConfig {
    double accelNoiseDensity;
    double gyroNoiseDensity;
    double accelBiasRandomWalk;
    double gyroBiasRandomWalk;
};