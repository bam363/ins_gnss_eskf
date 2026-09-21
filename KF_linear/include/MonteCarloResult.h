//
// Created by Barin A. Moghimi on 8/11/26.
//

#ifndef ESTIMATION_LAB_MONTECARLORESULT_H
#define ESTIMATION_LAB_MONTECARLORESULT_H

#include <vector>

struct MonteCarloResults {
    std::vector<unsigned int> trialSeeds;

    std::vector<double> trialMeanNEES;
    std::vector<double> ensembleMeanNEES;
    std::vector<double> ensemblePosNEES;
    std::vector<double> ensembleVelNEES;
    std::vector<double> ensembleBiasNEES;
    double meanNEES;
    double stdNEES;
    double minNEES;
    double maxNEES;

    std::vector<double> trialMeanNIS;
    double meanNIS;
    double stdNIS;
    double minNIS;
    double maxNIS;

    std::vector<Eigen::Matrix3d> empiricalUncertainty;
    std::vector<Eigen::Matrix3d> theoreticalUncertainty;
};

#endif //ESTIMATION_LAB_MONTECARLORESULT_H
