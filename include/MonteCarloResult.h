//
// Created by Barin A. Moghimi on 8/11/26.
//

#ifndef ESTIMATION_LAB_MONTECARLORESULT_H
#define ESTIMATION_LAB_MONTECARLORESULT_H

struct MonteCarloResults {
    std::vector<unsigned int> trialSeeds;

    std::vector<double> trialMeanNEES;
    double meanNEES;
    double stdNEES;
    double minNEES;
    double maxNEES;

    std::vector<double> trialMeanNIS;
    double meanNIS;
    double stdNIS;
    double minNIS;
    double maxNIS;
};

#endif //ESTIMATION_LAB_MONTECARLORESULT_H
