//
// Created by Barin A. Moghimi on 8/10/26.
//

#ifndef ESTIMATION_LAB_MONTECARLO_H
#define ESTIMATION_LAB_MONTECARLO_H

#include <vector>

#include "ICConfig.h"
#include "ModelConfig.h"
#include "MonteCarloResult.h"
#include "SimConfig.h"

class MonteCarlo {

public:
    MonteCarlo(std::size_t numTrials, unsigned int masterSeed);

    MonteCarloResults run(ModelConfig& modelConfig, ICConfig& ICconfig, SimConfig& simConfig);

private:
    std::vector<unsigned int> trialSeeds_;
    std::size_t numTrials_;
    unsigned int masterSeed_;
};

#endif //ESTIMATION_LAB_MONTECARLO_H
