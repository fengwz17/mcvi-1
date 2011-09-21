#ifndef __SOLVER_H
#define __SOLVER_H

#include "Model.h"
#include "ParticlesBeliefSet.h"
#include "BeliefTree.h"
#include "RandSource.h"
#include "PolicyGraph.h"
#include "Bounds.h"
#include "Simulator.h"
#include <iostream>
#include <sstream>
#include <cstdlib>
#include <ctime>

class Solver {
  public:
    Solver(): maxTime(3600), discount(0.95), targetPrecision(0.1),
              numBackUpStreams(100), numNextBeliefStreams(100),
              maxSimulLength(100), iterDeepMult(0.95),
              useMacro(0), seed(0), displayInterval(60)
    {}

    void input(int argc, char **argv, int noRequiredArgs);
    void solve(Model& currModel, std::vector<State> initialBeliefStates, std::vector<long> pathLength);

    std::ostringstream message;
    std::string policy_file;
    unsigned maxTime;
    double discount;
    double targetPrecision;
    long numBackUpStreams;
    long numNextBeliefStreams;
    long maxSimulLength;
    double iterDeepMult;
    long useMacro;
    unsigned seed;
    long displayInterval;
};

#endif
