#ifndef __ACTNODE_H
#define __ACTNODE_H

#include "Action.h"
#include <map>

class ObsEdge;
class Belief;
class Bounds;

class ActNode
{
  public:
    ActNode(const Action& action, const Belief& belief, Bounds* bounds);

    /**
       Back up on this ActNode
    */
    void backup();

    /**
       Sample observations and create corresponding ObsEdge.

       Each ObsEdge will have a number of particles corresponding to
       the probability of getting the observation.
    */
    void generateObsPartitions();

    /**
       Remove all the generated particles from all the ObsEdge
    */
    void clearObsPartitions();

    // The label of this ActNode
    const Action action;
    // The belief representation of the Belief father of this ActNode
    const Belief& belief;
    Bounds* bounds;
    // The average lower (upper) bounds compute by dividing the total
    // lower (upper) bounds of all ObsEdge children by the number of
    // such edges
    double avgLower, avgUpper;
    // The randSeed for generateing ObsEdge, so if we redo the
    // generation, it will be the same
    unsigned long randSeed;
    std::map<Obs, ObsEdge> obsChildren;
};

#endif
