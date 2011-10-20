#ifndef __BOUNDS_H
#define __BOUNDS_H

#include "Utils.h"
#include <map>
#include <list>

class Model;
class Belief;
class PolicyGraph;
class RandSource;

/**
   @class Bounds
   @brief Implements the backup operation.
   @details Both upper and lower bound backups are performed by the backup
   operation. Upper bound backup is done by sampling a set of starting
   points from the belief. For each action, a child each sampled for each
   starting point and the upper bounds at all the children are used for
   computing the backed up bound. For lower bound, for each action,
   a trajectory is sampled from each starting point using each of the
   current policy graph nodes as the starting point at the child of the
   action. Best current bounds and policy graph nodes that have been tested
   are stored in the Belief class to reduce computation.

   @author Wee Sun Lee
   @date 22 August 2009
*/
class Bounds
{
  public:
    /**
       @param[in] model Provides sampling functions
       @param[in, out] policyGraph Provides policy graph data structure
       @param[in] simulator Provides simulation from a starting point
       @param[in] randSource Source of random number
       @param[in] numRandStreams Number of starting points to be sampled
       @param[in] maxSimulLength Simulation length for each simulation
       @param[in] maxMacroActLength Max number of steps allowed for any
       macro action before returning control to the policy graph
       @param[in] cacheSize size of cache for storing particles at children
    */
    // make singleton instance for this class
    Bounds(Model& model, PolicyGraph& policyGraph,
           RandSource& randSource,
           long numRandStreams, long maxSimulLength):
            model(model), policyGraph(policyGraph),
            randSource(randSource),
            numRandStreams(numRandStreams),
            maxSimulLength(maxSimulLength)
    {}

    ~Bounds() {}

    /**
       Run backup for upper and lower bound.
       @param belief Belief to be backed up.
    */
    void backUp(Belief& belief);
    void updateBestActions(Belief& belief);

    /** Backup operations for each type of actions. Reset the count (number of particles) to 0
     */

    void backUpInitPolicies(Belief& belief);
    void backUpActions(Belief& belief);

    void buildActNodes(Belief& belief);
    void initBeliefForBackUp(Belief& belief);

    Model& model;
    PolicyGraph& policyGraph;
    RandSource& randSource;
    long numRandStreams;
    long maxSimulLength;
};

#endif //__BOUNDS_H
