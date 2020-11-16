#include "px4Model.h"
#include "Include.h"
#include "ParticlesBelief.h"
#include <cassert>
#include <cmath>
#include <cstdlib>
#include <deque>

#include <iostream>

using namespace std;

// int indoor(double pos) {
//     int i=0;
//     for(i=0;i<NumDoors;++i)
//         if(fabs(DoorPositions[i]-pos)<=DoorRadius)
//             return i;
//     return -1;
// }

// rotate theta by clockwise
// x' = xcos(theta) + ysin(theta)
// y' = -xsin(theta) + ycos(theta)
int DirectionObs(double x1, double y1, double theta1, double x2, double y2)
{
    double x = x2 - x1;
    double y = y2 - y1;
    double rx, ry;
    rx = x*cos(theta1) + y*sin(theta1);
    ry = -x*sin(theta1) + y*cos(theta1);

    // obsNothing
    if (rx < 0)
        return 0;
    else
    {
        if (rx*rx + ry*ry > detect*detect)
            return 0;
        else
        {
            if(ry > 0)
                return 1; // intruder is on the left of ownship
            else    
                return 2; // right
        }
    }
}

int NMAC(double x1, double y1, double x2, double y2)
{
    if (((x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1)) <= collisionDis * collisionDis)
        return 1;
    else 
        return 0;
}
// To simplify the problem, first assume that there is no noise

// /* add noise to the real value,
//  * eg. a true value has 0.05 chance become false
//  */
// //TODO check this, this seems incorrect
// //this is correct
// inline bool noisy(bool v)
// {
//     return (v ^ (randf()<Noise));
// }

// #define NOISY(v) ((v) ^ (randStream->getf()<Noise))

px4Model::px4Model(int numParticles):
        Model(NumStateVars, NumObsVars, NumActs, NumMacroActs, NumInitPolicies, Discount),
        numParticles(numParticles)
{
}

bool px4Model::allowableAct(Belief const& belief, Action const& action)
{
    if (action.type == Macro) return false;

    return true;
}

double px4Model::sample(State const& currState, Action const& action, State* nextState, Obs* obs, RandStream* randStream )
{
    double reward = 0;

    // double randReal = randStream->getf();

    // double intruderAngle;
    double x1, y1, theta1, x2, y2, theta2;

    theta1 = currState[3];
    x1 = currState[1];
    y1 = currState[2];
    x2 = currState[4]; 
    y2 = currState[5];

    if (x2 > x1)
        theta2 = atan( (y2 - y1)  / (x2 - x1) ) + M_PI;
    else
        theta2 = atan( (y2 - y1)  / (x2 - x1) );
   

    if (action.getActNumUser() == GoAhead)
    {
        // double x1, y1, theta1, x2, y2, theta2;
        theta1 = currState[3];
        x1 = currState[1] + ut * cos(theta1);
        y1 = currState[2] + ut * sin(theta1);
        // double randReal = randStream->getf();
        // cout << "r1: " << randReal << endl;
        // if (randReal > 2/3)
        //     intruderAngle = -M_PI / 6;
        // else if (randReal <= 2/3 && randReal > 1/3)
        //     intruderAngle = 0;
        // else
        // {
        //     if (randReal < 0 || randReal > 1/3)
        //         cout << "Wrong randReal" << endl;
        //     intruderAngle = M_PI / 6;
        // }
        

 
        // theta2 = currState[6] + intruderAngle;
        x2 = currState[4] + ut * cos(theta2);
        y2 = currState[5] + ut * sin(theta2);

        (*nextState)[0] = 0;
        (*nextState)[1] = x1;
        (*nextState)[2] = y1;
        (*nextState)[3] = theta1;
        (*nextState)[4] = x2;
        (*nextState)[5] = y2;
        (*nextState)[6] = theta2;

        if(fabs(x1 - 30) <= 1)
        {
            if(fabs(y1 - 0) <= 1)
            {
                (*nextState)[0] = TermState;
                obs->obs[0] = TermObs;
                return 100.0;
            }
        }
        
        if (NMAC(x1, y1, x2, y2) == 1)
        {
            // obs->obs[0] = Crash;
            (*nextState)[0] = TermState;
            obs->obs[0] = TermObs;
            // reward = CollisionPenalty;
            return -100.0;
        }
        else
        {
            if (DirectionObs(x1, y1, theta1, x2, y2) == 0)
            {
                obs->obs[0] = ObsNothing;
                reward = 0;
            }
            else if (DirectionObs(x1, y1, theta1, x2, y2) == 1)
            {
                obs->obs[0] = Left;
                reward = 0;
            }
            else
            {
                obs->obs[0] = Right;
                reward = 0;
            }
        }
    }

    else
    {
        assert(action.getActNumUser() == TurnLeft || action.getActNumUser() == TurnRight);
        // double x1, y1, theta1, x2, y2, theta2;
        theta1 = currState[3];
        if (action.getActNumUser() == TurnLeft)
            theta1 = currState[3] + (M_PI / 6);
        else if (action.getActNumUser() == TurnRight)
            theta1 = currState[3] - (M_PI / 6);
        x1 = currState[1] + ut * cos(theta1);
        y1 = currState[2] + ut * sin(theta1);

        // double randReal = randStream->getf();
        // cout << "r2: " << randReal << endl;
        // if (randReal > 2/3)
        //     intruderAngle = -M_PI / 6;
        // else if (randReal <= 2/3 && randReal > 1/3)
        //     intruderAngle = 0;
        // else
        // {
        //     if (randReal < 0 || randReal > 1/3)
        //         cout << "Wrong randReal" << endl;
        //     intruderAngle = M_PI / 6;
        // }

        // theta2 = currState[6] + intruderAngle;
        x2 = currState[4] + ut * cos(theta2);
        y2 = currState[5] + ut * sin(theta2);

        (*nextState)[0] = 0;
        (*nextState)[1] = x1;
        (*nextState)[2] = y1;
        (*nextState)[3] = theta1;
        (*nextState)[4] = x2;
        (*nextState)[5] = y2;
        (*nextState)[6] = theta2;

        if(fabs(x1 - 30) <= 1)
        {
            if(fabs(y1 - 0) <= 1)
            {
                (*nextState)[0] = TermState;
                obs->obs[0] = TermObs;
                return 100.0;
            }
        }
        
        if (NMAC(x1, y1, x2, y2) == 1)
        {
            // obs->obs[0] = Crash;
            // reward = CollisionPenalty;
             (*nextState)[0] = TermState;
            obs->obs[0] = TermObs;
            // reward = CollisionPenalty;
            return -100.0;
        }
        else
        {
            if (DirectionObs(x1, y1, theta1, x2, y2) == 0)
            {
                obs->obs[0] = ObsNothing;
                reward = MovementCost;
            }
            else if (DirectionObs(x1, y1, theta1, x2, y2) == 1)
            {
                obs->obs[0] = Left;
                reward = MovementCost;
            }
            else
            {
                obs->obs[0] = Right;
                reward = MovementCost;
            }
        }
    }

    return reward;
}

double px4Model::sample(State const& currState, Action const& macroAction, long controllerState, State* nextState, long* nextControllerState, Obs* obs, RandStream* randStream)
{
    // should never reach here
    assert(false);
    return 0;
}


double px4Model::initPolicy(State const& currState, Action const& initAction, long controllerState, State* nextState, long* nextControllerState, Obs* dummy, RandStream* randStream)
{
    Obs obs(vector<long>(getNumObsVar(), 0));
    if (currState[0] < 0){
        (*nextState) = currState;
        return 0;
    };

    return sample(currState, Action(Act,GoAhead), nextState, &obs, randStream);
}

// initial state
// In corridor problem, it is a random number between - 21 and 21
State px4Model::sampleInitState() const
{
    double x1 = 0;
    double y1 = 0;
    double theta1 = 0;

    double x2 = randf();
    double y2 = randf();
    double theta2 = M_PI;
    x2 = x2 * distanceRange;
    y2 = y2 * distanceRange * 2 - distanceRange;
    double dis = (x2 - x1) *(x2 - x1) + (y2 - y1)*(y2 - y1);

    cout << "x2: " << x2 << " y2: " << y2 << endl;
    cout << "dis" << sqrt(dis) << endl;
    while(dis <= collisionDis * collisionDis)
    {
        x2 = randf();
        y2 = randf();
        x2 = x2 * distanceRange;
        y2 = y2 * distanceRange * 2 - distanceRange;
        dis =  (x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1);
        cout << "resample-----" << endl;
        cout << "x2: " << x2 << " y2: " << y2 << endl;
        cout << "dis" << sqrt(dis) << endl;
    }

    // cout << "x2: " << x2 << endl;
    // cout << "y2: " << y2 << endl;
    // cout << "theta2: " << theta2 << endl;
// y2 = y2 * distanceRange * 2 - distanceRange;
//    // theta2 = theta2 * M_PI  +  (1/2) * M_PI;
//    theta2 = M_PI;
   
//     double dis = (x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1);
//     while(dis > distanceRange * distanceRange || dis <= 2 * 2)
//     {
//         y2 = randf() * distanceRange * 2 - distanceRange;
//         dis =  (x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1);
//     }
    // p = p*CorridorLength*2 - CorridorLength ;
    // p = p*AltitudeLength*2 - AltitudeLength;
    // double x2 = 20;
    // double y2 = 0;
    // double theta2 = M_PI;
    State st(getNumStateVar(), 0);
    // st[1] = p;
    st[1] = x1;
    st[2] = y1;
    st[3] = theta1;
    st[4] = x2;
    st[5] = y2;
    st[6] = theta2;
    return st;
}

/* Should be a tight upper bound, right? */
double px4Model::upperBound(State const& state)
{
    // double minSteps = fabs(state[1] - DoorPositions[0]);
    // double reward = pow(discount, minSteps) * EnterReward;
    double mreward = pow(discount, 28) * 100.0;
    return mreward;
}


double px4Model::getObsProb(Action const& action, State const& nextState, Obs const& obs)
{
    // double nxtPos = nextState[1];
    // double  ownAltitude = nextState[0];
    // double intruderAltitude = nextState[1];
    // double dis = intruderAltitude - ownAltitude;

    // int ndoor = indoor(nxtPos);

    if(obs.obs[0] == TermObs)
    {
        if(nextState[0] == TermState)
            return 1.0;
        else
            return 0.0;
    }
/*
    else if(obs.obs[0] == ObsCrash) 
    {
        if(nxtPos<=0 || nxtPos>=CorridorLength)
            return 1.0;
        else
            return 0.0;
    }else if(obs.obs[0]==ObsDoor) {
        if(ndoor>=0)
            return 1-Noise;
        else
            return 0;
    }else if(obs.obs[0]==ObsLeftEnd) {
        if(nxtPos<=-CorridorLength+CorridorEndLength)
            return 1;
        else
            return 0;
    }else if(obs.obs[0]==ObsRightEnd) {
        if(nxtPos>=CorridorLength-CorridorEndLength)
            return 1;
        else
            return 0;
    } else {
        assert(obs.obs[0]==ObsNothing);
        if(nxtPos<=-CorridorLength || nxtPos>=CorridorLength)
            return 0.0;
        else if(ndoor>=0)
            return Noise;
        else if((nxtPos<=-CorridorLength+CorridorEndLength) ||
                (nxtPos>=CorridorLength-CorridorEndLength))
            return 0;
        else
            return 1.0;
    }
*/

    double x1 = nextState[1];
    double y1 = nextState[2];
    double theta1 = nextState[3];
    double x2 = nextState[4];
    double y2 = nextState[5];

    if (obs.obs[0] == ObsNothing)
    {
        if (NMAC(x1, y1, x2, y2) == 1)
            return 0.0;
        else
        {
            if (DirectionObs(x1, y1, theta1, x2, y2) == 0)
                return 1.0;
            else 
                return 0.0;
        }
    }
    // else if (obs.obs[0] == Crash)
    // {
    //     if (NMAC(x1, y1, x2, y2) == 1)
    //         return 1.0;
    //     else 
    //         return 0.0;
    // }
    else if (obs.obs[0] == Left)
    {
        if (NMAC(x1, y1, x2, y2) == 1)
            return 0.0;
        else
        {
            if (DirectionObs(x1, y1, theta1, x2, y2) == 1)
                return 1.0;
            else
                return 0.0;
        }
    }
    else
    {
        assert(obs.obs[0] == Right);
        if (NMAC(x1, y1, x2, y2) == 1)
            return 0.0;
        else
        {
            if (DirectionObs(x1, y1, theta1, x2, y2) == 2)
                return 1.0;
            else
                return 0.0;
        }
    }
}

void px4Model::displayState(State state, long type)
{
    cout << "x1: " << state[1] << " y1: " << state[2] << " theta1: " << state[3] << endl;
    cout << "x2: " << state[4] << " y2: " << state[5] << " theta2: " << state[6] << endl;
}
