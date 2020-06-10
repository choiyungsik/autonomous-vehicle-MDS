
#ifndef STATEMACHINE_H_
#define STATEMACHINE_H_

#include <ros/ros.h>
#include <iostream>
#include <math.h>
#include <vector>
#include <map>

using namespace std;

class StateBase
{
    private:
        enum mode{
            cruise,
            avoidance,
            stop,
            traffic,
            parking,
            safetyzone
        };
        int cur_mode;


    public:
        //virtual void InvokeStateAction(State);


};

#endif 