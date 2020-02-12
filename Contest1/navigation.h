///////////////////////////////////////////////
///////////////////////////////////////////////////
//NOT READY FOR USE //////////////////////////////////
///////////////////////////////////////////////////////
////////////////////////////////////////////////////////////

#ifndef NAVIGATION
#define NAVIGATION

#include <chrono>

extern float MAX_ANGULAR, MAX_SPEED, linear, angular;

extern bool wallFollowON = false, wallFollowPrev = false; //Turn the wallFollow on or off by setting "wallFollowON" to either "true" (for "on") or "false"
extern float wallFollowSensorIn = 0.0;
extern float wallFollowLastErr;
std::chrono::time_point<std::chrono::system_clock> wallFollowLastTime;
std::chrono::time_point<std::chrono::system_clock> wallFollowCurrTime;

//--------------------------------------------------------------
//--------------- Wall following wallFollow ----------------------------------------------------------------------------------------------
//--------------------------------------------------------------
void WallFollow()
{
    //------Local variables
    float Output = 0.0, Setpoint = 0.0;
    uint64_t timeChange;
    float kp = 20.0, kd = 0.0;
    unsigned long SampleTime = 100; //milliseconds
    float outMax = MAX_ANGULAR, outMin = -MAX_ANGULAR;
    
    //------If wallFollow is in off state, do nothing------
    if(!wallFollowON)
    {
        wallFollowPrev = wallFollowON;
        angular = 0.0;
        linear = 0.0;
        return;
    }

    //------If wallFollow was previously off, reset history
    if(!wallFollowPrev)
    {
        wallFollowPrev = wallFollowON;
        wallFollowLastErr = 0.0;
        wallFollowLastTime = std::chrono::system_clock::now();
        timeChange = SampleTime;
    }
    else
    {
        //------Set up current time parameters------
        wallFollowCurrTime = std::chrono::system_clock::now();
        timeChange = std::chrono::duration_cast<std::chrono::microseconds>(wallFollowCurrTime - wallFollowLastTime).count();
        timeChange /= 1000; //Converts time to milliseconds
    }
    
    //------Start the computations------
    if(timeChange>=SampleTime) //Only compute new output at specified sample time
    {
        //Compute all the working error variables
        float error = Setpoint - wallFollowSensorIn;
        float dErr = (error - wallFollowLastErr) / (float)timeChange;
        float speedFactor;

        //Compute wallFollow Output
        Output = kp * error + kd * dErr; //**Question: how does this handle approaching a corner (object left and right)
        //ROS_INFO("wallFollowSensorIn %f wallFollowErrSum %f, dErr %f", wallFollowSensorIn, wallFollowErrSum, dErr);
        
        //Factor to limit linear speed if too close to an object
        speedFactor = fabs(Output) > outMax ? 0.3 : 1.0;

        if(Output > outMax) Output = outMax;
        else if(Output < outMin) Output = outMin;

       // ROS_INFO("output %f",Output);

        //Remember some variables for next time
        wallFollowLastErr = error;
        wallFollowLastTime = wallFollowCurrTime;
        
        //Finally, update velocities
        angular = Output;
        linear = MAX_SPEED * speedFactor;
    }
    return;
}


#endif