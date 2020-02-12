#ifndef MOVEMENT
#define MOVEMENT

#include "utility.h"

//declare global variables

extern float linear, angular, posX, posY, MAX_SPEED, MAX_ANGULAR, yaw_deg;

//declare functions
bool TurnAngle(float initialAng, float targetOffset);
bool MoveDistance(float initialX, float initialY, float targetDistance);


//--------------------------------------------------------------
//------------------ Travel A Desired Distance ----------------------------------------------------------------------------------
//--------------------------------------------------------------
//Move some fixed distance fwd/bckwd
bool MoveDistance(float initialX, float initialY, float targetDistance)
{
    float distanceTraveled = PointsDistance(initialX, posX, initialY, posY);
    float distanceToGo = fabs(targetDistance) - distanceTraveled;

    //Forward or backward depending on sign
    linear = targetDistance > 0.0 ? MAX_SPEED : -MAX_SPEED;
    angular = 0.0; 
    
    return distanceToGo <= 0.0 ? true : false;
}

//--------------------------------------------------------------
//------------------ Turn "X" Degrees -------------------------------------------------------------------------------------------
//--------------------------------------------------------------
//Turn the robot a fixed number of degrees from some initial orientation
//+ cw - ccw
bool TurnAngle(float initialAng, float targetOffset)
{
    float currentOffset, angleToTarget, turnError = 5.0;
    float turnSpeed = 1.0;

    // sets direction variable for later math
    int direction = targetOffset > 0 ? 1 : -1;
    
    currentOffset = yaw_deg - initialAng;// current angle turned calculation
    if (currentOffset * direction < 0.1) // value of "< 0.1" there to avoid errors (ask sergio)
    {
        currentOffset += 360 * direction;// correct angle overflow
    }

    angleToTarget = targetOffset - currentOffset;// remaining angle
    ROS_INFO("%f angletotarget, %f targetoffset, %f currentOffset, %f yaw", angleToTarget, targetOffset, currentOffset, yaw_deg);
    if (fabs(angleToTarget) < turnError)// ask if remaining angle is less than error
    {
        angular = 0.0;
        ROS_INFO("done turning!");
        return true; // desired angle reached
    }
    else
    {
        angular = MAX_ANGULAR * direction * turnSpeed;// set angular speed
        linear = 0.0;
        return false;
    }
}


#endif