#include <ros/console.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <iostream>

#include "functions/utility.h"
#include "functions/movement.h"

#include <stdio.h>
#include <cmath>
#include <math.h>

#include <chrono>

//Functions for converting angles
#define RAD2DEG(rad)((rad) * 180./M_PI)
#define DEG2RAD(deg)((deg) * M_PI/180.)


enum robotState
{
    TurnAndScan,
    TurnToTheta,
    AutoPilot,
    BumperHit,
    CornerFound
};

//-------------------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------------
//--------------------------------- GLOBAL VARIABLES ----------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------------

#define N_BUMPER (3)

//Global state variables for robot speed
float angular = 0.0;
float linear = 0.0;

//Global position arrays
const int histSize = 3;
float posHistX[histSize] = {0.0}, posHistY[histSize] = {0.0};

//Odometry state values for robot position and orientation
float posX=0.0, posY=0.0, yaw=0.0, yaw_deg;

//Maximum speeds for the robot
float MAX_SPEED = 0.15;
float MAX_ANGULAR = 0.45;

//Time for turn and scan
float initialScanTime = 10.0; // Scan time at first (decreases with time)
float finalScanTime = 45.0; // scan time at end
float scanTime = initialScanTime; // timer
float scanTimeSeconds = 300.0; // how long to reach finalScanTime

//Variables for laserCallBack
//minStraightLaserDist used to find minimum distance directly in front of TurtleBot
float minStraightLaserDist = std::numeric_limits<float>::infinity();
int32_t nLasers=0, desiredNLasers=0, desiredStraightAngle=1;
float leftDist = 0.0, rightDist = 0.0, survRad = 0.0;

//Array for storing bumper states
uint8_t bumper[3] = {kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED};

//PID VARIABLES
bool pidON = false, pidPrev = false; //Turn the PID on or off by setting "pidON" to either "true" (for "on") or "false"
float pidSensorIn = 0.0;
float pidErrSum, pidLastErr;
std::chrono::time_point<std::chrono::system_clock> pidLastTime;
std::chrono::time_point<std::chrono::system_clock> pidCurrTime;

//-------------------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------------
//--------------------------------- FUNCTIONS -----------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------------

//--------------------------------------------------------------
//------------------ Bumper Callback --------------------------------------------------------------------------------------------
//--------------------------------------------------------------
//Store bumper states
void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg)
{
    bumper[msg->bumper] = msg->state;
}

//--------------------------------------------------------------
//------------------ Laser Callback ---------------------------------------------------------------------------------------------
//--------------------------------------------------------------
//Store in minStraightLaserDist the straight-ahead distance
void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    //reset minStraightLaserDist to large number
    minStraightLaserDist = std::numeric_limits<float>::infinity();
    
    //number of lasers is angle of view divided by increment/resolution
    nLasers = (msg->angle_max - msg->angle_min)/msg->angle_increment;

    //desired number of lasers to consider
    desiredNLasers = DEG2RAD(desiredStraightAngle)/msg->angle_increment;

    //finds the straight ahead distance for the robot
    //if desired angle range is in the sensor range, use the desired range
    if(desiredStraightAngle*M_PI/180 < msg->angle_max && -desiredStraightAngle*M_PI/180 > msg->angle_min)
    {
    //find edge of desired angle range, look through all angles in range and find minimum distance
    //nLasers/2 is the middle of our range i.e straight ahead
        for(uint32_t laser_idx = nLasers/2 - desiredNLasers; laser_idx < nLasers/2 + desiredNLasers; ++laser_idx)
        {
            minStraightLaserDist = std::min(minStraightLaserDist, msg->ranges[laser_idx]);
        }
    }
    else //this is if the desired range is somehow larger than the available range
    {
        for (uint32_t laser_idx = 0;laser_idx<nLasers; ++laser_idx)
        {
            minStraightLaserDist = std::min(minStraightLaserDist, msg->ranges[laser_idx]);
        }
    }
    
    //If inf
    if (minStraightLaserDist == std::numeric_limits<float>::infinity())
    {
        minStraightLaserDist = -1.0;
    }

    //PID relevant laser distance variables (assuming distances are in meters)
    float botDiameter = 0.4; //354 millimeters diameter
    float surveyRadius = 0.8; //Radius at which the bot will react if an object is sensed in
    int x = (int)((asin(botDiameter/(2 * surveyRadius)))/msg->angle_increment);

    //Calculating sensor input for PID
    //In case of NaN, set as 0.1 (very close)
    float rightMeas = msg->ranges[(nLasers/2) + x] == msg->ranges[(nLasers/2) + x] ? msg->ranges[(nLasers/2) + x] : 0.1; //Maybe include some digital filter on this or averaging algorithm
    float leftMeas = msg->ranges[(nLasers/2) - x] == msg->ranges[(nLasers/2) - x] ? msg->ranges[(nLasers/2) - x] : 0.1;
    leftDist = leftMeas;
    rightDist = rightMeas;
    survRad = surveyRadius;
    
    if(rightMeas > surveyRadius) rightMeas = surveyRadius;
    if(leftMeas > surveyRadius) leftMeas = surveyRadius;
    pidSensorIn = leftMeas - rightMeas; // "0" if nothing sensed, "positive" if object on right, "negative" if object on left
    //ROS_INFO("%f", msg->ranges[(nLasers/2) - x]);
    //ROS_INFO("rightMeas %f, leftMeas %f, x %d, msg len %d", rightMeas, leftMeas, x, msg->ranges.size());
}

//--------------------------------------------------------------
//------------------ Odometer Callback ------------------------------------------------------------------------------------------
//--------------------------------------------------------------
//Store odometer values
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    posX = msg->pose.pose.position.x;
    posY = msg->pose.pose.position.y;
    yaw = tf::getYaw(msg->pose.pose.orientation);
    yaw_deg = RAD2DEG(yaw);
}

//----------------------------------------------------------
//------------------CHECK IF HEADING TO CORNER--------------
//--------------------------------------------------------------


bool CornerCheck(float leftD, float rightD, float straightD, float survR)
{
    if(leftD < survR && rightD < survR && straightD < survR && fabs(leftD-rightD) < 0.1 
    && straightD > leftD && straightD > rightD && straightD > 0.0)
    {
        //if left, right and straight distances less than survey radius and difference is small
        //between laser distances,and straight bigger than left or right, then heading into a corner
        ROS_INFO("Heading to Corner");
        return true;
    }
    else
    {
        return false;
    }

}


//--------------------------------------------------------------
//------------------ Autopilot PID ----------------------------------------------------------------------------------------------
//--------------------------------------------------------------
void pidAutoPilot()
{
    //------Local variables
    float Output = 0.0, Setpoint = 0.0;
    uint64_t timeChange;
    float kp = 20.0, ki = 0.0, kd = 0.0;
    unsigned long SampleTime = 100; //milliseconds
    float outMax = MAX_ANGULAR, outMin = -MAX_ANGULAR;
    
    //------If PID is in off state, do nothing------
    if(!pidON)
    {
        pidPrev = pidON;
        angular = 0.0;
        linear = 0.0;
        return;
    }

    //------If PID was previously off, reset history
    if(!pidPrev)
    {
        pidPrev = pidON;
        pidErrSum = 0.0;
        pidLastErr = 0.0;
        pidLastTime = std::chrono::system_clock::now();
        timeChange = SampleTime;
    }
    else
    {
        //------Set up current time parameters------
        pidCurrTime = std::chrono::system_clock::now();
        timeChange = std::chrono::duration_cast<std::chrono::microseconds>(pidCurrTime - pidLastTime).count();
        timeChange /= 1000; //Converts time to milliseconds
    }
    
    //------Start the computations------
    if(timeChange>=SampleTime) //Only compute new output at specified sample time
    {
        //Compute all the working error variables
        float error = Setpoint - pidSensorIn;
        pidErrSum += error * (float)timeChange;
        float dErr = (error - pidLastErr) / (float)timeChange;
        float speedFactor;

        //Compute PID Output
        Output = kp * error + ki * pidErrSum + kd * dErr; //**Question: how does this handle approaching a corner (object left and right)
        //ROS_INFO("pidSensorIn %f pidErrSum %f, dErr %f", pidSensorIn, pidErrSum, dErr);
        
        //Factor to limit linear speed if too close to an object
        speedFactor = fabs(Output) > outMax ? 0.3 : 1.0;

        if(Output > outMax) Output = outMax;
        else if(Output < outMin) Output = outMin;

       // ROS_INFO("output %f",Output);

        //Remember some variables for next time
        pidLastErr = error;
        pidLastTime = pidCurrTime;
        
        //Finally, update velocities
        angular = Output;
        linear = MAX_SPEED * speedFactor;
    }
    return;
}

//--------------------------------------------------------------
//------------------ Store Laser Distance Values --------------------------------------------------------------------------------
//--------------------------------------------------------------
//Add current global yaw and laser distance to vectors
void StoreLaser(std::vector<float> &yawValues, std::vector<float> &distanceValues)
{
    //ROS_INFO("About to store laser %f", minStraightLaserDist);
    if(minStraightLaserDist != std::numeric_limits<float>::infinity())
    {
        yawValues.push_back(yaw_deg);
        distanceValues.push_back(minStraightLaserDist);
        //ROS_INFO("Stored laser");
    }
}

//--------------------------------------------------------------
//------------------ Check Bumper Status ----------------------------------------------------------------------------------------
//--------------------------------------------------------------
//Are any bumpers currently hit?
bool BumperStatus()
{
    bool anyBumperPressed = false;
    for(uint32_t b_idx = 0; b_idx < N_BUMPER; ++b_idx)
    {
        anyBumperPressed |= (bumper[b_idx] == kobuki_msgs::BumperEvent::PRESSED);
    }
    return anyBumperPressed;
}

//--------------------------------------------------------------
//------------------ Find Optimal Next Direction --------------------------------------------------------------------------------
//--------------------------------------------------------------
float DirectionFinder(std::vector<float> spin_theta, std::vector<float> spin_dist)
{
    float max;
    int i, int_max = 0;
    if(spin_dist.size() < 1) return 189.0;
    max = spin_dist[0];
    for(i=1;i<spin_dist.size();i++)
    {
        ROS_INFO("%f theta, %f, distance", spin_theta[i], spin_dist[i]);
        if(spin_dist[i] > max)
        {
            max = spin_dist[i];
            int_max = i;
        }
    }
    ROS_INFO("%f theta max, %f distance max", spin_theta[int_max], spin_dist[int_max]);
    return spin_theta[int_max];
}



//--------------------------------------------------------------
//------------------ Direction Finder 2.0 --------------------------------------------------------------------------------
//--------------------------------------------------------------
//Decide next direction
float NextDirection(float initialYaw, std::vector<float> spin_theta, std::vector<float> spin_dist, float ignoreRange)
{
    float max, maxAngle, minAngle;// 60 on each side
    
    //Determine angles to ignore, which are angles within a range, from the direction we came
    minAngle = AngleOverflow(initialYaw + (180.0 - ignoreRange));
    maxAngle = AngleOverflow(minAngle + (2.0 * ignoreRange));

    int i, int_max = 0;
    if(spin_dist.size() < 1) return 189.0;
    max = spin_dist[0];
    for(i=1;i<spin_dist.size();i++)
    {
        //ROS_INFO("%f theta, %f, distance", spin_theta[i], spin_dist[i]);
        if(spin_dist[i] > max && !(spin_theta[i] < maxAngle && spin_theta[i] > minAngle))
        {
            max = spin_dist[i];
            int_max = i;
        }
    }
    ROS_INFO("%f theta max, %f distance max", spin_theta[int_max], spin_dist[int_max]);
    ROS_INFO("Initial yaw: %f, Max Deg Ignored %f, Min Deg Ignored %f", initialYaw, maxAngle, minAngle);
    return spin_theta[int_max];
}


/*
//--------------------------------------------------------------
//------------------ Direction Finder 3.0 --------------------------------------------------------------------------------
//--------------------------------------------------------------
float DirectionFinder_beta(std::vector<float> spin_theta, std::vector<float> spin_dist)
{
    int i;
    int spinSize = spin_dist.size();

    if(spin_dist.size() < 1) return 189.0; //If vectors empty, return impossible value

    float xCoor[histSize], yCoor[histSize], x_sum = 0, y_sum = 0, theta;
    for(i=0;i<histSize;i++)
    {
        //Set vector origins to be current position
        xCoor[i] = posHistX[i] - posX;
        yCoor[i] = posHistY[i] - posY;
        
        //Sum vectors (turn them into unit vectors, then divide each unit vector by the magnitude of each original vector before adding [this will put more weight onto vectors that had shorter magnitudes, meaning we want to prioritize getting away from places we have most recently been first])
        x_sum += xCoor[i]/((xCoor[i]*xCoor[i])+(yCoor[i]*yCoor[i]));
        y_sum += yCoor[i]/((xCoor[i]*xCoor[i])+(yCoor[i]*yCoor[i]));
    }   
    
    //Adjust tan^-1 to make it account for a full 360 cartesian plane, then set the desired angle 180 degrees away from sum of previous angles
    if( (x_sum <= 0) && (y_sum >= 0))
    {
        theta = (180 + ((atan(y_sum/x_sum))*(180/3.14159))) + 180;
    }
    else if( (x_sum <= 0) && (y_sum <= 0))
    {
        theta = ((atan(y_sum/x_sum))*(180/3.14159));
    }
    else
    {
        theta = ((atan(y_sum/x_sum))*(180/3.14159)) + 180;
    }
    //theta is now from 0 - 360 on the global coordinate frame
     
    if (theta >= 180.0) theta -= 360.0;
     
    //theta is now adjusted for the global coordinate frame
    
    //Find spin_theta element closest to the desired theta
    i=0;
    while( (i < spinSize) && (fabs(spin_theta[i] - theta) > 5.0) )
    {
        i++;
    }
    
    int j = 1, goodIndex = i;
    float good = spin_dist[i];
    while(good < 0.8 && j < spinSize / 2) //Arbitrary distance (we want to robot to have at least 0.5 to travel ahead of us [or something similar])
    {
        //std::cout << "Head in this direction: " << i+(k*j) << "\n";
        if(i + j >= spinSize)
        {
            goodIndex = (spin_dist[i+j-spinSize] > spin_dist[i-j]) ? i + j : i - j;
        }
        else if(i - j < 0)
        {
            goodIndex = (spin_dist[i+j] > spin_dist[spinSize+(i-j)]) ? i + j : i - j;
        }
        else
        {
            goodIndex = (spin_dist[i+j] > spin_dist[i-j]) ? i + j : i - j;
        }
        
        good = spin_dist[goodIndex];
        
        j++;
    }
    
    //std::cout << "Head in this direction: " << m << "\n";
    std::cout << "Head in this direction: " << spin_theta[goodIndex] << "\n";

    return spin_theta[goodIndex];
}

*/

//--------------------------------------------------------------
//------------------ Update Travel History -----------------------------------------------------------------------------------------------
//--------------------------------------------------------------
void UpdateHistory()
{
    int temp;

    for(temp = 0; temp < histSize-1; temp++)
    {
       posHistX[temp] = posHistX[temp+1];
    }
    posHistX[histSize-1] = posX;

    for(temp = 0; temp < histSize-1; temp++)
    {
       posHistY[temp] = posHistY[temp+1];
    }
    posHistY[histSize-1] = posY;
}

//--------------------------------------------------------------
//------------------ Update State -----------------------------------------------------------------------------------------------
//--------------------------------------------------------------
//Function to update robot state and necessary state variables
void UpdateState(robotState targetState, robotState &currentState, robotState &prevState,
                  float &initialTurnYaw, std::vector<float> &yawValues,
                  std::vector<float> &distanceValues, float &initialX, float &initialY)
{
    prevState = currentState; 
    if(targetState == TurnAndScan)
    {
        //Clears vectors to store laser scan data
        currentState = targetState;
        ROS_INFO("About to clear");
        yawValues.clear();
        distanceValues.clear();
        ROS_INFO("Cleared");
        initialTurnYaw = yaw_deg;
        ROS_INFO("Set initial turn yaw okay");
    }
    else if(targetState == TurnToTheta)
    {
        currentState = targetState;
        initialTurnYaw = yaw_deg;
    }
    else if(targetState == AutoPilot)
    {
        currentState = targetState;
        pidON = true;
    }
    else if((targetState == BumperHit) || (targetState == CornerFound))
    {
        //Initial X and Y for reversing
        currentState = targetState;
        initialX = posX;
        initialY = posY;
    }
}




//-------------------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------------
//--------------------------------- MAIN ----------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------------

int main(int argc, char **argv)
{
    ros::init(argc, argv, "maze_explorer");
    ros::NodeHandle nh;

    //Sensor subscribers
    ros::Subscriber bumper_sub = nh.subscribe("mobile_base/events/bumper", 10, &bumperCallback);
    ros::Subscriber laser_sub = nh.subscribe("scan", 10, &laserCallback);
    ros::Subscriber odom = nh.subscribe("odom",1, &odomCallback);

    //Velocity publisher
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);

    ros::Rate loop_rate(10);

    geometry_msgs::Twist vel;

    //Contest count down timer
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;
    uint64_t lastTurnTime = 0;

    //Set initial status
    robotState currentState = TurnAndScan;
    robotState prevState = TurnAndScan; 

    //Vectors to store theta values and laser distances
    //Used while completing 360 deg turn
    std::vector<float> yawValues;
    std::vector<float> distanceValues;

    //Float to store current target direction of exploration
    float targetYaw;

    //SpinOnce to get the current Yaw value before starting the program
    ros::spinOnce();
    float initialTurnYaw = yaw_deg;
    float initialX = posX;
    float initialY = posY;




    //-------------------------------------------------------------------------
    //---------------------- MAIN LOOP ------------------------------------------------------------------------------------------
    //-------------------------------------------------------------------------
    while(ros::ok() && secondsElapsed <= 480)
    {
        ros::spinOnce();

        //Check if any bumper is pressed, and update state if so
        if(BumperStatus() && currentState != BumperHit)
        {
            ROS_INFO("Bumper hit");
            UpdateState(BumperHit, currentState, prevState, initialTurnYaw, yawValues, distanceValues, initialX, initialY);
        }

        //------------------------------------------------
        //------------ 4 Main States Start --------------------------------------------------------------------------------------
        //------------------------------------------------
        //STATE 1 - Scan the area for a good place to go
        if(currentState == TurnAndScan)
        {
            //Turn 360 degrees given an initial starting yaw
            if(TurnAngle(initialTurnYaw-0.5, 360.))
            {
                ROS_INFO("Turned, finding direction to go");
                
                //Checks if the previous state was BumperHit to toggle between full range and limited range
                if((prevState == BumperHit) || (prevState == CornerFound))
                {
                    //targetYaw = DirectionFinder(yawValues, distanceValues);   
                    targetYaw = NextDirection(initialTurnYaw, yawValues, distanceValues, 30.0); // ignore angle of 30 on each side if bumper hit
                }else
                {
                    targetYaw = NextDirection(initialTurnYaw, yawValues, distanceValues, 60.0); //ignore angle of 60 on each side when no bumper hit
                }
                
                //targetYaw = DirectionFinder_beta(yawValues, distanceValues); // keep commented for ignoreAngle code

                //If no valid data was found, restart turn and scan
                if(targetYaw > 180.0)
                {
                    ROS_INFO("Turn and scan repeat %f", targetYaw);
                    UpdateState(TurnAndScan, currentState, prevState, initialTurnYaw, yawValues, distanceValues, initialX, initialY);
                }
                else
                {
                    ROS_INFO("Turning to %f", targetYaw);
                    //Angle to turn is offset from current angle
                    targetYaw = AngleOverflow(targetYaw - yaw_deg);
                    lastTurnTime = secondsElapsed;
                    UpdateState(TurnToTheta, currentState, prevState, initialTurnYaw, yawValues, distanceValues, initialX, initialY);
                    ROS_INFO("real targetAngle is %f", targetYaw);
                }
            }
            StoreLaser(yawValues, distanceValues);
        }
        //STATE 2 - Adjust to get to the optimal yaw angle
        else if(currentState == TurnToTheta)
        {
            if(TurnAngle(initialTurnYaw, targetYaw))
            {
                ROS_INFO("Autopilot engaged");
                UpdateState(AutoPilot, currentState, prevState, initialTurnYaw, yawValues, distanceValues, initialX, initialY);
            }
        }
        //STATE 3 - Robot drives straight using PID for guidance
        else if(currentState == AutoPilot)
        {
            //Call the PID function to drive straight while reacting to obstacles
            pidAutoPilot();
            
            //Every x seconds
            //CHANGE THIS TO BE BASED ON LASER OR SOMETHING?
            //check if corner approached, if true reverse back and turn and scan

            if(CornerCheck(leftDist, rightDist, minStraightLaserDist, 0.9*survRad))
            {
                ROS_INFO("Avoiding Corner!");
                UpdateState(CornerFound, currentState, prevState, initialTurnYaw, yawValues, distanceValues, initialX, initialY);
            }


            if(secondsElapsed - lastTurnTime > scanTime)
            {
                pidON = false;
                pidAutoPilot(); //Turn the PID off so the stored values can reset for next usage
                UpdateState(TurnAndScan, currentState, prevState, initialTurnYaw, yawValues, distanceValues, initialX, initialY);
                ROS_INFO("timed scan, scanTime is %f", scanTime);
            }
        }
        //STATE 4 - If the bumper has been hit, reverse 0.3m and then turn 360
        else if(currentState == BumperHit)
        {
            if(MoveDistance(initialX, initialY, -0.3))
            {
                ROS_INFO("Turn and scan");
                UpdateState(TurnAndScan, currentState, prevState, initialTurnYaw, yawValues, distanceValues, initialX, initialY);
            }
        }


        //If a corner has been detected, do a turn and scan
        else if(currentState == CornerFound)
        {
            if(MoveDistance(initialX, initialY, -0.1))
            {
                ROS_INFO("Turn and scan");
                UpdateState(TurnAndScan, currentState, prevState, initialTurnYaw, yawValues, distanceValues, initialX, initialY);
            }
        }
        //------------------------------------------------
        //------------ 4 Main States End ----------------------------------------------------------------------------------------
        //------------------------------------------------
        
        //Update position history
        if(PointsDistance(posHistX[histSize], posX, posHistY[histSize], posY) > 2.0) //2m
        {
            UpdateHistory();
        }

        //Update vel and ang to robot and publish
        vel.angular.z = angular;
        vel.linear.x = linear;
        vel_pub.publish(vel);

        //update scanTime

        if (scanTime < finalScanTime)
        {
            scanTime = initialScanTime + (((float)secondsElapsed / scanTimeSeconds) * (finalScanTime - initialScanTime));
        }

        // The last thing to do is to update the timer.
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
        loop_rate.sleep();
    }

    return 0;
}
