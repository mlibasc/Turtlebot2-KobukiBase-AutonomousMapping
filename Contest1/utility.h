#ifndef UTILITY
#define UTILITY

float AngleOverflow(float inputAngle)
{
    if (inputAngle > 180.0)
    {
        return inputAngle - 360.0;
    }
    else if (inputAngle < -180.0)
    {
        return inputAngle + 360.0;
    }
    return inputAngle;
}

//--------------------------------------------------------------
//------------------ Distance between two points----------------------------
//--------------------------------------------------------------
float PointsDistance(float x1, float x2, float y1, float y2)
{
    return sqrt(pow(fabs(x2-x1),2) + pow(fabs(y2-y1),2)); 
}

#endif