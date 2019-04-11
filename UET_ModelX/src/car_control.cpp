#include "car_control.h"
#include "lane_detect.h"
// #include <ros/ros.h>
#include <iostream>
using namespace std;
void CarControl::setSpeed (float maxSpeed, float minSpeed)
{
}

float CarControl::powerCar ()
{
    if (err >= 65)
    {
        return 1;
    }
    return speed;
}
void CarControl::getAngle (float ang)
{
    angle = ang;
}
void CarControl::setSetAngle (float ang)
{
    set_angle = ang;
}

void CarControl::setPID (float KP, float KI, float KD)
{
    kp = KP;
    ki = KI;
    kd = KD;
}
void CarControl::set_Point (const int poi)
{
    setPoint = poi;
}
void CarControl::setTurn (int TURN)
{
    turn = TURN;
}

float CarControl::driverCar(const vector<Point> &left, const vector<Point> &right)
{
    int temp = 0;

    i = 0;
    j = 0;


    sizeL = left.size();
    sizeR = right.size();


    if (turn == -1)
    {
        i = setPoint;
        cout << "\nTurn right.\n" << endl;
        scaleL = 0.7;
        scaleR = 0.3;
        while (i >= sizeR || right[i] == DetectLane::null)
        {
            i--;
            if (i < 0)
            {
                break;
            }
        }
        if (i > 0)
        {
            err = right[i].x - carPos.x - laneWidth * scaleR;
        }
        // if (setPoint <= sizeR && right[setPoint] != DetectLane::null)
        // {
        //     err = right[setPoint].x - carPos.x - laneWidth * scaleR;
        //     temp = 0;
        // }
        else
        {
            //steer = 0;
            temp = 1;
            //err = preErr;
            cout << "///////////////// mat lan /////////////////" << endl;
        }
        // cout << "err = " << err << "\nright[setPoint].x = " << right[setPoint].x << "\n" << endl;
    }
    else if (turn == 1)
    {
        i = setPoint;
        cout << "\nTurn left.\n" << endl;
        scaleL = 0.3;
        scaleR = 0.7;
        while (i >= sizeL || left[i] == DetectLane::null)
        {
            i--;
            if (i < 0)
            {
                break;
            }
        }
        if (i > 0)
        {
            err = left[i].x - carPos.x + laneWidth * scaleR;
        }
        // if (setPoint <= sizeR && right[setPoint] != DetectLane::null)
        // {
        //     err = right[setPoint].x - carPos.x - laneWidth * scaleR;
        //     temp = 0;
        // }
        else
        {
            //steer = 0;
            temp = 1;
            //err = preErr;
            cout << "///////////////// mat lan /////////////////" << endl;
        }
    }
    else
    {
        i = setPoint;
        cout << "\n Di giua duong.\n" << endl;
        scaleL = 0.5;
        scaleR = 0.5;
        while (i >= sizeR || right[i] == DetectLane::null)
        {
            i--;
            if (i < 0)
            {
                break;
            }
        }
        if (i > 0)
        {
            err = right[i].x - carPos.x - laneWidth * scaleR;
        }
        // if (setPoint <= sizeR && right[setPoint] != DetectLane::null)
        // {
        //     err = right[setPoint].x - carPos.x - laneWidth * scaleR;
        //     temp = 0;
        // }
        else
        {
            //steer = 0;
            temp = 1;
            //err = preErr;
            cout << "///////////////// mat lan /////////////////" << endl;
        }
    }
    
    
    // kp = 0.85;
    // ki = 0.0008;
    // kd = 0.09;

    P =  err;
    I = I + err;
    D = err - preError;

    preError = err;

    if (temp == 1)
    {
        if (angle > set_angle && angle > 180)
        {
            angle = 360 - angle;
        }
        steer = (angle - set_angle) * 1.2;
    }
    else
    {
        steer = kp * P + ki * I + kd * D;
    }
    if (steer > 65)
    {
        steer = 65;
    }
    if (steer < -65)
    {
        steer = -65;
    }

    //steer = err * 0.4;

    // io.setSpeed(speed);
    // io.setSteer(steer);

    
    cout << "Van toc hien tai cua xe: " << speed << "\t" << "Goc quay cua xe: " << steer << "\t" << endl;
    return -steer;
    // msg_angle.data = steer;
    // msg_speed.data = speed;
    // // steer_publisher.publish(msg_angle);
    // // speed_publisher.publish(msg_speed);


}