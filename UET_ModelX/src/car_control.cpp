#include "car_control.h"
#include "lane_detect.h"
// #include <ros/ros.h>
#include <iostream>
using namespace std;


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


    if (turn == 1)
    {
        i = setPoint;
        cout << "\nTurn right.\n" << endl;
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
            steer = 0;
            temp = 1;
            //err = preErr;
            cout << "///////////////// mat lan /////////////////" << endl;
        }
        // cout << "err = " << err << "\nright[setPoint].x = " << right[setPoint].x << "\n" << endl;
    }
    else if (turn == -1)
    {
        cout << "\nTurn left.\n" << endl;
        scaleL = 0.3;
        scaleR = 0.7;
        if (setPoint <= sizeL && left[setPoint] != DetectLane::null)
        {
            err = left[setPoint].x - carPos.x - laneWidth * scaleL;
            temp = 0;
        }
        else
        {
            steer = 0;
            temp = 1;
            //err = preErr;
            cout << "///////////////// mat lan ////////////////" << endl;
        }
    }
    else
    {
        scaleL = 0.5;
        scaleR = 0.5;
        if (setPoint <= sizeL && setPoint <= sizeR)
        {
            if (left[setPoint] == DetectLane::null && right[setPoint] == DetectLane::null)
            {
                steer = 0;
                temp = 1;
                //err = preErr;
                cout << "///////////////// mat lan /////////////////" << endl;
            }
            else
            {
                if (left[setPoint] == DetectLane::null)
                {
                    err = right[setPoint].x - carPos.x - laneWidth * scaleR;
                }
                else if (right[setPoint] == DetectLane::null)
                {
                    err = left[setPoint].x - carPos.x + laneWidth * scaleL;
                }
                else
                {
                    err = left[setPoint].x * scaleL + right[setPoint].x * scaleR - carPos.x;
                }
                temp = 0;
            }
        }
        else
        {
            if (setPoint <= sizeL)
            {
                err = left[setPoint].x - carPos.x + laneWidth * scaleL;
                temp = 0;
            }
            else if (setPoint <= sizeR)
            {
                err = right[setPoint].x - carPos.x - laneWidth * scaleR;
                temp = 0;
            }
            else
            {
                steer = 0;
                temp = 1;
                //err = preErr;
                cout << "///////////////// mat lan /////////////////" << endl;
            }
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
        steer = 0;
    }
    else
    {
        steer = kp * P + ki * I + kd * D;
    }
    

    //steer = err * 0.4;
    speed = 5;

    // io.setSpeed(speed);
    // io.setSteer(steer);

    
    cout << "Van toc hien tai cua xe: " << speed << "\t" << "Goc quay cua xe: " << steer << "\t" << endl;
    return -steer;
    // msg_angle.data = steer;
    // msg_speed.data = speed;
    // // steer_publisher.publish(msg_angle);
    // // speed_publisher.publish(msg_speed);


}