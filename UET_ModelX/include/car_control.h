#ifndef CARCONTROL_H
#define CARCONTROL_H
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <vector>
#include <math.h>

#include "lane_detect.h"

using namespace std;
using namespace cv;

class CarControl 
{
public:
    CarControl()
    {
        carPos.x = 160;
        carPos.y = 240;
        // steer_publisher = node_obj1.advertise<std_msgs::Float32>("set_steer_car_api", 10);
        // speed_publisher = node_obj2.advertise<std_msgs::Float32>("set_speed_car_api", 10);

        laneWidth = 100;
        turn = 0;
        speed = 0;
        steer = 0;
        err = 0;
        preError = 0;
        kp = 0;
        ki = 0;
        kd = 0;
        P = 0;
        I = 0;
        D = 0;
        pid_file.open("/home/nvidia/pid.txt");
        pid_file >> kp >> ki >> kd;
    }
    void setPID (float KP, float KI, float KD);
    void set_Point (const int poi);
    void setTurn (int TURN);
    float driverCar(const vector<Point> &left, const vector<Point> &right);
    //void setIO(IOInterface io_interface);
private:
    //IOInterface io;

    // ros::NodeHandle node_obj1;
    // ros::NodeHandle node_obj2;
    ifstream pid_file;
    // ros::Publisher steer_publisher;
    // ros::Publisher speed_publisher;

    float laneWidth;
    Point carPos;
    int turn;

    float speed;
    float steer;

    float err;
    float preError;

    float kp, ki, kd;
    float P, I, D;

    // std_msgs::Float32 msg_angle;
    // std_msgs::Float32 msg_speed;

    int setPoint;

    int i, j;
    int sizeL = 0;
    int sizeR = 0;
    float scaleL = 0.5, scaleR = 0.5;

};


#endif