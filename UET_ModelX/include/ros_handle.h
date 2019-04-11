#ifndef ROS_HANDLE_H_
#define ROS_HANDLE_H_
#include <ros/ros.h>
#include <string>
#include <iostream>

#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>

// #include <msgs/Int.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include "OpenNIHelper.h"


using namespace std;
using namespace cv;

class ros_handle
{
private:
    //callback

    // Publishers
    ros::Publisher pub_steer;
    ros::Publisher pub_speed;
    ros::Publisher pub_camSteer;

    // Subscribers
    ros::Subscriber sub_button;
    ros::Subscriber sub_sensor;
    ros::Subscriber sub_mpu;

public:
    ros_handle(/* args */);
    // init
    void init_ros();
    
    // callbacks
    void buttonCallback(const std_msgs::Int32::ConstPtr& msg);
    void sensorCallback(const std_msgs::Bool::ConstPtr& msg);
    void mpuCallback(const std_msgs::Float32::ConstPtr& msg);

    //data msg return
    int button_pressed;
    bool has_obj;
    float mpu_angle;

    // data to publish
    float speed;
    float steer;
    float camSteer;

    // pub to car
    void pub_to_car();
};

ros_handle::ros_handle()
{
    init_ros();
}
#endif