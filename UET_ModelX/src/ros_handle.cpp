#include "ros_handle.h"
#include "detecttrafficsign.h"

void ros_handle::init_ros()
{
    ros::NodeHandle nh;

    // Publishers
    pub_camSteer = nh.advertise<std_msgs::Float32>("set_pos_camera_api", 1);
    pub_speed = nh.advertise<std_msgs::Float32>("set_speed_car_api", 1);
    pub_steer = nh.advertise<std_msgs::Float32>("set_steer_car_api", 1);

    // Subscribers
    sub_button = nh.subscribe("button_on", 5, &ros_handle::buttonCallback, this);
    sub_sensor = nh.subscribe("status_sensor", 5, &ros_handle::sensorCallback, this);
    sub_mpu = nh.subscribe("imu_angle", 5, &ros_handle::mpuCallback, this);
}

void ros_handle::pub_to_car()
{
    std_msgs::Float32 steer_msg, speed_msg, camSteer_msg;
    steer_msg.data = steer;
    speed_msg.data = speed;

    if (camSteer_msg.data != camSteer)
    {
        camSteer_msg.data = camSteer;
        pub_camSteer.publish(camSteer_msg);
    }

    pub_speed.publish(speed_msg);
    pub_steer.publish(steer_msg);
}

void ros_handle::buttonCallback(const std_msgs::Int32::ConstPtr& msg)
{
    button_pressed = msg->data;

}
void ros_handle::sensorCallback(const std_msgs::Bool::ConstPtr& msg)
{
    has_obj = !msg->data;
    std::cout << "has_obj" << has_obj << std::endl;
    std::cout << "msg->data" << msg->data << std::endl;
}

void ros_handle::mpuCallback(const std_msgs::Float32::ConstPtr& msg)
{
    mpu_angle = msg->data;
}