#include <ros/ros.h>
#include <string>

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <std_msgs/Float32.h>
#include "lane_detect.h"
#include "ros_handle.h"

#include "car_control.h"
#include "detecttrafficsign.h"

cv::VideoWriter video("out.avi",CV_FOURCC('M','J','P','G'),30, cv::Size(320, 240),true);
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "UET_ModelX");
  ni::openni2_init();
  Mat depth, color;
  DetectLane dtLane;
  DetectTrafficSign TFSign;
  vector<cv::Point> leftLane, rightLane;

  ros_handle ros_obj;

  CarControl car;
  ros::Rate rate(30.);
  while(true)
  {
    ni::openni2_getmat(color, depth);

    imshow("color", color);
    imshow("depth", depth);

    // video writer
    video.write(color);

    dtLane.update(color, RIGHT);
    leftLane = dtLane.getLeftLane();
    rightLane = dtLane.getRightLane();

    TFSign.detectTrafficSign(color);
    cout << TFSign.getTrafficSignInfor() << endl;


    car.set_Point (7);
    //car.setPID (0.9, 0, 0);
    car.setTurn (1);
    
    ros_obj.steer = car.driverCar(leftLane, rightLane);
    
    // ros_obj.speed = 0;
    // ros_obj.camSteer = -30;
    
    ros_obj.pub_to_car();
 
    waitKey(1);
    rate.sleep();
  }
  ni::openni2_destroy();
  cv::destroyAllWindows();
  return 0;
}