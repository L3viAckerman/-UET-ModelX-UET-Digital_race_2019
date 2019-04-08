#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui/highgui.hpp>
#include<time.h>

#include "detectlane.h"
#include "carcontrol.h"
#include "detectTrafficSign.h"
#include "detectObstacle.h"


double comp_Hist = 0;

Mat preImg;
Mat RGB_Img;
Mat Depth_Img;
static int COUNT = 0;


string type2str(int type)
{
    string r;

    uchar depth = type & CV_MAT_DEPTH_MASK;
    uchar chans = 1 + (type >> CV_CN_SHIFT);

    switch (depth)
    {
    case CV_8U:
        r = "8U";
        break;
    case CV_8S:
        r = "8S";
        break;
    case CV_16U:
        r = "16U";
        break;
    case CV_16S:
        r = "16S";
        break;
    case CV_32S:
        r = "32S";
        break;
    case CV_32F:
        r = "32F";
        break;
    case CV_64F:
        r = "64F";
        break;
    default:
        r = "User";
        break;
    }

    r += "C";
    r += (chans + '0');

    return r;
}


double Compare_Hist(Mat &Img, Mat &preImg)
{
    Mat ImgHSV, preImgHSV;
    cvtColor(Img, ImgHSV, CV_BGR2HSV);
    cvtColor(preImg, preImgHSV, CV_BGR2HSV);

    // Using 50 bins for hue and 60 for saturation
    int h_bins = 50; int s_bins = 60;
    int histSize[] = { h_bins, s_bins };

    // hue varies from 0 to 179, saturation from 0 to 255
    float h_ranges[] = { 0, 180 };
    float s_ranges[] = { 0, 256 };

    const float* ranges[] = { h_ranges, s_ranges };

    // Use the o-th and 1-st channels
    int channels[] = { 0, 1 };

    /// Histograms
    MatND hist_Img, hist_preImg;

    // Calc Histogram
    calcHist( &ImgHSV, 1, channels, Mat(), hist_Img, 2, histSize, ranges, true, false );
    normalize( hist_Img, hist_Img, 0, 1, NORM_MINMAX, -1, Mat() );

    calcHist( &preImgHSV, 1, channels, Mat(), hist_preImg, 2, histSize, ranges, true, false );
    normalize( hist_preImg, hist_preImg, 0, 1, NORM_MINMAX, -1, Mat() );

    int compare_method = 1;
    double baseHist = compareHist( hist_Img, hist_preImg, compare_method );
    return baseHist;
}

bool checkRestart()
{
    /* tham so chinh tuy map, voi map 2 bien kia la 15 */
    if ( comp_Hist > 30) return true;
    else return false;
}

const double TIMEFORTURN = 0.10;

bool STREAM = true;

VideoCapture capture("video.avi");
DetectLane *detect;
CarControl *car;
DetectTrafficSign *trafficSign;
DetectObstacle *obstacle;

/**********************************************
 *  BO tham so de quay xe o cac map khac nhau *
 * ********************************************
 *  */
// TypeTurning typeOfTurning[2] = {{1800000, 120 * 125, 30, 6}, {31600000,115 * 125, 35, 7}};
TypeTurning typeOfTurning[2] = {{1500000, 91 * 91, 30, 6}, {2500000, 95 * 95, 30, 7}};

/***************************
 *  Bien trang thai cua xe *
 * *************************
 *  */
int TYPE_TURN = 0;

/*******************************
 *  Bien thoi gian quay cua xe *
 * *****************************
 *  */
double timeTurn = 0;

// bool beginTurn = false;

int skipFrame = 1;

/****************************************************************************
 *  Bien dem so luong frame, tu dong reset khi xe quay lai vi tri xuat phat *
 * **************************************************************************
 *  */
int countFrame = 0;

void imageCallback_depth(const sensor_msgs::ImageConstPtr& msg)
{

    cv_bridge::CvImagePtr cv_ptr;
    // Mat out;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC3);
        cv_ptr->image.copyTo(Depth_Img);
        

	    waitKey(1);
        // detect->update(cv_ptr->image);
        // car->driverCar(detect->getLeftLane(), detect->getRightLane(), 50);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to '16UC3'.", e.what());
    }
}



void imageCallback_RGB(const sensor_msgs::ImageConstPtr& msg)
{
    /**********************
     * Mo file Log de ghi *
     * ********************
     *  */
    
    

    cv_bridge::CvImagePtr cv_ptr;
    Mat out;
    Mat Img;
    if(!Depth_Img.empty()) 
    {
        // Depth_Img.convertTo(Depth_Img, CV_8U);
        // print(Depth_Img);
        // Depth_Img.convertTo(Depth_Img, CV_8UC3);
        Mat gray;
        cvtColor(Depth_Img, gray, COLOR_BGR2GRAY);
        
        cv::imshow("View_depth", gray);
        normalize(gray, gray, 0, 255, NORM_MINMAX);
        GaussianBlur(gray, gray, Size(3, 3), 0, 0);
        /*
            Histogram
        */
        // vector<Mat> bgr_planes;
        // split( gray, bgr_planes );
        // int histSize = 256;
        // float range[] = { 0, 256 }; //the upper boundary is exclusive
        // const float* histRange = { range };
        // bool uniform = true, accumulate = false;
        // Mat b_hist;
        // calcHist( &bgr_planes[0], 1, 0, Mat(), b_hist, 1, &histSize, &histRange, uniform, accumulate );
        
        // int hist_w = 512, hist_h = 400;
        // int bin_w = cvRound( (double) hist_w/histSize );
        // Mat histImage( hist_h, hist_w, CV_8UC1, Scalar( 0) );
        // normalize(b_hist, b_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );
        
        // for( int i = 1; i < histSize; i++ )
        // {
        //     line( histImage, Point( bin_w*(i-1), hist_h - cvRound(b_hist.at<float>(i-1)) ),
        //         Point( bin_w*(i), hist_h - cvRound(b_hist.at<float>(i)) ),
        //         Scalar( 255, 0, 0), 2, 8, 0  );
            
        // }
       
        // imshow("calcHist Demo", histImage );



        string ty =  type2str( gray.type() );
        gray.convertTo(gray, CV_8UC1);
    
        
        Mat canny(gray.size(), CV_32FC1);
        
        Canny(gray, canny, 90, 120, 3);
        inRange(gray, 30 , 254, gray);
        cv::imshow("Pre", gray);
        // imwrite("Pre" + to_string(COUNT+1) + ".png", gray);
        Canny(gray, gray, 10, 30, 3);
        
        
        // imshow("sobel", sobel);
        
        imshow("canny", canny);
        // imshow("canny_pre", gray);

        // imwrite("canny" + to_string(COUNT+1) + ".png", canny);
        // imwrite("canny_pre" + to_string(COUNT+1) + ".png", gray);
        // COUNT++;
        


        // print(gray);
    }
     
    static std::clock_t start;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
       
        cv_ptr->image.copyTo(Img);

        Img.copyTo(RGB_Img);

	    waitKey(1);

        // Compare Hist of Img and preImgView_BGR
        if(!preImg.empty())
        {
            comp_Hist = Compare_Hist(Img, preImg);
            // cout << "compare Hist " << comp_Hist << endl;
        }

       

        // Check restart
        if (checkRestart()) 
        {
            TYPE_TURN = 0;
            countFrame = 0;
        }

        // cout << "Day la frame thu " << countFrame << "\t";
        // cout << "Gia tri cua compare Histograme: " << comp_Hist << "\t";

        // cout << "type turn " << TYPE_TURN << endl;
        // cout << "Dang la loai bien thu may: " << TYPE_TURN << "\t";

        detect->update(cv_ptr->image);
        car->setTypeTurning(typeOfTurning[TYPE_TURN]);
        trafficSign->setTypeTurning(typeOfTurning[TYPE_TURN]);
        trafficSign->detectTrafficSign(Img);
        // car->setHasTrafficSign(trafficSign->getHasTrafficSign()) ;
        // car->setBeginRegconition(trafficSign->getBeginRegconition());
        // car->setBeginTurn(trafficSign->getBeginTurn());
        // car->setTurnCar(trafficSign->getTurnCar());
        // car->setTurnWhat(trafficSign->getTurnWhat());
        car->setStatus(trafficSign->getStatus());
        car->setTurnWhat(trafficSign->getTurnWhat());
        // cout << "Turn what " << car->getTurnWhat() << endl;
        // cout << car->getStatus() << endl;

        // cout << "Trang thai ngay truoc cua xe la: " << car->getPreStatus() << "\t" << "Trang thai cua xe hien tai: " << car->getStatus() << "\t";
        if(car->getStatus() == TURING)
        {
            if(car->getPreStatus() == SIGN)
            {
                start = std::clock();
            } 
            else 
            {
                // cout << "Thoi gian quay cua xe: " << std::clock() - start << endl << "\t";
                // cout << "Thoi gian quay cua xe: " << std::clock() - start << endl << "\t";
                if(std::clock() - start > typeOfTurning[TYPE_TURN].time)
                {
                    car->setStatus(NOSIGN);
                    trafficSign->setStatus(NOSIGN);
                    TYPE_TURN++;
                    if(TYPE_TURN > 1) 
                        TYPE_TURN = 0;
                }
            }
        }
        // if( TYPE_TURN == 1 && std::clock() - start > typeOfTurning[TYPE_TURN].time )
        // {
        //     TYPE_TURN = 0;
        // }
        car->driverCar(detect->getLeftLane(), detect->getRightLane(), 15);
        preImg = Img;
        // Mat objDetect = Img.clone();
        // obstacle->update(objDetect);
        // imshow("obstacle", objDetect);

        countFrame ++;
        // cout << endl;
        // circle(Img, CENTER, 1, (0, 255, 255), LINE_8, 0);
        
        // cv::imshow("View", Img);
        // if ( car->getStatus() ==  )
        // {
            
        //     if (timeTurn < TIMEFORTURN) {
        //         std::clock_t start;
        //         start = std::clock();
        //         car->setStopTurn(false);
        //         car->driverCar(detect->getLeftLane(), detect->getRightLane(), 60);
        //         timeTurn += ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
        //     }
        //     else 
        //     {View_BGR
        //         car->setBeginTurn(false);
        //         car->setStopTurn(true); 
        //         car->setTurnCar(0);
        //         trafficSign->setTurnCar(0);
        //         car->driverCar(detect->getLeftLane(), detect->getRightLane(), 60);
        //         timeTurn = 0;
        //     }
        // }
        // else 
        //     car->driverCar(detect->getLeftLane(), detect->getRightLane(), 60);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

void videoProcess()
{
    Mat src;
    while (true)
    {
        capture >> src;
        if (src.empty()) break;
        
        imshow("View", src);
        detect->update(src);
        waitKey(30);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_listener");
    cv::namedWindow("View");
    cv::namedWindow("View_depth");
    cv::namedWindow("Binary");
    cv::namedWindow("Threshold");
    cv::namedWindow("Bird View");
    cv::namedWindow("Lane Detect");
    
    // double duration;

    


    detect = new DetectLane();
    car = new CarControl();
    trafficSign = new DetectTrafficSign();
    obstacle = new DetectObstacle();    

    if (STREAM) {
        cv::startWindowThread();

        ros::NodeHandle nh;
        
        image_transport::ImageTransport it(nh);

        image_transport::Subscriber sub_depth = it.subscribe("camera/depth/image_raw", 1, imageCallback_depth);
        image_transport::Subscriber sub_rgb = it.subscribe("camera/rgb/image_raw", 1, imageCallback_RGB);
        
        
        ros::spin();
    } else {
        videoProcess();
    }
    cv::destroyAllWindows();
    
    
    return 0;
}
