#ifndef DETECTLANE_H
#define DETECTLANE_H

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <vector>
#include <math.h>
#include <algorithm>
#include <fstream>

using namespace std;
using namespace cv;
enum Type{
    LEFT, RIGHT, BOTH
};
class DetectLane
{
public:
    DetectLane();
    ~DetectLane();

    void update(const Mat &src, Type laneType);
    
    vector<Point> getLeftLane();
    vector<Point> getRightLane();
 
    static int slideThickness; 

    static int BIRDVIEW_WIDTH;
    static int BIRDVIEW_HEIGHT;

    static Point null; // 

private:
    Mat preProcess(const Mat &src);
    Mat morphological(const Mat &imgHSV);
    Mat birdViewTranform(const Mat &source);
    void fillLane(Mat &src);
    vector<Mat> splitLayer(const Mat &src);
    vector<vector<Point> > centerRoadSide(const vector<Mat> &src);
    void detectLeftRight(const vector<vector<Point> > &points);
    Mat laneInShadow(const Mat &src);
    Point getPeekValueIndex(Mat &roi);
    void findLaneCenterVector(Mat &src);
    void showLaneDetect();
    vector<Point> getLine(Mat &src, Point &startPoint, Type lanetype);
    void checkRestartBase();
    int minThreshold[3] = {0, 0, 200};
    int maxThreshold[3] = {65, 30, 255};
    int minShadowTh[3] = {90, 43, 36};
    int maxShadowTh[3] = {120, 81, 171};
    int minLaneInShadow[3] = {90, 43, 97};
    int maxLaneInShadow[3] = {120, 80, 171};
    int skyLine = 85;
    int shadowParam = 40;
    int SLIDING_WINDOW_WIDTH = 20;
    int SLIDING_WINDOW_HEIGHT = 10;

    vector<Point> leftLane, rightLane;
    Point leftBase = null, rightBase = null;
    Type laneType;
    Mat lanedetect;
    double rotateAngle;
    

};

#endif