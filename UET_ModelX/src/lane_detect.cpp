#include "lane_detect.h"

int min(int a, int b)
{
    return a < b ? a : b;
}

int left_x = 116;
int left_y = 90;
int right_x = 203;
int right_y = 90;
int down_l_x = 29;
int down_l_y = 160;
int down_r_x = 292;
int down_r_y = 160;
int DetectLane::slideThickness = 10;
int DetectLane::BIRDVIEW_WIDTH = 320;
int DetectLane::BIRDVIEW_HEIGHT = 240;
Point DetectLane::null = Point(-1, -1);
using namespace std;
DetectLane::DetectLane()
{
    namedWindow("Threshold");
    cvCreateTrackbar("LowH", "Threshold", &minThreshold[0], 179);
    cvCreateTrackbar("HighH", "Threshold", &maxThreshold[0], 179);

    cvCreateTrackbar("LowS", "Threshold", &minThreshold[1], 255);
    cvCreateTrackbar("HighS", "Threshold", &maxThreshold[1], 255);

    cvCreateTrackbar("LowV", "Threshold", &minThreshold[2], 255);
    cvCreateTrackbar("HighV", "Threshold", &maxThreshold[2], 255);

    cvCreateTrackbar("Shadow Param", "Threshold", &shadowParam, 255);
}

DetectLane::~DetectLane() {}

vector<Point> DetectLane::getLeftLane()
{
    return leftLane;
}

vector<Point> DetectLane::getRightLane()
{
    return rightLane;
}

void DetectLane::update(const Mat &src, Type laneType)
{
    static int i = 0;
    checkRestartBase();
    leftLane.clear();
    rightLane.clear();
    lanedetect = Mat::zeros(240, 320, CV_8UC3);
    this->laneType = laneType;
    switch (laneType)
    {
        case RIGHT:
            rotateAngle = -18;
            break;
        case LEFT:
            rotateAngle = 19;
            break;
        default:
            rotateAngle = 0;
            break;
    }
    Mat birdview = preProcess(src);
    //birdview.convertTo(lanedetect, CV_8UC3);
    // imwrite("src" + to_string(i)+".png", src);
    // imwrite("birdview" + to_string(i) + ".png", birdview);
    findLaneCenterVector(birdview);
    showLaneDetect();

    // imwrite("lane detect" + to_string(i) + ".png", lanedetect);
    i++;
}

Mat DetectLane::preProcess(const Mat &src)
{
    Mat imgThresholded, imgHSV, dst;

    cvtColor(src, imgHSV, COLOR_BGR2HSV);
    inRange(imgHSV, Scalar(minThreshold[0], minThreshold[1], minThreshold[2]),
            Scalar(maxThreshold[0], maxThreshold[1], maxThreshold[2]),
            imgThresholded);
    // set ROI
    // for(int i = 0; i < (int) src.cols; i++)
    //     {
    //         for(int j = 0; j < src.rows/2.2; j++)
    //         {
    //             imgThresholded.at<uchar>(j, i) = 0;
    //         }
            
    //     }

    // switch (laneType)
    // {
    //     case RIGHT:
    //         for(int i = 0; i < (int) 2*src.cols / 5; i++)
    //         {
    //             for(int j = 0; j < src.rows; j++)
    //             {
    //                 imgThresholded.at<uchar>(j, i) = 0;
    //             }
    //         }  
    //         break;
    //     case LEFT:
    //         for(int i = src.cols / 2; i < src.cols; i++)
    //         {
    //             Size(320, 320)for(int j = 0; j < src.rows; j++)
    //             {
    //                 imgThresholded.at<uchar>(j, i) = 0;
    //             }
                
    //         }
    //         break;
    //     default:
    //         break;
    // }
    // imshow("Binary", imgThresholded);
// Size(320, 320)
    dst = birdViewTranform(imgThresholded);
    // imshow("Bird View", dst);
    
    fillLane(dst);
    //imshow("fill lane", dst);
    return dst;
}

Mat DetectLane::birdViewTranform(const Mat &src)
{
    Point2f src_vertices[4];

    int width = src.size().width;
    int height = src.size().height;

    src_vertices[0] = Point(down_r_x , down_r_y);
    src_vertices[1] = Point(down_l_x , down_l_y);
    src_vertices[2] = Point(left_x, left_y);
    src_vertices[3] = Point(right_x, right_y);

    Point2f dst_vertices[4];
    dst_vertices[0] = Point( width - 120, height + 40);
    dst_vertices[1] = Point(0 + 120, height + 40);
    dst_vertices[2] = Point(0 + 120, 0);
    dst_vertices[3] = Point(width - 120, 0);

    Mat M = getPerspectiveTransform(src_vertices, dst_vertices);
    
    Mat dst(BIRDVIEW_HEIGHT + 80, BIRDVIEW_WIDTH, CV_8UC3);
    warpPerspective(src, dst, M, dst.size(), INTER_LINEAR, BORDER_CONSTANT);
    // imshow("perspective", dst);
    Mat rotationMatrix = getRotationMatrix2D(Point2f(dst.cols / 2.0, dst.rows),rotateAngle, 1);
    warpAffine(dst, dst, rotationMatrix, dst.size());
    // imshow("rotated", dst);
    return dst;
}

void DetectLane::fillLane(Mat &src)
{
    vector<Vec4i> lines;
    HoughLinesP(src, lines, 1, CV_PI / 180, 1);
    for (size_t i = 0; i < lines.size(); i++)
    {
        Vec4i l = lines[i];
        line(src, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255), 1, CV_AA);
    }
    medianBlur(src, src, 3);
}

Mat DetectLane::morphological(const Mat &img)
{
    Mat dst;

    // erode(img, dst, getStructuringElement(MORPH_ELLIPSE, Size(1, 1)) );
    // dilate( dst, dst, getStructuringElement(MORPH_ELLIPSE, Size(1, 1)) );

    dilate(img, dst, getStructuringElement(MORPH_ELLIPSE, Size(10, 20)));
    erode(dst, dst, getStructuringElement(MORPH_ELLIPSE, Size(10, 20)));

    // blur(dst, dst, Size(3, 3));

    return dst;
}

Point DetectLane::getPeekValueIndex(Mat &roi){
    Mat sumByColumn;

    reduce(roi, sumByColumn, 0, CV_REDUCE_SUM, CV_32SC1);
    normalize(sumByColumn, sumByColumn, 0, 51, NORM_MINMAX);
    sumByColumn.convertTo(sumByColumn, CV_8UC1);
    Point anchor = Point(-1, -1);
    Mat kernel = Mat::ones(1, 5, CV_8UC1);
    filter2D(sumByColumn, sumByColumn, -1, kernel, anchor, 0, BORDER_CONSTANT);
    Point maxIdx;
    double maxValue;
    minMaxLoc(sumByColumn, NULL, &maxValue, NULL, &maxIdx);
    cout << maxValue << "\n";
    if(maxValue <= 0){
        return null;
    }
    return maxIdx;
}

void DetectLane::findLaneCenterVector(Mat &src){
    Mat ROI;
    Rect ROIRect;
    Point leftStartingPoint, rightStartingPoint;

    if(laneType == LEFT || laneType == BOTH){
        if(leftBase != null && leftBase.x > 10 && leftBase.x < src.cols - 10 &&  leftBase.y > SLIDING_WINDOW_HEIGHT && leftBase.y <= src.rows){
            ROIRect = Rect(leftBase.x - 10, leftBase.y - SLIDING_WINDOW_HEIGHT, 20, SLIDING_WINDOW_HEIGHT);
        } else {
            cout << "changeROI" << "\n";
            ROIRect = Rect(0, BIRDVIEW_HEIGHT * 4 / 5, BIRDVIEW_WIDTH / 2, BIRDVIEW_HEIGHT / 5);
        }
        ROI = Mat(src, ROIRect);
        leftStartingPoint = getPeekValueIndex(ROI);
        cout << leftStartingPoint;
        if(leftStartingPoint == null && leftBase != null){
            cout << "null\n";
            leftStartingPoint = leftBase;
        }
        else if(leftStartingPoint != null){
            leftStartingPoint.x += ROIRect.x;
            leftStartingPoint.y += BIRDVIEW_HEIGHT;
        }
        leftLane = getLine(src, leftStartingPoint, LEFT);
        leftBase = leftStartingPoint;
        cout << leftBase<<"\n";
        
    }

    if(laneType == RIGHT || laneType == BOTH){
        if(rightBase != null && rightBase.x > 10 && rightBase.x < src.cols - 10 &&  rightBase.y > SLIDING_WINDOW_HEIGHT && rightBase.y <= src.rows){
            ROIRect = Rect(rightBase.x - 10, rightBase.y - SLIDING_WINDOW_HEIGHT, 20, SLIDING_WINDOW_HEIGHT);
        } else {
            cout << "changeROI" << "\n";
            ROIRect = Rect(BIRDVIEW_WIDTH / 2, BIRDVIEW_HEIGHT * 4 / 5, BIRDVIEW_WIDTH / 2, BIRDVIEW_HEIGHT / 5);
        }
        ROI = Mat(src, ROIRect);
        rightStartingPoint = getPeekValueIndex(ROI);
        cout << rightStartingPoint;
        if(rightStartingPoint == null && rightBase != null){
            cout << "null\n";
            rightStartingPoint = rightBase;
        }
        else if(rightStartingPoint != null){
            rightStartingPoint.x += ROIRect.x;
            rightStartingPoint.y += BIRDVIEW_HEIGHT;
        }
        rightLane = getLine(src, rightStartingPoint, RIGHT);
        rightBase = rightStartingPoint;
        cout << rightBase<<"\n";
        
    }

}
int getWhitePoint_X(Mat src, Type lanetype){
    Mat sumByColumn;
    reduce(src, sumByColumn, 0, CV_REDUCE_SUM, CV_32SC1);
    if(lanetype == RIGHT){
        for(int i = sumByColumn.cols - 1; i >= 0; i--){
            if(sumByColumn.at<int>(0, i) > 0){
                return i;
            }
        }
    }
    if(lanetype == LEFT){
        for(int i = 0; i < sumByColumn.cols; i++){
            if(sumByColumn.at<int>(0, i) > 0){
                return i;
            }
        }
    }
    return -1;
}
vector<Point> DetectLane::getLine(Mat &src, Point &startPoint, Type lanetype){
    vector<Point> result;
    if(startPoint == null) return result;
    result.push_back(startPoint);
    int predict_x = startPoint.x, sum_x_error  = 0, pre_x = startPoint.x;
    int x_slidingWindow, y_slidingWindow;
    Point center1 = startPoint;
    y_slidingWindow = startPoint.y - SLIDING_WINDOW_HEIGHT;
    x_slidingWindow = startPoint.x - SLIDING_WINDOW_WIDTH / 2;
    while(x_slidingWindow >=0 && y_slidingWindow >= 0 && 
          x_slidingWindow + SLIDING_WINDOW_WIDTH <= src.cols &&
          y_slidingWindow + SLIDING_WINDOW_HEIGHT <= src.rows ){
        Rect slidingWindow = Rect(x_slidingWindow, y_slidingWindow,
                                 SLIDING_WINDOW_WIDTH, SLIDING_WINDOW_HEIGHT);
        rectangle(lanedetect, slidingWindow, Scalar(0, 0, 255));
        // Moments M1 = moments(Mat(src, slidingWindow), true);
        // if(M1.m00 == 0){
        //     result.push_back(null);
        //     //center1.x = center1.x * 2 -pre_x;
        // }
        // else {
        //     center1 = Point(static_cast<float>(M1.m10 / M1.m00), static_cast<float>(M1.m01 / M1.m00));
        //     center1.x += x_slidingWindow;
        //     //center1.x = center1.x * 0.7 + predict_x * 0.3;
        //     center1.y += y_slidingWindow;
        //     result.push_back(center1);
        //     x_slidingWindow = center1.x * 2 -pre_x - SLIDING_WINDOW_WIDTH/2;
        //     pre_x = center1.x;
            
        // } 
        int m_x = getWhitePoint_X(Mat(src, slidingWindow), lanetype);
        if(m_x == -1){
            result.push_back(null);
            center1.x = x_slidingWindow + SLIDING_WINDOW_WIDTH/2;
        }
        else {
            center1 = Point(m_x, slidingWindow.height / 2);
            center1.x += x_slidingWindow;
            //center1.x = center1.x * 0.7 + predict_x * 0.3;
            center1.y += y_slidingWindow;
            result.push_back(center1);
            
        } 
        x_slidingWindow = center1.x * 2 -pre_x - SLIDING_WINDOW_WIDTH/2;
        pre_x = center1.x;
        if(result.back() != null)
        rectangle(lanedetect, slidingWindow, Scalar(255, 0, 0), 2);
        y_slidingWindow -= SLIDING_WINDOW_HEIGHT;
        //pre_x = result.back().x;
        sum_x_error += result.back().x - pre_x;
        //predict_x = result.back().x +(int)((result.back().x - pre_x) * 0.9 + (sum_x_error) * 0.1 / (result.size() - 1));
    }
    return result;
}

void DetectLane::showLaneDetect(){
    for(int i = 0; i < leftLane.size(); i++){
        circle(lanedetect, leftLane[i], 2, Scalar(255, 0, 0), -1);
    }
    for(int i = 0; i < rightLane.size(); i++){
        circle(lanedetect, rightLane[i], 2, Scalar(0, 255, 0), -1);
    }
    circle(lanedetect, Point(160, 240), 3, Scalar(0,0,255), -1);
    // circle(lanedetect, rightLane[1], 3, Scalar(0,255,255), -1);
    // imshow("lane_detect +", lanedetect);
}
void DetectLane::checkRestartBase(){
    int countnull = 0;
    if(rightLane.size() == 0 || 
        (rightLane.size() > 4 && rightLane[3] == null && rightLane[1] == null && rightLane[2] == null)){
        rightBase = null;
    }
    if(leftLane.size() == 0 || 
        (leftLane.size() > 4 && leftLane[3] == null && leftLane[1] == null && leftLane[2] == null)){
        leftBase = null;
    }
}