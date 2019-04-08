#include "detectTrafficSign.h"

DetectTrafficSign::DetectTrafficSign()
{
    createTrackbars();
};
int DetectTrafficSign::abcxyz = 0;

// DetectTrafficSign::~DetectTrafficSign() {}
// void DetectTrafficSign::setHasTrafficSign(bool HasTrafficSign)
// {
//     hasTrafficSign = HasTrafficSign;
// }
// bool DetectTrafficSign::getHasTrafficSign()
// {
//     return hasTrafficSign;
// }
void DetectTrafficSign::setTurnWhat(int turnwhat)
{
    turnWhat = turnwhat;
}
int DetectTrafficSign::getTurnWhat()
{
    return turnWhat;
}

void DetectTrafficSign::setTypeTurning(TypeTurning typeofturning)
{
    typeOfTurning = typeofturning;
}
// void DetectTrafficSign::setBeginRegconition(bool BeginRegconition)
// {
//     beginRegconition = BeginRegconition;
// }
// bool DetectTrafficSign::getBeginRegconition()
// {
//     return beginRegconition;
// }
// void DetectTrafficSign::setTurnCar(int TurnCar)
// {
//     turnCar = TurnCar;
// }
// int DetectTrafficSign::getTurnCar()
// {
//     return turnCar;
// }
// void DetectTrafficSign::setBeginTurn(bool BeginTurn)
// {
//     beginTurn = BeginTurn;
// }
// bool DetectTrafficSign::getBeginTurn()
// {
//     return beginTurn;
// }

Status DetectTrafficSign::getStatus()
{
    return statusCar;
}

void DetectTrafficSign::setStatus(Status status)
{
    statusCar = status;
}

void DetectTrafficSign::on_trackbar(int, void *)
{ //This function gets called whenever a
    // trackbar position is changed
}

string intToString(int number)
{

    std::stringstream ss;
    ss << number;
    return ss.str();
}

void DetectTrafficSign::createTrackbars()
{
    //create window for trackbars

    // namedWindow(trackbarWindowName, 0);
    //create memory to store trackbar name on window
    // char TrackbarName[50];
    // sprintf(TrackbarName, "H_MIN", H_MIN);
    // sprintf(TrackbarName, "H_MAX", H_MAX);
    // sprintf(TrackbarName, "S_MIN", S_MIN);
    // sprintf(TrackbarName, "S_MAX", S_MAX);
    // sprintf(TrackbarName, "V_MIN", V_MIN);
    // sprintf(TrackbarName, "V_MAX", V_MAX);
    // createTrackbar("H_MIN", trackbarWindowName, &H_MIN, H_MAX);
    // createTrackbar("H_MAX", trackbarWindowName, &H_MAX, H_MAX);
    // createTrackbar("S_MIN", trackbarWindowName, &S_MIN, S_MAX);
    // createTrackbar("S_MAX", trackbarWindowName, &S_MAX, S_MAX);
    // createTrackbar("V_MIN", trackbarWindowName, &V_MIN, V_MAX);
    // createTrackbar("V_MAX", trackbarWindowName, &V_MAX, V_MAX);
}

void DetectTrafficSign::morphOps(Mat &thresh)
{

    //create structuring element that will be used to "dilate" and "erode" image.
    //the element chosen here is a 3px by 3px rectangle

    Mat erodeElement = getStructuringElement(MORPH_RECT, Size(3, 3));
    //dilate with larger element so make sure object is nicely visible
    Mat dilateElement = getStructuringElement(MORPH_RECT, Size(8, 8));

    // erode(thresh, thresh, erodeElement);
    // erode(thresh, thresh, erodeElement);
    // erode(thresh, thresh, erodeElement);

    // dilate(thresh, thresh, dilateElement);
    // dilate(thresh, thresh, dilateElement);
    dilate(thresh, thresh, dilateElement);
    dilate(thresh, thresh, dilateElement);
    dilate(thresh, thresh, dilateElement);

    erode(thresh, thresh, erodeElement);
    erode(thresh, thresh, erodeElement);

    // dilate(thresh, thresh, dilateElement);

    // imshow("threashg", thresh);
}

string DetectTrafficSign::type2str(int type)
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

int DetectTrafficSign::RegconizeTrafficSign(Mat cropImg)
{
    // cout.open(pathOfLogFile, ios::app);
    int turnwhat;
    long valLeft = 0;
    long valRight = 0;
    // cout << cropImg.rows << " " << cropImg.cols << endl;
    // cout << "Anh bien bao" << "\t" << "So cot: " << cropImg.cols << "\t" << "So hang: " << cropImg.rows << "\t";
    for (int i = cropImg.rows / 2; i < cropImg.rows; i++)
    {
        ///cout << "asd ad asd" << endl;
        for (int j = cropImg.cols / 4; j < cropImg.cols / 2; j++)
        {
            // cout << "asdas " << endl;
            valLeft += (int)cropImg.at<uchar>(i, j);
            // cout << "val "<<(int)cropImg.at<uchar>(i, j)<<endl;
            valRight += (int)cropImg.at<uchar>(i, j + cropImg.cols / 4);
        }
    }
    // cout << "valLeft " << valLeft << endl
    //      << "valRight " << valRight << endl;
    // cout << "Gia tri pixel cua anh: " << "\t" << "val Left: " << valLeft << "\t" << "val Right: " << valRight << "\t";
    valLeft > valRight ? turnwhat = 1 : turnwhat = -1;
   
    // cout.close();
    return turnwhat;
}

void DetectTrafficSign::trackFilteredTrafficSign(Mat threshold, Mat blur, Mat &cameraFeed, Mat &cropImg, float &radius, Point &center)
{
    // cout.open(pathOfLogFile, ios::app);
    Mat temp;
    threshold.copyTo(temp);

    vector<vector<Point>> contours;
    // vector<Vec4i> hierarchy;

    findContours(temp, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    double refArea = 0;
    vector<Point> contours_poly;
    vector<Point> _temp;

    Rect boundRect;
    Point2f _center;
    float _radius;
    bool flag = true;
    int indexMax = -1;

    for (int index = 0; index < contours.size(); index++)
    {
        // Mat temp_Img;
        // vector<Point> contours_poly_temp;
        Rect boundRect_temp;
        // approxPolyDP(contours[index], contours_poly_temp, 3, true);
        boundRect_temp = boundingRect(contours[index]);
        // temp_Img = blur(boundRect_temp);
        // cout << index << endl;
        // cout << "Contours "<< index <<  contours[index] << endl;
        // Moments moment = moments(contours[index]);
        double area = boundRect_temp.width * boundRect_temp.height;
        // cout << "area " << area << endl;
        // cout << "Max object area " << MAX_OBJECT_AREA << endl;
        // cout << "refArea " <<  refArea << endl;
        if (area > MIN_SIGN_AREA && area < MAX_SIGN_AREA && area > refArea && boundRect_temp.width / boundRect_temp.height > 0.9 && boundRect_temp.width / boundRect_temp.height < 1.05)
        {

            indexMax = index;
            refArea = area;
        }
    }
    if (indexMax <= -1)
    {
        return;
    }
    // imshow("threshold", threshold);
    
    
    // cout << "Dien tich vung gan vuong co bien bao: " << "\t" << refArea << "\t";

    // Kiem tra loai bien thuoc dang nao o ban do va thay doi vung nhan dien quay cho hop ly
    // checkTrafficSignType();
    MAX_SIGN_AREA_FOR_REG = typeOfTurning.Area;
    // cout << "Dien tich lon nhat cho phep: " << MAX_SIGN_AREA_FOR_REG << "\t";
    if (refArea > MIN_SIGN_AREA_FOR_REG && refArea < MAX_SIGN_AREA_FOR_REG)
    {
        statusCar = SIGN;
        //bat dau recognize
        approxPolyDP(contours[indexMax], contours_poly, 3, true);
        boundRect = boundingRect(contours_poly);
        minEnclosingCircle(contours_poly, _center, _radius);
        radius = _radius / 3;
        center.x = _center.x / 3;
        center.y = _center.y / 3;

        /* *********************************************
         * Ghi lai tam va ban kinh khu vuc co bien bao *
         * *********************************************
         * */
        // cout << "Tam bien: " << center.x << "\t" << center.y << "\t";

        cropImg = blur(boundRect);
        // imshow("trafficsign", cropImg);
        
        // string name1;
        // name1 = "/home/linhlpv/Desktop/LogImg/crop/Crop" + to_string(abcxyz) + ".jpg";
        // string name2 = "/home/linhlpv/Desktop/LogImg/blur/Blur" + to_string(abcxyz) + ".jpg";

        // string name3 = "/home/linhlpv/Desktop/LogImg/threshold/Threshold" + to_string(abcxyz) + ".jpg";

        // imwrite(name1, cropImg);
        // imwrite(name2, blur);
        // imwrite(name3, threshold);
        // abcxyz++;
        turnWhat = RegconizeTrafficSign(cropImg);
        // cout << "Gia tri cua bien bao: " << turnWhat << "\t";
        // cout.close();
    }
    else if (refArea > MAX_SIGN_AREA_FOR_REG)
    {
        statusCar = TURING;
    }
    // else
    // {
    //     hasTrafficSign = false;
    // }
    // if (hasTrafficSign == true)
    //     putText(cameraFeed, "DM co bien bao kia", Point(0, 150), 2, 1, Scalar(0, 255, 0), 2);
    // if (refArea > MIN_SIGN_AREA_FOR_REG && hasTrafficSign == true)
    // {
    //     turnCar++;
    //     beginRegconition = true;
    // }
    // else
    //     beginRegconition = false;

    // // imshow("crop", cropImg);

    // // cout << objectFound << endl;
    // if (beginRegconition == true)
    // {
    //     approxPolyDP(contours[indexMax], contours_poly, 3, true);
    //     boundRect = boundingRect(contours_poly);
    //     minEnclosingCircle(contours_poly, _center, _radius);
    //     radius = _radius;
    //     center = (Point)_center;
    //     cropImg = blur(boundRect);

    //     turnWhat = RegconitionTrafficSign(cropImg);

    //     //cout << type2str(cropImg.type()) << endl;
    //     putText(cameraFeed, "Bien bao ne :3" + turnWhat, Point(0, 50), 2, 1, Scalar(0, 255, 0), 2);
    // }
}

// void DetectTrafficSign::decision()
// {
//     if (hasTrafficSign == false && turnCar != 0)
//     {
//         beginTurn = true;
//     }
//     else
//     {
//         beginTurn = false;
//     }
// }

void DetectTrafficSign::checkTrafficSignType()
{
    if (trafficSignType == TYPE_ONE)
        MAX_SIGN_AREA_FOR_REG = ONE;
    else if (trafficSignType == TYPE_TWO)
        MAX_SIGN_AREA_FOR_REG = TWO;
    else if (trafficSignType == TYPE_THREE)
        MAX_SIGN_AREA_FOR_REG = THREE;
    else 
        MAX_SIGN_AREA_FOR_REG = FOUR;
}

void DetectTrafficSign::detectTrafficSign(Mat &Img)
{
    bool trackObjects = true;
    bool useMorphOps = true;

    Mat HSV;
    Mat threshold;
    Mat re;
    Mat Gaussian;
    Mat blur;
    Mat crop;

    Point center;
    float radius = 0;
    resize(Img, re, Size(), 3, 3, 1);

    cvtColor(re, HSV, COLOR_BGR2HSV);

    inRange(HSV, Scalar(H_MIN, S_MIN, V_MIN), Scalar(H_MAX, S_MAX, V_MAX), threshold);

    /* Gaussian blur */

    GaussianBlur(threshold, Gaussian, Size(5, 5), 0, 0);

    /* Mean filter */

    medianBlur(threshold, blur, 3);

    // Mat negativeImg = getNegativeImage(threshold);
    // Mat morphops;
    // Mat element = getStructuringElement(2, Size(3, 3), Point(1, 1));
    // morphologyEx(negativeImg, morphops, 2, element);
    // Mat ne_morphops = getNegativeImage(morphops);

    //perform morphological operations on thresholded image to eliminate noise
    //and emphasize the filtered object(s)
    if (useMorphOps)
    {
        morphOps(threshold);
    }

    //pass in thresholded frame to our object tracking function
    //this function will return the x and y coordinates of the
    //filtered object
    if (trackObjects)
    {
        trackFilteredTrafficSign(threshold, blur, re, crop, radius, center);
    }

    // center of traffic sign

    circle(Img, center, radius, Scalar(0, 0, 255), 1, 8, 0);

    //show frames
    // imshow(windowName2, threshold);
    // imshow(windowName, Img);
    // imshow("resize", re);
    // imshow(windowName1, HSV);
    // // imshow("Gaussian", Gaussian);
    // // imshow("morphological", ne_morphops);
    // imshow("blur", blur);

    // // imwrite("/home/linhlpv/Desktop/LogImg/blur" + to_string(count) + ".png", blur);
    // // imwrite("/home/linhlpv/Desktop/LogImg/threshold" + to_string(count) + ".png", threshold);
    // if (!crop.empty())
    // {
    //     imshow("crop", crop);
    //     // imwrite("/home/linhlpv/Desktop/LogImg/crop" + to_string(count) + ".png", crop);
    // }
}

// Mat DetectTrafficSign::getNegativeImage(Mat src)
// {
//     Mat negativeImg = Mat::zeros(src.size(), src.type());
//     Mat sub_mat = Mat::ones(src.size(), src.type()) * 255;
//     subtract(sub_mat, src, negativeImg);
//     return negativeImg;
// }
