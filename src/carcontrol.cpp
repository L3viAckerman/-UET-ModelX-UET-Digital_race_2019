#include "carcontrol.h"

int TYPE_TURN = 0;

fstream fileLog;
string pathOfLogFile = "LogFIle.txt";
Point CENTER;


CarControl::CarControl()
{
    carPos.x = 120;
    carPos.y = 300;
    steer_publisher = node_obj1.advertise<std_msgs::Float32>("set_steer_car_api", 10);
    speed_publisher = node_obj2.advertise<std_msgs::Float32>("set_speed_car_api", 10);
}

CarControl::~CarControl() {}

void CarControl::setTypeTurning(TypeTurning typeofturning)
{
    typeOfTurning = typeofturning;
}

// void CarControl::setHasTrafficSign(bool HasTrafficSign)
// {
//     hasTrafficSign = HasTrafficSign;
// }
// bool CarControl::getHasTrafficSign()
// {
//     return hasTrafficSign;
// }
void CarControl::setTurnWhat(int turnwhat)
{
    turnWhat = turnwhat;
}
int CarControl::getTurnWhat()
{
    return turnWhat;
}
// void CarControl::setBeginRegconition(bool BeginRegconition)
// {
//     beginRegconition = BeginRegconition;
// }
// bool CarControl::getBeginRegconition()
// {
//     return beginRegconition;
// }
// void CarControl::setTurnCar(int TurnCar)
// {
//     turnCar = TurnCar;
// }
// int CarControl::getTurnCar()
// {
//     return turnCar;
// }
// void CarControl::setBeginTurn(bool BeginTurn)
// {
//     beginTurn = BeginTurn;
// }
// bool CarControl::getBeginTurn()
// {
//     return beginTurn;
// }
// void CarControl::setStopTurn(bool StopTurn)
// {
//     stopTurn = StopTurn;
// }
// bool CarControl::getStopTurn()
// {
//     return stopTurn;
// }

Status CarControl::getStatus()
{
    return statusCar;
}

void CarControl::setStatus(Status status)
{
    preStatusCar = statusCar;
    statusCar = status;
}

Status CarControl::getPreStatus()
{
    return preStatusCar;
}

float CarControl::errorAngle(const Point &dst)
{
    if (dst.x == carPos.x)
        return 0;
    if (dst.y == carPos.y)
        return (dst.x < carPos.x ? -90 : 90);
    double pi = acos(-1.0);
    double dx = dst.x - carPos.x;
    double dy = carPos.y - dst.y;
    if (dx < 0)
        return -atan(-dx / dy) * 180 / pi;
    return atan(dx / dy) * 180 / pi;
}

void CarControl::driverCar(const vector<Point> &left, const vector<Point> &right, float velocity)
{
    int i = left.size() - 11;
    float error = preError;
    float scaleL = 0.5, scaleR = 0.5;
    switch (statusCar)
    {
    case SIGN:
        velocity = 30;
        if(turnWhat == -1){
            scaleL = 0.60;
            scaleR = 0.40;
        } else{
            scaleL = 0.40;
            scaleR = 0.60;
        }
    case TURING:
        velocity = 20;
        if(turnWhat == 1){
            scaleL = 0.60;
            scaleR = 0.40;
        } else{
            scaleL = 0.40;
            scaleR = 0.60;
        }
    case NOSIGN:
        while (left[i] == DetectLane::null && right[i] == DetectLane::null)
        {
            i--;
            if (i < 0)
                break;
        }
        if(i >= 0){
            if (left[i] != DetectLane::null && right[i] != DetectLane::null)
            {
                error = errorAngle((left[i] * scaleL + right[i] * scaleR));
                // CENTER = (left[i] * scaleL + right[i] * scaleR);
            }
            else if (left[i] != DetectLane::null)
            {
                error = errorAngle(left[i] + Point(laneWidth * 0.4 , 0));
                // CENTER = left[i] + Point(laneWidth * 0.50, 0);
            }
            else
            {
                error = errorAngle(right[i] - Point(laneWidth * 0.4 , 0));
                // CENTER = right[i] - Point(laneWidth * 0.50, 0);
            }
        }
        break;
    // case TURING:
    //     // error = typeOfTurning.error * turnWhat;
    //     // velocity = typeOfTurning.velocity;
        
    //     break;
    }
    preError = error;

    std_msgs::Float32 angle;
    std_msgs::Float32 speed;
    // cout << "turnCar " << turnCar << endl;

    // if (turnCar != 0)
    // {
    //     cout << "Begin turn " <<  beginTurn << endl;
    //     if (beginTurn != 0)
    //     {

    //         if (stopTurn != true)
    //         {

    //             if (turnWhat == "TL")
    //             {
    //                 cout << "Tunrn what " << turnWhat << endl;
    //                 angle.data = -rotateAngle;
    //             }
    //             else
    //             {
    //                 cout << "Tunrn what " << turnWhat << endl;
    //                 angle.data = rotateAngle;
    //             }

    //             speed.data = 30;
    //         }
    //         // else
    //         // {
    //         //     angle.data = error;
    //         //     speed.data = velocity;
    //         // }

    //     }
    //     else
    //     {
    //         angle.data = error;
    //         speed.data = 30;
    //     }
    // }
    // else
    // {
    //     angle.data = error;
    //     speed.data = velocity;
    // }

    angle.data = error;
    speed.data = velocity;

    steer_publisher.publish(angle);
    // cout << "angle " << angle.data << endl
    //      << "speed " << speed.data << endl;
    // ghi vao file Log
    // cout << "Van toc hien tai cua xe: " << speed.data << "\t" << "Goc quay cua xe: " << angle.data << "\t";
    speed_publisher.publish(speed);
}