#include "SocketRobot.h"
#define IP_ADDRESS "192.168.1.2"
#define NUM_OF_AXES (6)
#define START_VACCUM true
#define END_VACCUM false

using namespace std;

string getTime() {
    struct tm t;
    time_t now;
    time(&now);
    localtime_s(&t, &now);
    char time[32];
    sprintf_s(time, "%d%02d%02d_%02d%02d%02d", t.tm_year + 1900,
        t.tm_mon + 1,
        t.tm_mday,
        t.tm_hour,
        t.tm_min,
        t.tm_sec);
    return time;
}

bool doubleCmp(float a, float b, float thresh) {
    if (abs(a - b) <= thresh) return true;
    else return false;
}

SocketRobot::SocketRobot() {
    motionFinished = true;
    NACHI_COMMIF_INFO Info;
    ZeroMemory(&Info, sizeof(Info));
    char tmp[] = "192.168.1.2";
    Info.pcAddrs = tmp;
    Info.lKind = NR_DATA_XML;
    nXmlOpenId = NR_Open(&Info);
    printf("Robot connection: %d\n", nXmlOpenId);
}

double* SocketRobot::getPose() {
    float fValue[6];
    ZeroMemory(&fValue, sizeof(fValue));
    int nErr = NR_AcsToolTipPos(nXmlOpenId, fValue, 1, 6);
    double xyz[] = { (double)fValue[0], (double)fValue[1], (double)fValue[2] };
    return xyz;
}

void SocketRobot::doMove(double* coords) {
    thread t1(&SocketRobot::moveRobotToAndFro, this, coords);
    t1.detach();
}

void SocketRobot::moveRobotMid(float* angles, int type) {
    //NR_POSE Pose = { coords[0], coords[1], coords[2], coords[3], coords[4], coords[5] };
    if (int nErr = NR_CtrlMoveJ(nXmlOpenId, angles, 6, type, 1) != NR_E_NORMAL) {
        printf("NR_CtrlMoveJ error : %d\n", nErr);
        return;
    }
}

void SocketRobot::moveRobotToAndFro(double* coords) {
    int nValue = 0;
    if (int nErr = NR_AcsInterpolationKind(nXmlOpenId, &nValue, TRUE) != NR_E_NORMAL) {
        printf("Interpolation setting gone wrong!");
    }
    if (this->nXmlOpenId <= 0) {
        printf("Robot connection error!");
        return;
    }

    /*double mid_x = (coords[0] + coords[6]) / 2;
    double mid_y = (coords[1] + coords[7]) / 2;
    double mid_z = coords[2] + 200;
    double mid[] = { mid_x, mid_y, mid_z, 30.0, 0.0, 0.0 };*/

    float midAngles[] = { 30.0f, 90.0f, 0.0f, 0.0f, -90.0f, 0.0f };
    /*moveRobotTo(coords, START_VACCUM);
    moveRobotMid(mid, 0);
    moveRobotTo(coords + 6, END_VACCUM);
    moveRobotMid(mid, 1);*/
    //cv::imwrite("D:\\Desktop\\saved_depth_imgs\\" + getTime() + ".png", heightEstimator->getCurrentFrame());
    
    moveRobotTo(coords, START_VACCUM);
    //moveRobotMid(mid, 0);
    /*waitForRobot(mid, 5);
    double height = heightEstimator->getPackageHeight();
    if (height > 0) {
        coords[8] = height - 90;
    }
    else {
        coords[9] = 100;
    }*/
    moveRobotTo(coords + 6, END_VACCUM);
    moveRobotMid(midAngles, 1);
    
    printf("Current task finished.\n");
}

void SocketRobot::moveRobotTo(double* coords, bool startOrEnd) {
    NR_POSE PoseAbove = { coords[0], coords[1], coords[2] + 300.0f, coords[3], coords[4], coords[5] };
    NR_POSE PoseEnd = { coords[0], coords[1], coords[2], coords[3], coords[4], coords[5] };

    if (startOrEnd) {
        if (int nErr = NR_CtrlMoveX(this->nXmlOpenId, &PoseAbove, 0, 1, 0) != NR_E_NORMAL) {
            printf("NR_CtrlMoveX error : %d\n", nErr);
            return;
        }
        if (int nErr = NR_CtrlMoveX(this->nXmlOpenId, &PoseEnd, 2, 1, 0) != NR_E_NORMAL) {
            printf("NR_CtrlMoveX error : %d\n", nErr);
            return;
        }
    }
    else {
        float fValue[NUM_OF_AXES] = { 0 };
        int nErr = NR_AcsAxisTheta(nXmlOpenId, fValue, 1, NUM_OF_AXES);
        float curBaseAngle = 0;
        if (NR_E_NORMAL == nErr) {
            curBaseAngle = fValue[0];
        }
        else {
            printf("NR_AcsAxisTheta error : %d\n", nErr);
            return;
        }
        fValue[0] = -60.0f - curBaseAngle; fValue[1] = 0.0f; fValue[2] = 0.0f;
        fValue[3] = 0.0f; fValue[4] = 0.0f; fValue[5] = 0.0f;
        if (int nErr = NR_CtrlMoveJA(this->nXmlOpenId, fValue, 6, 0, 1) != NR_E_NORMAL) {
            printf("NR_CtrlMoveX error : %d\n", nErr);
            return;
        }

        if (int nErr = NR_CtrlMoveX(this->nXmlOpenId, &PoseEnd, 2, 1, 0) != NR_E_NORMAL) {
            printf("NR_CtrlMoveX error : %d\n", nErr);
            return;
        }
;   }

    waitForRobot(coords, 5);
    if (startOrEnd) vaccum(startOrEnd);

    mu.lock();
    if (!startOrEnd) motionFinished = true;
    mu.unlock();
    if(!startOrEnd) vaccum(startOrEnd);

    NR_POSE PoseAboveSecond = { coords[0], coords[1], coords[2] + 300.0f, 0.0f, 0.0f, 0.0f };
    if (int nErr = NR_CtrlMoveX(this->nXmlOpenId, &PoseAboveSecond, 0, 1, 0) != NR_E_NORMAL) {
        printf("NR_CtrlMoveX error : %d\n", nErr);
        return;
    }
    //delete[] angles;
}

void SocketRobot::waitForRobot(double* coords, double thresh) {
    Sleep(50);
    while (1) {
        float fValue[6];
        ZeroMemory(&fValue, sizeof(fValue));
        if (int nErr = NR_AcsToolTipPos(nXmlOpenId, fValue, 1, 6) == NR_E_NORMAL) {
            if (doubleCmp(fValue[0], coords[0], thresh)
                && doubleCmp(fValue[1], coords[1], thresh)
                && doubleCmp(fValue[2], coords[2], thresh)
                && doubleCmp(fValue[3], coords[3], thresh)
                && doubleCmp(fValue[4], coords[4], thresh)
                && doubleCmp(fValue[5], coords[5], thresh)
                ) {
                break;
            }
        }
        Sleep(5);
    }
}

void SocketRobot::vaccum(bool startOrEnd) {
    BOOL bValue[1];
    bValue[0] = startOrEnd;
    if (int nErr = NR_AcsGeneralOutputSignal(nXmlOpenId, bValue, TRUE, 1, 1) != NR_E_NORMAL) {
        printf("NR_AcsGeneralOutputSignal reading error : %d\n", nErr);
        return;
    }
    if (!startOrEnd) Sleep(400);
}