#include "SocketRobot.h"
#define IP_ADDRESS "192.168.1.2"
#define NUM_OF_AXES (6)
#define START_VACCUM true
#define END_VACCUM false
using namespace std;

bool floatCmp(float a, float b) {
    if (abs(a - b) <= 1) return true;
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

void SocketRobot::doMove(float* coords) {
    thread t1(&SocketRobot::moveRobotToAndFro, this, coords);
    t1.detach();
}

void SocketRobot::moveRobotMid(float* coords) {
    NR_POSE Pose = { coords[0], coords[1], coords[2], coords[3], coords[4], coords[5] };
    if (int nErr = NR_CtrlMoveX(this->nXmlOpenId, &Pose, 0, 1, 0) != NR_E_NORMAL) {
        printf("NR_CtrlMoveX error : %d\n", nErr);
        return;
    }
}

void SocketRobot::moveRobotToAndFro(float* coords) {
    int nValue = 0;
    if (int nErr = NR_AcsInterpolationKind(nXmlOpenId, &nValue, TRUE) != NR_E_NORMAL) {
        printf("Interpolation setting gone wrong!");
    }
    if (this->nXmlOpenId <= 0) {
        printf("Robot connection error!");
        return;
    }

    float mid[] = { 424.354f, 245.013f, 531.977f, 30.0f, 0.0f, 0.0f };
    moveRobotMid(mid);
    moveRobotTo(coords, START_VACCUM);
    moveRobotMid(mid);   
    moveRobotTo(coords + 6, END_VACCUM);
    printf("Current task finished.\n");
}

void SocketRobot::moveRobotTo(float* coords, bool startOrEnd) {
    NR_POSE PoseAbove = { coords[0], coords[1], coords[2] + 250.0f, coords[3], coords[4], coords[5] };
    NR_POSE PoseEnd = { coords[0], coords[1], coords[2], coords[3], coords[4], coords[5] };
    if (int nErr = NR_CtrlMoveX(this->nXmlOpenId, &PoseAbove, 0, 1, 0) != NR_E_NORMAL) {
        printf("NR_CtrlMoveX error : %d\n", nErr);
        return;
    }
    if (int nErr = NR_CtrlMoveX(this->nXmlOpenId, &PoseEnd, 2, 1, 0) != NR_E_NORMAL) {
        printf("NR_CtrlMoveX error : %d\n", nErr);
        return;
    }
    waitForRobot(coords);

    mu.lock();
    if (!startOrEnd) motionFinished = true;
    mu.unlock();
    vaccum(startOrEnd);

    if (int nErr = NR_CtrlMoveX(this->nXmlOpenId, &PoseAbove, 1, 1, 0) != NR_E_NORMAL) {
        printf("NR_CtrlMoveX error : %d\n", nErr);
        return;
    }
    float tmp[] = { coords[0], coords[1], coords[2] + 250.0f, coords[3], coords[4], coords[5] };
    waitForRobot(tmp);
}

void SocketRobot::waitForRobot(float* coords) {
    Sleep(100);
    while (1) {
        float fValue[6];
        ZeroMemory(&fValue, sizeof(fValue));
        if (int nErr = NR_AcsToolTipPos(nXmlOpenId, fValue, 1, 6) == NR_E_NORMAL) {
            if (floatCmp(fValue[0], coords[0])
                && floatCmp(fValue[1], coords[1])
                && floatCmp(fValue[2], coords[2])
                && floatCmp(fValue[3], coords[3])
                && floatCmp(fValue[4], coords[4])
                && floatCmp(fValue[5], coords[5])
                ) {
                break;
            }
        }
        Sleep(50);
    }
}

void SocketRobot::vaccum(bool startOrEnd) {
    BOOL bValue[1];
    bValue[0] = startOrEnd;
    if (int nErr = NR_AcsGeneralOutputSignal(nXmlOpenId, bValue, TRUE, 1, 1) != NR_E_NORMAL) {
        printf("NR_AcsGeneralOutputSignal reading error : %d\n", nErr);
        return;
    }
    Sleep(200);
}