#include "SocketRobot.h"
#define IP_ADDRESS "192.168.1.2"
#define NUM_OF_AXES (6)
using namespace std;

bool floatCmp(float a, float b) {
    if (abs(a - b) <= 1) return true;
    else return false;
}

SocketRobot::SocketRobot() {
    NACHI_COMMIF_INFO Info;
    ZeroMemory(&Info, sizeof(Info));
    char tmp[] = "192.168.1.2";
    Info.pcAddrs = tmp;
    Info.lKind = NR_DATA_XML;
    nXmlOpenId = NR_Open(&Info);
    printf("Robot connection: %d", nXmlOpenId);
}

void SocketRobot::moveRobotToAndFro(float* coords) {
    if (this->nXmlOpenId <= 0) {
        printf("Robot connection error!");
        return;
    }

    moveRobotTo(coords, true);
    //startVaccum();    
    
    moveRobotTo(coords + 6, false);
    //endVaccum();
}

void SocketRobot::moveRobotTo(float* coords, bool startOrEnd) {
    NR_POSE PoseMid = { coords[0], coords[1], coords[2] + 250.0f, coords[3], coords[4], coords[5] };
    NR_POSE PoseEnd = { coords[0], coords[1], coords[2], coords[3], coords[4], coords[5] };
    if (int nErr = NR_CtrlMoveX(this->nXmlOpenId, &PoseMid, 0, 1, 0) != NR_E_NORMAL) {
        printf("NR_CtrlMoveX error : %d\n", nErr);
        return;
    }
    if (int nErr = NR_CtrlMoveX(this->nXmlOpenId, &PoseEnd, 2, 1, 0) != NR_E_NORMAL) {
        printf("NR_CtrlMoveX error : %d\n", nErr);
        return;
    }
    waitForRobot(coords);

    vaccum(startOrEnd);

    NR_POSE Pose = { 0.0f, 0.0f, 250.0f, 0.0f, 0.0f, 0.0f };
    if (int nErr = NR_CtrlMoveXR(this->nXmlOpenId, &Pose, 0, 1, 0) != NR_E_NORMAL) {
        printf("NR_CtrlMoveX error : %d\n", nErr);
        return;
    }
}

void SocketRobot::waitForRobot(float* coords) {
    Sleep(200);
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
    }
    Sleep(100);
}

void SocketRobot::vaccum(bool startOrEnd) {
    BOOL bValue[1];
    bValue[0] = startOrEnd;
    if (int nErr = NR_AcsGeneralOutputSignal(nXmlOpenId, bValue, TRUE, 1, 1) != NR_E_NORMAL) {
        printf("NR_AcsGeneralOutputSignal reading error : %d\n", nErr);
        return;
    }
    Sleep(500);
}

void SocketRobot::startVaccum() {
    BOOL bValue[1];
    bValue[0] = TRUE;
    if (int nErr = NR_AcsGeneralOutputSignal(nXmlOpenId, bValue, TRUE, 1, 1) != NR_E_NORMAL) {
        printf("NR_AcsGeneralOutputSignal reading error : %d\n", nErr);
        return;
    }
    Sleep(500);
}

void SocketRobot::endVaccum() {
    BOOL bValue[1];
    bValue[0] = false;
    if (int nErr = NR_AcsGeneralOutputSignal(nXmlOpenId, bValue, TRUE, 1, 1) != NR_E_NORMAL) {
        printf("NR_AcsGeneralOutputSignal reading error : %d\n", nErr);
        return;
    }
    Sleep(500);
}