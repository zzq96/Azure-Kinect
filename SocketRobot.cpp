#include <stdio.h>
#include <iostream>
#include <string.h>
#include <string>
#include "SocketRobot.h"


#define SEND_DATA_SIZE 8
#define SEND_BUFFER_LEN (SEND_DATA_SIZE * 12)
#define REC_DATA_SIZE 12
#define REC_BUFFER_LEN (REC_DATA_SIZE * 6)

using namespace std;

SocketRobot::SocketRobot() {
    //Socket Setup
    struct sockaddr_in addr;
    struct sockaddr_in client;
    addr.sin_port = htons(PORT_NUM);
    addr.sin_family = AF_INET;
    addr.sin_addr.S_un.S_addr = INADDR_ANY;
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 0), &wsaData) != 0) {
        cout << "WSASetup failed!!!" << endl;
        assert(0);
    }
    SOCKET listenSock = socket(AF_INET, SOCK_STREAM, 0);
    if (listenSock == INVALID_SOCKET) {
        cout << "Socket: " << WSAGetLastError() << endl;
        assert(0);
    }
    if (bind(listenSock, (struct sockaddr*) & addr, sizeof(addr)) != 0) {
        cout << "Bind: " << WSAGetLastError() << endl;
        assert(0);
    }
    if (listen(listenSock, 5) != 0) {
        cout << "Listen: " << WSAGetLastError() << endl;
        assert(0);
    }
    int len = sizeof(client);
    cout << "Waiting for connection ..." << endl;
    this->acceptSock = accept(listenSock, (struct sockaddr*) & client, &len);
    if (this->acceptSock == INVALID_SOCKET) {
        cout << "Accept: " << WSAGetLastError() << endl;
        assert(0);
    }
    char receive_buf[1024] = {};
    recv(this->acceptSock, receive_buf, sizeof(receive_buf), 0);
    cout << receive_buf << endl;
    memset(receive_buf, 0, sizeof(receive_buf));
    recv(this->acceptSock, receive_buf, sizeof(receive_buf), 0);
    cout << receive_buf << endl;
}
void SocketRobot::moveRobot(float* coords) {
    //Receive "connected" from Client
    int check = 0;
    char receive_buf[REC_BUFFER_LEN] = {};

    memset(receive_buf, 0, sizeof(receive_buf));
    char startX[] = "00000.00";
    sprintf_s(startX, "%08.2f", coords[0]);
    char startY[] = "00000.00";
    sprintf_s(startY, "%08.2f", coords[1]);
    char startZ[] = "00000.00";
    sprintf_s(startZ, "%08.2f", coords[2]);
    char startRoll[] = "00000.00";
    sprintf_s(startRoll, "%08.2f", coords[3]);
    char startPitch[] = "00000.00";
    sprintf_s(startPitch, "%08.2f", coords[4]);
    char startYaw[] = "00000.00";
    sprintf_s(startYaw, "%08.2f", coords[5]);
    char endX[] = "00000.00";
    sprintf_s(endX, "%08.2f", coords[6]);
    char endY[] = "00000.00";
    sprintf_s(endY, "%08.2f", coords[7]);
    char endZ[] = "00000.00";
    sprintf_s(endZ, "%08.2f", coords[8]);
    char endRoll[] = "00000.00";
    sprintf_s(endRoll, "%08.2f", coords[9]);
    char endPitch[] = "00000.00";
    sprintf_s(endPitch, "%08.2f", coords[10]);
    char endYaw[] = "00000.00";
    sprintf_s(endYaw, "%08.2f", coords[11]);
    char buf[SEND_BUFFER_LEN + 1];
    sprintf_s(buf, "%s%s%s%s%s%s%s%s%s%s%s%s", startX, startY, startZ, startRoll, startPitch, startYaw,
        endX, endY, endZ, endRoll, endPitch, endYaw);
    check = send(this->acceptSock, buf, sizeof(buf) - 1, 0);
    
    check = recv(this->acceptSock, receive_buf, sizeof(receive_buf), 0);
    cout << receive_buf << endl;

    memset(receive_buf, 0, sizeof(receive_buf));
    check = recv(this->acceptSock, receive_buf, sizeof(receive_buf), 0);
    cout << receive_buf << endl;
}

void SocketRobot::close() {
    float end[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    moveRobot(end);
    closesocket(this->acceptSock);
    WSACleanup();
}
