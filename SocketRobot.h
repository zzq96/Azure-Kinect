#pragma once
#include <winsock2.h>
#include <cassert>
#define PORT_NUM 10030

//Ready for rewrite
class SocketRobot {
private:
	SOCKET acceptSock;
public:
	SocketRobot();
	void moveRobot(float* coords);
	void close();
};