#pragma once
#include <windows.h>
#include <iostream>
#include <OpenNR-IF.h>
#include <thread>
#include <mutex>

class SocketRobot {
private:
	int nXmlOpenId;
	bool motionFinished;
	std::mutex mu;
public:
	SocketRobot();
	~SocketRobot() {};
	void doMove(float* coords);
	void setMotionFinished(bool finished) { motionFinished = finished; }
	bool getMotionFinished() { return motionFinished; }
	void moveRobotMid(float* coords);
	void moveRobotToAndFro(float* coords);
	void moveRobotTo(float* coords, bool startOrEnd);
	void waitForRobot(float* coords);
	void vaccum(bool startOrEnd);
};