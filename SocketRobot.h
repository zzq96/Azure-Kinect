#pragma once
#include <windows.h>
#include <iostream>
#include <OpenNR-IF.h>
#include <thread>
#include <mutex>
#include <ctime>
#include "Robot.h"

extern Robot rob;

class SocketRobot {
private:
	const double kFx = 505;
	const double kFy = 505;
	const double kCx = 334;
	const double kCy = 334;
	const int kDepthWidth = 640;
	const int kDepthHeight = 576;
	const int kScaleFactor = 1;
	int nXmlOpenId;
	bool motionFinished;
	std::mutex mu;
	std::vector<std::vector<double>> path_first;
	std::vector<std::vector<double>> path_second;
public:
	SocketRobot();
	~SocketRobot() {};
	double* getPose();
	void doMove(double* coords);
	void setMotionFinished(bool finished) { motionFinished = finished; }
	bool getMotionFinished() { return motionFinished; }
	void moveRobotMid(float* coords, int type);
	void moveRobotMid();
	void moveRobotToAndFro(double* coords);
	void moveRobotTo(double* coords, bool startOrEnd);
	void waitForRobot(double* coords, double thresh);
	void vaccum(bool startOrEnd);
};