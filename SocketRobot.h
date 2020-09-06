#pragma once
#include <windows.h>
#include <iostream>
#include <OpenNR-IF.h>

class SocketRobot {
private:
	int nXmlOpenId;
public:
	SocketRobot();
	void moveRobotToAndFro(float* coords);
	void moveRobotTo(float* coords, bool startOrEnd);
	void waitForRobot(float* coords);
	void vaccum(bool startOrEnd);
	void startVaccum();
	void endVaccum();
};