#pragma once
#include <iostream>
#include <thread>
#include <mutex>
#include <opencv2/opencv.hpp>
#include "Vzense_api2.h"
#include "eig33sym.hpp"

class Vzense {
private:
	cv::Mat currentFrame;
	std::mutex mu;
	PsDeviceHandle deviceHandle;
	uint32_t sessionIndex;
	std::thread loop;
	double fx;
	double fy;
	double cx;
	double cy;
public:
	Vzense();
	~Vzense() { loop.detach(); };
	void frameLoop(double thresh);
	cv::Mat getCurrentFrame();
	double getPackageHeight();
};