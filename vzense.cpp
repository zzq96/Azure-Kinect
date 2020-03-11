#pragma once
#include "vzense.h"

static void Opencv_Depth(uint32_t slope, int height, int width, uint8_t* pData, cv::Mat& dispImg)
{
	dispImg = cv::Mat(height, width, CV_16UC1, pData);
	cv::Point2d pointxy(width / 2, height / 2);
	int val = dispImg.at<ushort>(pointxy);
	char text[20];
#ifdef _WIN32
	sprintf_s(text, "%d", val);
#else
	snprintf(text, sizeof(text), "%d", val);
#endif
	dispImg.convertTo(dispImg, CV_8U, 255.0 / slope);
	applyColorMap(dispImg, dispImg, cv::COLORMAP_RAINBOW);
	int color;
	if (val > 2500)
		color = 0;
	else
		color = 4096;
	circle(dispImg, pointxy, 4, cv::Scalar(color, color, color), -1, 8, 0);
	putText(dispImg, text, pointxy, cv::FONT_HERSHEY_DUPLEX, 2, cv::Scalar(color, color, color));
}

Vzense::Vzense() {
    /*fx = 463.021, fy = 462.489, cx = 331.657, cy = 238.224;
	PsReturnStatus status;
	uint32_t deviceCount = 0;

	PsDepthRange depthRange = PsNearRange;
	PsDataMode dataMode = PsDepthAndRGB_30;
	status = Ps2_Initialize();
	if (status != PsReturnStatus::PsRetOK){
		std::cout << "PsInitialize failed!" << std::endl;
		system("pause");
	}

GET:
	status = Ps2_GetDeviceCount(&deviceCount);
	if (status != PsReturnStatus::PsRetOK){
		std::cout << "PsGetDeviceCount failed!" << std::endl;
		system("pause");

	}
	std::cout << "Get device count: " << deviceCount << std::endl;
	if (0 == deviceCount){
		std::this_thread::sleep_for(std::chrono::seconds(1));
		goto GET;
	}
	PsDeviceInfo* pDeviceListInfo = new PsDeviceInfo[deviceCount];
	status = Ps2_GetDeviceListInfo(pDeviceListInfo, deviceCount);
	deviceHandle = 0;
	status = Ps2_OpenDevice(pDeviceListInfo->uri, &deviceHandle);
	if (status != PsReturnStatus::PsRetOK){
		std::cout << "OpenDevice failed!" << std::endl;
		system("pause");
	}
	sessionIndex = 0;

	Ps2_StartStream(deviceHandle, sessionIndex);
	loop = std::thread(&Vzense::frameLoop, this, 60);*/
}

void Vzense::frameLoop(double thresh) {
	PsReturnStatus status;
	status = Ps2_SetThreshold(deviceHandle, sessionIndex, thresh);
	if (PsRetOK == status) {
		std::cout << "Set background threshold value: " << thresh << std::endl;
	}
	else {
		std::cout << "Set background threshold error,check if the datamode is WDR mode" << std::endl;
	}
	for (;;) {
		PsFrameReady frameReady = { 0 };
		PsFrame depthFrame = { 0 };
		status = Ps2_ReadNextFrame(deviceHandle, sessionIndex, &frameReady);
		status = Ps2_GetFrame(deviceHandle, sessionIndex, PsDepthFrame, &depthFrame);
		mu.lock();
		currentFrame = cv::Mat(depthFrame.height, depthFrame.width, CV_16UC1, depthFrame.pFrameData);
		mu.unlock();
	}
}

cv::Mat Vzense::getCurrentFrame() {
	return currentFrame.clone();
}

double Vzense::getPackageHeight() {
	cv::Mat img = getCurrentFrame();
    double t = (double)cv::getTickCount();
    int row = img.rows, col = img.cols;

    int roiStartX = 180, roiStartY = 190, roiWidth = 270, roiHeight = 230;
    cv::Rect roi = cv::Rect(roiStartX, roiStartY, roiWidth, roiHeight);
    cv::Mat mask = cv::Mat::zeros(row, col, CV_8UC1);
    mask(roi).setTo(255);
    cv::Mat tmp;
    img.copyTo(tmp, mask);

    cv::Mat edit = cv::Mat::zeros(row, col, CV_8UC1);
    for (int i = 0; i < row; ++i) {
        for (int j = 0; j < col; ++j) {
            if (tmp.at<uint16_t>(i, j) != 0 && tmp.at<uint16_t>(i, j) != 65535) {
                edit.at<uchar>(i, j) = 255;
            }
        }
    }
    std::vector<std::vector<cv::Point>> contours;
    findContours(edit, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE, cv::Point());
    if (contours.size() == 0) return -1;
    int maxSize = 0, cIndex = 0;
    for (int i = 0; i < contours.size(); ++i) {
        if (contours[i].size() > maxSize) {
            maxSize = contours[i].size();
            cIndex = i;
        }
    }
    cv::RotatedRect minRect;
    cv::Point2f vertices[4];
    minRect = cv::minAreaRect(cv::Mat(contours[cIndex]));
    cv::Mat final = cv::Mat::zeros(img.size(), CV_8UC3);
    if (minRect.size.area() > 100) {
        std::vector<std::vector<cv::Point> > vpts;
        vpts.push_back(contours[cIndex]);
        mask = cv::Mat::zeros(row, col, CV_8UC1);
        fillPoly(mask, vpts, cv::Scalar(255, 255, 255), 8, 0);
        bitwise_and(img, img, final, mask);

        double topX = 0, topY = 0, topZ = 0, bottomX = 0, bottomY = 0, bottomZ = 0;
        int flag = 0;
        for (int i = roiStartY; i < roiStartY + roiHeight; ++i) {
            for (int j = roiStartX; j < roiStartX + roiWidth; ++j) {
                if (final.at<uint16_t>(i, j) > 0) {
                    flag = 1;
                    topZ = (double)(final.at<uint16_t>(i, j));
                    topX = ((double)j - cx) * topZ / fx;
                    topY = ((double)i - cy) * topZ / fy;
                    break;
                }
            }
            if (flag) break;
        }
        flag = 0;
        for (int i = roiStartY + roiHeight - 1; i >= roiStartY; --i) {
            for (int j = roiStartX + roiWidth - 1; j >= roiStartX; --j) {
                if (final.at<uint16_t>(i, j) > 0) {
                    flag = 1;
                    bottomZ = (double)(final.at<uint16_t>(i, j));
                    bottomX = ((double)j - cx) * bottomZ / fx;
                    bottomY = ((double)i - cy) * bottomZ / fy;
                    break;
                }
            }
            if (flag) break;
        }
        double diagonal[] = { topX - bottomX, topY - bottomY, topZ - bottomZ };

        double sx = 0, sy = 0, sz = 0,
            sxx = 0, syy = 0, szz = 0,
            sxy = 0, syz = 0, sxz = 0;
        int pointCount = 0;
        for (int i = roiStartY; i < roiStartY + roiHeight; ++i) {
            for (int j = roiStartX; j < roiStartX + roiWidth; ++j) {
                if (final.at<uint16_t>(i, j) > 0) {
                    double z = (double)(final.at<uint16_t>(i, j));
                    double x = ((double)j - cx) * bottomZ / fx;
                    double y = ((double)i - cy) * bottomZ / fy;
                    sx += x; sy += y; sz += z;
                    sxx += x * x; syy += y * y; szz += z * z;
                    sxy += x * y; syz += y * z; sxz += x * z;
                    ++pointCount;
                }
            }
        }
        double sc = ((double)1.0) / pointCount;
        double K[3][3] = {
            {sxx - sx * sx * sc,sxy - sx * sy * sc,sxz - sx * sz * sc},
            {                 0,syy - sy * sy * sc,syz - sy * sz * sc},
            {                 0,                 0,szz - sz * sz * sc}
        };
        K[1][0] = K[0][1]; K[2][0] = K[0][2]; K[2][1] = K[1][2];
        double sv[3] = { 0,0,0 };
        double V[3][3] = { 0 };
        LA::eig33sym(K, sv, V);

        double heightCandidate_0 = 0, heightCandidate_1 = 0, heightCandidate_2 = 0;
        bool possibleChoice_0 = false, possibleChoice_1 = false, possibleChoice_2 = false;
        if (abs(V[1][0]) > abs(V[0][0]) && abs(V[1][0]) > abs(V[2][0])) possibleChoice_0 = true;
        if (abs(V[1][1]) > abs(V[0][1]) && abs(V[1][1]) > abs(V[2][1])) possibleChoice_1 = true;
        if (abs(V[1][2]) > abs(V[0][2]) && abs(V[1][2]) > abs(V[2][2])) possibleChoice_2 = true;

        if (possibleChoice_0) heightCandidate_0 = abs(diagonal[0] * V[0][0] + diagonal[1] * V[1][0] + diagonal[2] * V[2][0]);
        if (possibleChoice_1) heightCandidate_1 = abs(diagonal[0] * V[0][1] + diagonal[1] * V[1][1] + diagonal[2] * V[2][1]);
        if (possibleChoice_2) heightCandidate_2 = abs(diagonal[0] * V[0][2] + diagonal[1] * V[1][2] + diagonal[2] * V[2][2]);

        double height = std::max(heightCandidate_0, std::max(heightCandidate_1, heightCandidate_2));

        t = (cv::getTickCount() - t) / cv::getTickFrequency();
        std::cout << "The estimated height is: " << height << "mm" << std::endl;
        std::cout << std::endl;
        return height;
    }
    else {
        return -1;
    }
}