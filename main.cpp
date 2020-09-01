#include <winsock2.h>
#include <vector>
#include <stdio.h>
#include<string>
#include <stdlib.h>
#include <iostream>
#include <k4a/k4a.h>
#include "k4a_grabber.h"
#include "plane_detection.h"
#include<ctime>
#include <fstream>
#include"SocketRobot.h"
#include "flask.h"
#include "Robot.h"


//相机内参畸变系数所在文件
string caliberation_camera_file = "caliberation_camera.xml";
//外参所在文件
string Homo_cam2base_file = "Homo_cam2base.xml";

//初始化kinect相机,里面有各种相机相关的参数和函数
k4a::KinectAPI kinect(caliberation_camera_file, true);
//初始化robot类，里面有各种外参和坐标计算的函数
Robot rob(Homo_cam2base_file, &kinect, true);

//原始的深度图，深度图的伪彩色图，红外线图，红外线伪彩色图
cv::Mat depthMatOld, colorMatOld, depthcolorMatOld, irMatOld, ircolorMatOld;
//通过畸变系数校正过后的图片
cv::Mat depthMat, colorMat, depthcolorMat, irMat, ircolorMat;

//深度值经过外参校正后的深度图像
cv::Mat depthMatRevise;
int main()
{
	bool useRobot = false;
#ifdef ROBOT
	useRobot = true;
#endif // ROBOT

	SocketRobot* sr = NULL;
	if (useRobot)
		sr = new SocketRobot();

	//计数到现在为止有多少个快递
	int cnt = 0;
	while (true)
	{

		//得到原始的各图片，并且不用相机内置的参数校正图片
		kinect.GetOpenCVImage(colorMatOld, depthMatOld, depthcolorMatOld, irMat, FALSE);

		///*根据我们文件提供的内参和畸变系数校正图像*/
		kinect.undistort(depthMatOld, depthMat, "depth");
		kinect.undistort(depthcolorMatOld, depthcolorMat, "depth");
		kinect.undistort(colorMatOld, colorMat, "color");

		//根据我们文件提供的外参将color图转化到深度图视角
		cv::Mat colorMatRevise(depthMat.rows, depthMat.cols, CV_8UC4, cv::Scalar(0));
		kinect.ConvertColor2Depth(colorMat, depthMat, colorMatRevise);

		//下面是得到快递的程序
		double* center;
		vector<VertexType> highestPlanePoints_3D;
		cv::Point2f vertices[4];
		vector<cv::Mat> masks;

		//把rgb图传给服务器并返回结果
		getMasks(colorMatRevise, masks);

		cout << "有" << masks.size() << "个快递" << endl;

		double* x_axis, * y_axis, * z_axis;
		colorMatRevise = processImg(colorMatRevise, depthMat, masks, center, x_axis, y_axis, z_axis, highestPlanePoints_3D, vertices);
		kinect.ShowOpenCVImage(colorMatRevise, "depthcolor", useRobot);

		//保存图片，用于出问题debug
		string name = "Data/imgs/img" + std::to_string(cnt);
		cv::imwrite(name + "_depth.png", depthMat);
		cv::imwrite(name + "_depthcolor.png", colorMatRevise);

		//如果没快递就休眠1秒
		if (highestPlanePoints_3D.size() == 0)
		{
			Sleep(1000);
			continue;
		}

		//根据外接矩形，计算快递中心
		double x = 0, y = 0;
		for (int j = 0; j < 4; j++)
		{
			x += vertices[j].x;
			y += vertices[j].y;
		}
		x /= 4, y /= 4;

		/*齐次坐标*/
		cv::Mat point2D(3, 1, CV_64F, cv::Scalar(0));
		point2D.at<double>(2, 0) = 1;
		cv::Mat point3D;
		point2D.at<double>(0, 0) = x;
		point2D.at<double>(1, 0) = y;
		point2D.at<double>(2, 0) = 1;

		/*得到中心点的深度*/
		double Zc = rob.getDepthValue(depthMat, y, x, 6);

		//平面法向量
		cv::Mat rotationMatrix = rob.calRotationMatrix(depthMat, vertices, 0.6);
		cv::Vec3f eulerAngles = rob.rotationMatrixToEulerAngles(rotationMatrix);

		cout << "最上方物体的旋转矩阵:" << endl;
		cout << rotationMatrix << endl;

		//得到中心点的3D坐标
		point3D = rob.calPoint3D(depthMat, point2D);

		cout << "坐标:" << endl;
		cout << point3D << endl;
		cout << "欧拉角:" << endl;
		cout << eulerAngles << endl;

		float coords[12];
		coords[0] = point3D.at<double>(0, 0);
		coords[1] = point3D.at<double>(1, 0);
		coords[2] = point3D.at<double>(2, 0) - 4;
		coords[3] = eulerAngles[0];
		coords[4] = eulerAngles[1];
		coords[5] = eulerAngles[2];
		coords[6] = 698 + 50;
		//放成一排
		//coords[7] = 342 - 750 + cnt * 220;
		//放到固定位置
		coords[7] = 0;
		//快递计数
		cnt++;
		//放置的高度
		if (coords[2] > 160)
			coords[8] = 160 - 50;
		else
			coords[8] = coords[2] - 50;

		//平着放
		coords[9] = 0;
		coords[10] = 0;
		coords[11] = 0;

		if (useRobot)
			sr->moveRobot(coords);
	}
	return 0;
}
