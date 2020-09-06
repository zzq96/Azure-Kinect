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
#include <OpenNR-IF.h>


//����ڲλ���ϵ�������ļ�
string caliberation_camera_file = "caliberation_camera.xml";
//��������ļ�
string Homo_cam2base_file = "Homo_cam2base.xml";

//��ʼ��kinect���,�����и��������صĲ����ͺ���
k4a::KinectAPI kinect(caliberation_camera_file, true);
//��ʼ��robot�࣬�����и�����κ��������ĺ���
Robot rob(Homo_cam2base_file, &kinect, true);

//ԭʼ�����ͼ�����ͼ��α��ɫͼ��������ͼ��������α��ɫͼ
cv::Mat depthMatOld, colorMatOld, depthcolorMatOld, irMatOld, ircolorMatOld;
//ͨ������ϵ��У�������ͼƬ
cv::Mat depthMat, colorMat, depthcolorMat, irMat, ircolorMat;

//���ֵ�������У��������ͼ��
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

	//����������Ϊֹ�ж��ٸ����
	int cnt = 0;
	while (true)
	{

		//�õ�ԭʼ�ĸ�ͼƬ�����Ҳ���������õĲ���У��ͼƬ
		kinect.GetOpenCVImage(colorMatOld, depthMatOld, depthcolorMatOld, irMat, FALSE);

		///*���������ļ��ṩ���ڲκͻ���ϵ��У��ͼ��*/
		kinect.undistort(depthMatOld, depthMat, "depth");
		kinect.undistort(depthcolorMatOld, depthcolorMat, "depth");
		kinect.undistort(colorMatOld, colorMat, "color");

		//���������ļ��ṩ����ν�colorͼת�������ͼ�ӽ�
		cv::Mat colorMatRevise(depthMat.rows, depthMat.cols, CV_8UC4, cv::Scalar(0));
		kinect.ConvertColor2Depth(colorMat, depthMat, colorMatRevise);

		//�����ǵõ���ݵĳ���
		double* center;
		vector<VertexType> highestPlanePoints_3D;
		cv::Point2f vertices[4];
		vector<cv::Mat> masks;

		//��rgbͼ���������������ؽ��
		getMasks(colorMatRevise, masks);

		cout << "��" << masks.size() << "�����" << endl;

		double* x_axis, * y_axis, * z_axis;
		colorMatRevise = processImg(colorMatRevise, depthMat, masks, center, x_axis, y_axis, z_axis, highestPlanePoints_3D, vertices);

		//����ͼƬ�����ڳ�����debug
		string name = "Data/imgs/img" + std::to_string(cnt);
		cv::imwrite(name + "_depth.png", depthMat);
		cv::imwrite(name + "_depthcolor.png", colorMatRevise);

		//���û��ݾ�����1��
		if (highestPlanePoints_3D.size() == 0)
		{
			Sleep(1000);
			continue;
		}
		//pca
		cv::Mat centerMat = (cv::Mat_<double>(3, 1) << center[0], center[1], center[2]);
		cv::Mat eigenvectors = (cv::Mat_<double>(3, 3)<< x_axis[0], y_axis[0], z_axis[0],
			x_axis[1], y_axis[1], z_axis[1], 
			x_axis[2], y_axis[2], z_axis[2]);
		cout << x_axis[0] << " " << x_axis[1] << " " << x_axis[2] << endl;
		cout << eigenvectors << endl;

		cout << centerMat << endl;
		cv::Mat express2depthHomo = rob.RT2HomogeneousMatrix(eigenvectors, centerMat);
		cout << rob.depth_Homo_cam2base << " " << express2depthHomo << endl;
		cv::Mat express2roboticHomo = rob.depth_Homo_cam2base * express2depthHomo;
		cout << express2roboticHomo << endl;
		cv::Mat express2roboticRotation, express2roboticTranslation;
		rob.HomogeneousMtr2RT(express2roboticHomo, express2roboticRotation, express2roboticTranslation);
		if (express2roboticRotation.at<double>(0, 0) < 0)
		{
			express2roboticRotation.at<double>(2, 0) *= -1;
			express2roboticRotation.at<double>(1, 0) *= -1;
			express2roboticRotation.at<double>(0, 0) *= -1;
		}
		if (express2roboticRotation.at<double>(2, 2) < 0)
		{
			express2roboticRotation.at<double>(2, 2) *= -1;
			express2roboticRotation.at<double>(1, 2) *= -1;
			express2roboticRotation.at<double>(0, 2) *= -1;
		}
		cv::Mat vector_y = express2roboticRotation.colRange(2, 3).cross(express2roboticRotation.colRange(0, 1)).reshape(0, 3);
		//cout << vector_y << endl;
		//cout << express2roboticRotation.col(1) << endl;
		vector_y.copyTo(express2roboticRotation.col(1));
		cv::Vec3f eulerAngles = rob.rotationMatrixToEulerAngles(express2roboticRotation);
		cout << "----------------------pca---------------" << endl;
		cout << express2roboticRotation << endl;
		cout << "����Ϊ" << endl;
		cout << express2roboticTranslation << endl;
		cout << "ŷ����Ϊ" << endl;
		cout << eulerAngles<< endl;

		////������Ӿ��Σ�����������
		//double x = 0, y = 0;
		//for (int j = 0; j < 4; j++)
		//{
		//	x += vertices[j].x;
		//	y += vertices[j].y;
		//}
		//x /= 4, y /= 4;

		///*�������*/
		//cv::Mat point2D(3, 1, CV_64F, cv::Scalar(0));
		//point2D.at<double>(2, 0) = 1;
		//cv::Mat point3D;
		//point2D.at<double>(0, 0) = x;
		//point2D.at<double>(1, 0) = y;
		//point2D.at<double>(2, 0) = 1;

		///*�õ����ĵ�����*/
		//double Zc = rob.getDepthValue(depthMat, y, x, 6);

		////ƽ�淨����
		//cv::Mat rotationMatrix = rob.calRotationMatrix(depthMat, vertices, 0.6);
		//eulerAngles = rob.rotationMatrixToEulerAngles(rotationMatrix);

		////cout << "���Ϸ��������ת����:" << endl;
		////cout << rotationMatrix << endl;

		////�õ����ĵ��3D����
		//point3D = rob.calPoint3D(depthMat, point2D);

		//cout << "----------------------ori---------------" << endl;
		//cout << rotationMatrix << endl;
		//cout << "����:" << endl;
		//cout << point3D << endl;
		//cout << "ŷ����:" << endl;
		//cout << eulerAngles << endl;

		float coords[12];
		cout << express2roboticTranslation.at<double>(0, 0) << endl;
		cout << express2roboticTranslation.at<double>(1, 0) << endl;
		cout << express2roboticTranslation.at<double>(2, 0) << endl;
		coords[0] = express2roboticTranslation.at<double>(0, 0);
		coords[1] = express2roboticTranslation.at<double>(1, 0);
		coords[2] = express2roboticTranslation.at<double>(2, 0) - 4;
		coords[3] = eulerAngles[0];
		coords[4] = eulerAngles[1];
		coords[5] = eulerAngles[2];
		coords[6] = 698 + 50;
		//�ų�һ��
		//coords[7] = 342 - 750 + cnt * 220;
		//�ŵ��̶�λ��
		coords[7] = 0;
		//��ݼ���
		cnt++;
		//���õĸ߶�
		if (coords[2] > 160)
			coords[8] = 160 - 50;
		else
			coords[8] = coords[2] - 50;

		//ƽ�ŷ�
		coords[9] = 0;
		coords[10] = 0;
		coords[11] = 0;

		kinect.ShowOpenCVImage(colorMatRevise, "depthcolor", useRobot);
		if (useRobot)
			sr->moveRobot(coords);
	}
	return 0;
}
