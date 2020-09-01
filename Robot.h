#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui_c.h>
#include"k4a_grabber.h"
class Robot
{
public:
	k4a::KinectAPI *kinect;
	cv::Mat color_R_base2cam, color_t_base2cam;
	cv::Mat depth_R_base2cam, depth_t_base2cam;
	cv::Mat color_R_cam2base, color_t_cam2base;
	cv::Mat depth_R_cam2base, depth_t_cam2base;
	cv::Mat color_Homo_cam2base, depth_Homo_cam2base;
	Robot(const std::string& Homo_cam2base_file, k4a::KinectAPI *kinect, bool verbose);
	const double PI = 3.1415926;
	// Checks if a matrix is a valid rotation matrix.
	bool isRotationMatrix(cv::Mat& R);

	// Calculates rotation matrix to euler angles
	// The result is the same as MATLAB except the order
	// of the euler angles ( x and z are swapped ).
	cv::Vec3f rotationMatrixToEulerAngles(cv::Mat& R);
	//计算点(row,col）的深度，为了降低噪声影响，以周围2*size大小的正方形区域为采样点。

	double getDepthValue(cv::Mat depthMat, int row, int col, int size);
	//计算点point2D的深度，为了降低噪声影响，以周围2*size大小的正方形区域为采样点。
	cv::uint16_t getDepth(const cv::Mat& depthMat, cv::Mat& point2D);
	double getLength(cv::Mat a);
	double getDistance(cv::Point3f a, cv::Point3f b);
	//单应矩阵转化为旋转矩阵和平移向量
	void HomogeneousMtr2RT(cv::Mat& HomoMtr, cv::Mat& R, cv::Mat& T);
	
	//旋转矩阵和平移向量转化为单应矩阵
	cv::Mat RT2HomogeneousMatrix(const cv::Mat& R, const cv::Mat& T);
	//传入像素坐标系的点，返回机械臂坐标系下的坐标
	void calPoint3D(cv::Mat point2D, cv::Point3f& real, cv::uint16_t Zc);
	cv::Mat calPoint3D(cv::Mat& depthMat, cv::Mat point2D);
	//根据外接矩形计算快递的旋转矩阵
	cv::Mat calRotationMatrix(cv::Mat& depthMat, cv::Point2f* R, double scale);
	//根据
	/*不考虑z轴*/
	double calAngle(cv::Point2f* R, int h, int w, cv::uint16_t depth);
};
