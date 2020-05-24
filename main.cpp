#include <vector>
#include <stdio.h>
#include<string>
#include <stdlib.h>
#include <iostream>
#include <k4a/k4a.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/visualization/cloud_viewer.h>
#include<pcl/filters/passthrough.h>
#include <pcl/common/common.h>
#include "k4a_grabber.h"
#include<ctime>
#include <fstream>
#include"Function_SHUJING.h"

cv::Mat depthCameraMatrix, depthDisCoeffs;
Object ObjectRes[10];

void HomogeneousMtr2RT(cv::Mat& HomoMtr, cv::Mat& R, cv::Mat& T)
{
	//Mat R_HomoMtr = HomoMtr(Rect(0, 0, 3, 3)); //注意Rect取值
	//Mat T_HomoMtr = HomoMtr(Rect(3, 0, 1, 3));
	//R_HomoMtr.copyTo(R);
	//T_HomoMtr.copyTo(T);
	/*HomoMtr(Rect(0, 0, 3, 3)).copyTo(R);
	HomoMtr(Rect(3, 0, 1, 3)).copyTo(T);*/
	cv::Rect R_rect(0, 0, 3, 3);
	cv::Rect T_rect(3, 0, 1, 3);
	R = HomoMtr(R_rect);
	T = HomoMtr(T_rect);

}
//把中心点涂黑，方便找到桌面上的（0,0）点
void DrawCenterPoints(cv::Mat& colorMat);
//测试GetXYZAtCameraView函数
void TestGetXYZAtCameraView();
k4a::KinectAPI kinect;
int main1()
{
	
	try {
		//pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud < pcl::PointXYZRGB>());
		//opencv图像
		int img_cnt = 100;
		int real_cnt = 0;
		for(int i = 0; i < img_cnt; i++)
		{
			printf("第%d张图片\n", real_cnt);
			cv::Mat depthMat, colorMat, depthcolorMat ,irMat, ircolorMat;
			kinect.GetOpenCVImage(colorMat, depthMat, depthcolorMat, irMat, FALSE);
			//cv::Mat irMat_ = irMat.clone();

			//for(int x = 0; x < irMat.rows; x ++)
			//	for (int y = 0; y < irMat.cols; y++)
			//	{
			//		irMat_.at<UINT16>(x, y) /= 5;
			//		cout << irMat_.at<UINT16>(x, y) << endl;
			//	}

			//cout << endl;
			//cout << "finished ori" << endl;

			kinect.ShowOpenCVImage(colorMat, "color");
			//kinect.ShowOpenCVImage(depthcolorMat, "depthcolor");
			//kinect.ShowOpenCVImage(irMat_, "ir");
			while (1)
			{
				printf("是否保存图片？输入y或者n\n");
				string key;
				getline(cin, key);

				if (key == "y") {}
				else if (key == "n") 
				{ 
					printf("图片未保存，按任意键继续拍照\n");
					system("pause");
					break; 
				}
				else { continue; }
				std::string name;
				if(real_cnt < 10)
					name = "imgs/img0" + std::to_string(real_cnt);
				else 
					name = "imgs/img" + std::to_string(real_cnt);
				cv::imwrite(name + "_depth.png", depthMat);
				cv::imwrite(name + "_depthcolor.png", depthcolorMat);
				cv::imwrite(name + "_color.png", colorMat);
				cv::imwrite(name + "_ir.png", irMat);
				real_cnt++;
				printf("第%d张图片保存成功，按任意键继续拍照\n", real_cnt);
				system("pause");
				break;
			}

			//cv::Mat img = cv::imread(name+"_depth.png",cv::IMREAD_ANYDEPTH);
			//cout << "after read:" << endl;
			//for(int x = 400; x <410; x ++)
			//	for (int y = 600; y <610;y++)
			//		std::cout << (img.at<UINT16>(x, y)) << " ";
			//cout << endl;
		}

	//	//转化为点云
	//	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
	//	kinect.GetPointCloud(cloud);
	//	cout << cloud->width << " " << cloud->height << endl;

	//	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filter(new pcl::PointCloud<pcl::PointXYZRGBA>());
	//	kinect.GetPointCloud(cloud);

	//	pcl::PointXYZRGBA minpt, maxpt;
	//	pcl::getMinMax3D(*cloud, minpt, maxpt);
	//	cout << minpt.x << " " << minpt.y << " " << minpt.z << endl;
	//	cout << maxpt.x << " " << maxpt.y << " " << maxpt.z << endl;

	//	pcl::PassThrough<pcl::PointXYZRGBA> pass;
	//	pass.setInputCloud(cloud);
	//	pass.setFilterFieldName("z");          //设置过滤时所需要点云类型的Z字段
	//	pass.setFilterLimits(0.2, 10);         //设置在过滤字段的范围
	//	pass.setFilterLimitsNegative(false);   //保留还是过滤掉范围内的点
	//	pass.filter(*cloud_filter);


	//	pcl::visualization::CloudViewer view("PCL");
	//	view.showCloud(cloud_filter);
	//	cout << cloud_filter->width << " " << cloud_filter->height << endl;
	//	std::system("pause");
		kinect.ReleaseDevice();
	}
	catch(const char * msg)
	{
		printf("%s\n", msg);
		return 1;
	}
    return 0;
}
int main()
{
	string caliberation_camera_file = "caliberation_camera.xml";
	string Homo_cam2base_file = "Homo_cam2base.xml";
	cv::FileStorage fs(caliberation_camera_file, cv::FileStorage::READ); //读取标定XML文件  
	cv::Mat depthCameraMatrix, depthDistCoeffs;
	fs["cameraMatrix"] >> depthCameraMatrix;
	fs["distCoeffs"]>> depthDistCoeffs;
	fs.release();
	cv::FileStorage fs2(Homo_cam2base_file, cv::FileStorage::READ); //读取相机与基座的转化关系XML文件  
	cv::Mat Homo_cam2base;
	fs2["Homo_cam2base"] >> Homo_cam2base;
	Homo_cam2base = Homo_cam2base.inv();
	fs2.release();
	/*讲单应矩阵转化为旋转矩阵和平移向量方便接下来运算*/
	cv::Mat R_cam2base, t_cam2base;
	HomogeneousMtr2RT(Homo_cam2base, R_cam2base, t_cam2base);


	for (int i = 0; i < 50; i++)
	{
		cv::Mat depthMatOld, depthMat, colorMat, depthcolorMat,  depthcolorMatOld, irMat, ircolorMat;
		kinect.GetOpenCVImage(colorMat, depthMatOld, depthcolorMatOld, irMat, FALSE);
		kinect.ShowOpenCVImage(depthcolorMatOld, "old");

		/*根据内参和畸变系数校正图像*/
		//cv::Size image_size;
		//cv::Mat mapx = cv::Mat(image_size, CV_32FC1);
		//cv::Mat mapy = cv::Mat(image_size, CV_32FC1);
		//cv::Mat R = cv::Mat::eye(3, 3, CV_32F);
		//cout << "保存矫正图像" << endl;
		//initUndistortRectifyMap(depthCameraMatrix, depthDisCoeffs, R, depthCameraMatrix, image_size, CV_32FC1, mapx, mapy);
		undistort(depthMatOld,depthMat,depthCameraMatrix,depthDistCoeffs);  
		undistort(depthcolorMatOld,depthcolorMat,depthCameraMatrix,depthDistCoeffs);  
		kinect.ShowOpenCVImage(depthcolorMat, "new");


		//把机械臂部分深度设为0，这样就不会检测到机械臂了
		for (int h = 0; h < 250; h++)
			for (int w = 0; w < 300; w++)
			{
				depthMat.at<UINT16>(h, w) = 0;
				depthcolorMat.at<cv::Vec4b>(h, w)[0] = 0;
				depthcolorMat.at<cv::Vec4b>(h, w)[1] = 0;
				depthcolorMat.at<cv::Vec4b>(h, w)[2] = 0;
			}
		
		kinect.ShowOpenCVImage(colorMat, "img_color");
		//
		float robot_x = 336.931, robot_y = 394.312, robot_z = 114.206, robot_len = 215;
		kinect.GetIntrinsicParam(depthCameraMatrix, depthDisCoeffs, "depth");
		int iDistance = 1200;
		int iObj_num = ObjectLocation(depthCameraMatrix, (UINT16*)depthMat.data, iDistance, depthMat.cols, depthMat.rows,0, depthMat.rows, ObjectRes);
		cout << "iObj_num:"<<iObj_num << endl;
		for (int i = 0; i < iObj_num; i++)
		{
			Draw_Convex(depthcolorMat, depthcolorMat.cols, depthcolorMat.rows, ObjectRes[i].R);
			float x=0, y=0;
			for (int j = 0; j < 4; j++)
			{
				x += ObjectRes[i].R[j].x;
				y += ObjectRes[i].R[j].y;
			}
			x /= 4, y /= 4;
			/*齐次坐标*/
			cv::Mat point2D(3, 1, CV_64F, cv::Scalar(0));
			point2D.at<double>(0, 0) = x;
			point2D.at<double>(1, 0) = y;
			point2D.at<double>(2, 0) = 1;
			/*平均周围深度，减少误差*/
			double Zc = (depthMat.at<UINT16>(y, x) + depthMat.at<UINT16>(y + 5, x + 5)
				+ depthMat.at<UINT16>(y - 5, x - 5)
				+ depthMat.at<UINT16>(y + 5, x - 5)
				+ depthMat.at<UINT16>(y - 5, x + 5)) / 5;

			//cout << R_cam2base.type() << depthCameraMatrix.type() << point2D.type() << t_cam2base.type() << endl;
			depthCameraMatrix.convertTo(depthCameraMatrix, CV_64F);

			cv::Mat point3D = R_cam2base.t() * (depthCameraMatrix.inv() * Zc * point2D - t_cam2base);
			cout << R_cam2base.t() << endl;
			cout << depthCameraMatrix.inv() << endl;
			cout << point2D << endl;
			cout << t_cam2base << endl;
			cout << depthCameraMatrix.inv() * Zc << endl;
			cout << depthCameraMatrix.inv() * Zc * point2D << endl;
			cout << (depthCameraMatrix.inv() * Zc * point2D - t_cam2base) << endl;
			cout << "坐标为:" << endl;
			cout << point3D << endl;

			//kinect.GetXYZAtCameraView(point2D, img_depth.at<UINT16>(point2D.x, point2D.y), point3D);

			////机械臂基座坐标系和相机坐标系xy轴是对调的。
			//swap(point3D.x, point3D.y);
			//point3D.z = 1310 - point3D.z;

			//cout << "pixel:"<<x << " " << y << endl;
			//point3D.x += robot_x;
			//point3D.y += robot_y;
			//point3D.z += -robot_z + robot_len;
			//cout <<"real_robot:"<< "x:" << point3D.x << " " << "y:" << point3D.y << " " << "z:" << point3D.z << endl;

		}
		kinect.ShowOpenCVImage(depthcolorMat, "depthcolor");
	}
	
return 0;
}

void TestGetXYZAtCameraView()
{	
	for (int i = 1; i <= 6; i++)
	{
		std::string name = "imgs/img" + to_string(i);
		cout << name << endl;
		cv::Mat img_depth = cv::imread(name+"_depth.png",cv::IMREAD_ANYDEPTH);
		cv::Mat img_color = cv::imread(name+"_color.png");
		cv::Mat img_depthcolor = cv::imread(name+"_depthcolor.png");
		int iDistance = 1380;

		//测试GetXYZAtCameraView函数
		//cv::Point2i point2D(360, 640);
		for(int x = 0; x<720;x++)
			for (int y = 0; y < 1280; y++)
			{
				cv::Point2i point2D(x, y);
				cv::Point3f point3D;
				//kinect.GetXYZAtCameraView(point2D, img_depth.at<UINT16>(point2D.x, point2D.y), point3D);
				kinect.GetXYZAtCameraView(point2D, 1000, point3D);
				if (abs(point3D.x) < 5 && abs(point3D.y) < 5)
				{
					cout << x << " " << y << endl;
					cout << "x:" << point3D.x << " " << "y:" << point3D.y << " " << "z:" << point3D.z << endl;
				}
			}
		
		int iObj_num = ObjectLocation(depthCameraMatrix, (UINT16*)img_depth.data, iDistance, img_depth.cols, img_depth.rows,10, 710, ObjectRes);
		cout << "iObj_num:"<<iObj_num << endl;
		for (int i = 0; i < iObj_num; i++)
		{
			Draw_Convex(img_depthcolor, img_depthcolor.cols, img_depthcolor.rows, ObjectRes[i].R);
		}
		kinect.ShowOpenCVImage(img_depthcolor, "depthcolor");
	}
}
void DrawCenterPoints(cv::Mat& colorMat)
{
	colorMat.at<cv::Vec4b>(360, 640)[0] = 0;
	colorMat.at<cv::Vec4b>(360, 640)[1] = 0;
	colorMat.at<cv::Vec4b>(360, 640)[2] = 0;
	colorMat.at<cv::Vec4b>(360, 641)[0] = 0;
	colorMat.at<cv::Vec4b>(360, 641)[1] = 0;
	colorMat.at<cv::Vec4b>(360, 641)[2] = 0;
	colorMat.at<cv::Vec4b>(360, 639)[0] = 0;
	colorMat.at<cv::Vec4b>(360, 639)[1] = 0;
	colorMat.at<cv::Vec4b>(360, 639)[2] = 0;
	colorMat.at<cv::Vec4b>(361, 640)[3]= 0;
	colorMat.at<cv::Vec4b>(359, 640)[0] = 0;
	colorMat.at<cv::Vec4b>(359, 640)[1] = 0;
	colorMat.at<cv::Vec4b>(359, 640)[2] = 0;
}

