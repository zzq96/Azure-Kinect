#include <vector>
#include <stdio.h>
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
#include"Function_SHUJING.h"

k4a::KinectAPI kinect;
cv::Mat depthCameraMatrix, depthDisCoeffs;
Object ObjectRes[10];
int main()
{
	for (int i = 0; i < 50; i++)
	{
		cv::Mat img_depth, colorMat, img_depthcolor;
		kinect.GetOpenCVImage(colorMat, img_depth, img_depthcolor);
		//colorMat.at<INT32>(640, 360) = 0;
		//colorMat.at<INT32>(641, 360) = 0;
		//colorMat.at<INT32>(639, 360) = 0;
		//colorMat.at<INT32>(640, 361) = 0;
		//colorMat.at<INT32>(640, 359) = 0;
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
		kinect.ShowOpenCVImage(colorMat, "color");
		kinect.ShowOpenCVImage(img_depthcolor, "depthcolor");
		kinect.GetIntrinsicParam(depthCameraMatrix, depthDisCoeffs, "depth");
		cout << "depth intriParam:";
		for (int i = 0; i < 5; i++)
			cout << depthDisCoeffs.at<float>(i) << " ";
		cout << endl;
		int iDistance = 1290;
		int iObj_num = ObjectLocation(depthCameraMatrix, (UINT16*)img_depth.data, iDistance, img_depth.cols, img_depth.rows,10, 710, ObjectRes);
		cout << "iObj_num:"<<iObj_num << endl;
		for (int i = 0; i < iObj_num; i++)
		{
			Draw_Convex(img_depthcolor, img_depthcolor.cols, img_depthcolor.rows, ObjectRes[i].R);
		}
		kinect.ShowOpenCVImage(img_depthcolor, "depthcolor");
	}
	
	//for (int i = 1; i <= 6; i++)
	//{
	//	std::string name = "imgs/img" + to_string(i);
	//	cout << name << endl;
	//	cv::Mat img_depth = cv::imread(name+"_depth.png",cv::IMREAD_ANYDEPTH);
	//	cv::Mat img_color = cv::imread(name+"_color.png");
	//	cv::Mat img_depthcolor = cv::imread(name+"_depthcolor.png");
	//	int iDistance = 1380;

	//	//测试GetXYZAtCameraView函数
	//	//cv::Point2i point2D(360, 640);
	//	for(int x = 0; x<720;x++)
	//		for (int y = 0; y < 1280; y++)
	//		{
	//			cv::Point2i point2D(x, y);
	//			cv::Point3f point3D;
	//			//kinect.GetXYZAtCameraView(point2D, img_depth.at<UINT16>(point2D.x, point2D.y), point3D);
	//			kinect.GetXYZAtCameraView(point2D, 1000, point3D);
	//			if (abs(point3D.x) < 5 && abs(point3D.y) < 5)
	//			{
	//				cout << x << " " << y << endl;
	//				cout << "x:" << point3D.x << " " << "y:" << point3D.y << " " << "z:" << point3D.z << endl;
	//			}
	//		}
	//	
	//	int iObj_num = ObjectLocation(depthCameraMatrix, (UINT16*)img_depth.data, iDistance, img_depth.cols, img_depth.rows,10, 710, ObjectRes);
	//	cout << "iObj_num:"<<iObj_num << endl;
	//	for (int i = 0; i < iObj_num; i++)
	//	{
	//		Draw_Convex(img_depthcolor, img_depthcolor.cols, img_depthcolor.rows, ObjectRes[i].R);
	//	}
	//	kinect.ShowOpenCVImage(img_depthcolor, "depthcolor");
	//}
	return 0;
}


int main1()
{
	
	try {
		//pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud < pcl::PointXYZRGB>());
		k4a::KinectAPI kinect;
		//opencv图像
		int img_cnt = 50;
		for(int i = 0; i < img_cnt; i++)
		{
			printf("第%d张图片\n", i);
			system("pause");
			cv::Mat depthMat, colorMat, depthcolorMat;
			kinect.GetOpenCVImage(colorMat, depthMat, depthcolorMat);

			for(int x = 400; x <410; x ++)
				for (int y = 600; y <610;y++)
					std::cout << (depthMat.at<UINT16>(x, y))<<" ";
			cout << endl;
			cout << "finished ori" << endl;

			kinect.ShowOpenCVImage(colorMat, "color");
			kinect.ShowOpenCVImage(depthcolorMat, "depthcolor");
			std::string name = "imgs/img" + std::to_string(i);
		//	cv::imwrite(name+"_depth.png", depthMat);
		//	cv::imwrite(name+"_depthcolor.png", depthcolorMat);
		//	cv::imwrite(name+"_color.png", colorMat);

			cv::Mat img = cv::imread(name+"_depth.png",cv::IMREAD_ANYDEPTH);
			cout << "after read:" << endl;
			for(int x = 400; x <410; x ++)
				for (int y = 600; y <610;y++)
					std::cout << (img.at<UINT16>(x, y)) << " ";
			cout << endl;
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
