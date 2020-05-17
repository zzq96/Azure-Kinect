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

int main()
{
	k4a::KinectAPI kinect;
	//cv::Mat depthMat, colorMat, depthcolorMat;
	//kinect.GetOpenCVImage(colorMat, depthMat, depthcolorMat);
	//kinect.ShowOpenCVImage(colorMat, "color");
	//kinect.ShowOpenCVImage(depthcolorMat, "depthcolor");
	cv::Mat intriParam;
	kinect.GetIntrinsicParam(intriParam);
	
	//std::string name = "imgs/img0";
	//cv::Mat img_depth = cv::imread(name+"_depth.png",cv::IMREAD_ANYDEPTH);
	//cv::Mat img_color = cv::imread(name+"_color.png");
	//kinect.ShowOpenCVImage(img_color, name);
	//cv::imshow(name, img_color);
	//cv::waitKey(0);

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
