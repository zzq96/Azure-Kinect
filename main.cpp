#include <winsock2.h>
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
#include"SocketRobot.h"

const double PI = 3.1415926;
cv::Mat depthMatOld, depthMat, colorMat, depthcolorMat,  depthcolorMatOld, irMat, ircolorMat;
//深度值经过外参校正后的深度图像
cv::Mat depthMatRevise;
Object ObjectRes[10];
cv::Mat depthCameraMatrix, depthDistCoeffs;
cv::Mat R_cam2base, t_cam2base;
bool shang = TRUE;
// Checks if a matrix is a valid rotation matrix.
bool isRotationMatrix(cv::Mat &R)
{
    cv::Mat Rt;
    transpose(R, Rt);
    cv::Mat shouldBeIdentity = Rt * R;
    cv::Mat I = cv::Mat::eye(3,3, shouldBeIdentity.type());
    return  norm(I, shouldBeIdentity) < 1e-6;
     
}
 
// Calculates rotation matrix to euler angles
// The result is the same as MATLAB except the order
// of the euler angles ( x and z are swapped ).
cv::Vec3f rotationMatrixToEulerAngles(cv::Mat &R)
{
    assert(isRotationMatrix(R));

    float sy = sqrt(R.at<float>(0,0) * R.at<float>(0,0) +  R.at<float>(1,0) * R.at<float>(1,0) );

    bool singular = sy < 1e-6; // If

    float x, y, z;
    if (!singular) {
        x = atan2(R.at<float>(2,1) , R.at<float>(2,2));
        y = atan2(-R.at<float>(2,0), sy);
        z = atan2(R.at<float>(1,0), R.at<float>(0,0));
    } else {
        x = atan2(-R.at<float>(1,2), R.at<float>(1,1));
        y = atan2(-R.at<float>(2,0), sy);
        z = 0;
    }
    return cv::Vec3f(z, y, x) / CV_PI * 180;   
}
UINT16 getDepth(cv::Mat point2D)
{
	int x = (int)point2D.at<float>(0, 0);
	int y = (int)point2D.at<float>(1, 0);
	//TODO:加入边界判断
	if (x < 0) x = 1;
	if (y < 0) y = 1;
	UINT16 Zc = (depthMat.at<UINT16>(y, x)
		+depthMat.at<UINT16>(y+1, x)
		+depthMat.at<UINT16>(y-1, x)
		+depthMat.at<UINT16>(y, x-1)
		+depthMat.at<UINT16>(y, x+1)
		)/5.;
	return Zc;
}
float getLength(cv::Mat a)
{
	float dx = a.at<float>(0, 0);
	float dy = a.at<float>(1, 0);
	float d = sqrt(dx * dx + dy * dy);
	return d;
}
float getDistance(cv::Point3f a, cv::Point3f b)
{
	float  d = sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y)); 
	return d;
}
bool compare(Object a, Object b)
{
	double ay = (a.R[0].y + a.R[1].y + a.R[2].y + a.R[3].y)/4;
	double by = (b.R[0].y + b.R[1].y + b.R[2].y + b.R[3].y)/4;
	if (shang == TRUE)
	{
		//double ay = -1000000, by = -100000;
		//ay = min(a.R[0].y, min(a.R[1].y, min(a.R[2].y, a.R[3].y)));
		//by = min(b.R[0].y, min(b.R[1].y, min(b.R[2].y, b.R[3].y)));

		if (ay > 400)
			return FALSE;
		if (by > 400)
			return TRUE;
		return ay < by;
	}
	else
	{
		double ax = -1000000, bx = -100000;
		ax = max(a.R[0].x, max(a.R[1].x, max(a.R[2].x, a.R[3].x)));
		bx = max(b.R[0].x, max(b.R[1].x, max(b.R[2].x, b.R[3].x)));

		if (ay < 400)
			return FALSE;
		if (by < 400)
			return TRUE;
		return ax < bx;
	}
}
//TODO:没测试过
//以高度从上到下排序
bool compareByHeight(Object a, Object b)
{
	float ay = (a.R[0].y + a.R[1].y + a.R[2].y + a.R[3].y)/4;
	float ax = (a.R[0].x + a.R[1].x + a.R[2].x + a.R[3].x)/4;
	float by = (b.R[0].y + b.R[1].y + b.R[2].y + b.R[3].y)/4;
	float bx = (b.R[0].x + b.R[1].x + b.R[2].x + b.R[3].x)/4;
	cv::Mat aM =(cv::Mat_<float>(3, 1)<<ax, ay, 1);
	cv::Mat bM =(cv::Mat_<float>(3, 1)<<bx, by, 1);

	UINT16 aDepth = getDepth(aM);
	UINT16 bDepth = getDepth(bM);
	//根据外参计算到机械臂坐标系
	cv::Mat a3D = R_cam2base.t() * (depthCameraMatrix.inv() * aDepth * aM - t_cam2base);
	cv::Mat b3D = R_cam2base.t() * (depthCameraMatrix.inv() * bDepth * bM - t_cam2base);
	return a3D.at<float>(2, 0) > b3D.at<float>(2, 0);
}
bool compareByArea(Object a, Object b)
{
	double aArea = abs((a.R[0] - a.R[1]) ^ (a.R[1] - a.R[2]));
	double bArea = abs((b.R[0] - b.R[1]) ^ (b.R[1] - b.R[2]));
	return aArea > bArea;
}
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
//计算周围5个点的平均值
void calPoint3D(cv::Mat point2D, cv::Point3f & real, UINT16 Zc)
{
	assert(Zc != 0);
	cv::Mat point3D = R_cam2base.t() * (depthCameraMatrix.inv() * Zc * point2D - t_cam2base);
	real.x = point3D.at<float>(0, 0);
	real.y = point3D.at<float>(1, 0);
	real.z = point3D.at<float>(2, 0);
}

cv::Mat calRotationMatrix(const vector<Point> &R, float scale)
{
	cv::Mat a = (cv::Mat_<float>(3, 1) << R[0].x, R[0].y, 1);
	cv::Mat b = (cv::Mat_<float>(3, 1) << R[1].x, R[1].y, 1);
	cv::Mat c = (cv::Mat_<float>(3, 1) << R[2].x, R[2].y, 1);
	cv::Mat d = (cv::Mat_<float>(3, 1) << R[3].x, R[3].y, 1);
	cout << "abcd" << a << b << c << d << endl;
	cv::Mat center = (a + b + c + d) / 4;
	cout << "center" << center << endl;

	cv::Mat aa = center + (a - center)  * scale;
	cv::Mat bb = center + (b - center)  * scale;
	cv::Mat cc = center + (c - center)  * scale;
	cv::Mat dd = center + (d - center)  * scale;
	cout << aa << bb << cc << dd << endl;
	
	cv::Point3f raa, rbb, rcc, rdd, rcenter;
	calPoint3D(center, rcenter, getDepth(center));
	calPoint3D(aa, raa, getDepth(aa));
	calPoint3D(bb, rbb, getDepth(bb));
	calPoint3D(cc, rcc, getDepth(cc));
	calPoint3D(dd, rdd, getDepth(dd));
	//计算4条边的向量
	cv::Point3f ab = raa - rbb;
	cv::Point3f dc = rdd - rcc;
	cv::Point3f ad = raa - rdd;
	cv::Point3f bc = rbb - rcc;
	//将边的方向当成x轴和y轴平均一下
	cv::Point3f v1 = (ab + dc) / 2;
	cv::Point3f v2 = (ad + bc) / 2;
	//把较长的边当做x轴，因为2个吸盘是x轴方向的
	cv::Point3f X, Y;
	if (v1.dot(v1) > v2.dot(v2))
	{
		X = v1 / sqrt(v1.dot(v1));//单位化
		Y = v2 / sqrt(v2.dot(v2));
	}
	else 
	{
		X = v2 / sqrt(v2.dot(v2));//单位化
		Y = v1 / sqrt(v1.dot(v1));
	}

	if (X.x < 0)
		X *= -1;

	//算两个法向量，然后平均一下，这样
	cv::Point3f normal1 = (raa - rbb).cross(raa - rdd);
	cv::Point3f normal2 = (rcc - rbb).cross(rcc - rdd);
	cv::Point3f normal3 = (rbb - raa).cross(rbb - rcc);
	cv::Point3f normal4 = (rdd - raa).cross(rdd - rcc);
	normal1 = normal1 / sqrt(normal1.dot(normal1));
	normal2 = normal2 / sqrt(normal2.dot(normal2));
	normal3 = normal3 / sqrt(normal3.dot(normal3));
	normal4 = normal4 / sqrt(normal4.dot(normal4));

	if (normal1.z < 0)
		normal1 *= -1;
	if (normal2.z < 0)
		normal2 *= -1;
	if (normal3.z < 0)
		normal3 *= -1;
	if (normal4.z < 0)
		normal4 *= -1;

	cv::Point3f normal = (normal1 + normal2 + normal3 + normal4) / 4;
	normal = normal / sqrt(normal.dot(normal));

	//已知x轴和z轴，根据叉积求出y轴,和之前得到y轴统一方向
	if (normal.cross(X).x * Y.x < 0)
		Y *= -1;
	cout << "范数" << endl;
	cout << X.dot(X) << " " << Y.dot(Y) << " " << normal.dot(normal) << endl;;

	cv::Mat rotation = cv::Mat_<float>(3, 3);
	rotation.at<float>(0, 0) = X.x;
	rotation.at<float>(1, 0) = X.y;
	rotation.at<float>(2, 0) = X.z;
	rotation.at<float>(0, 1) = Y.x;
	rotation.at<float>(1, 1) = Y.y;
	rotation.at<float>(2, 1) = Y.z;
	rotation.at<float>(0, 2) = normal.x;
	rotation.at<float>(1, 2) = normal.y;
	rotation.at<float>(2, 2) = normal.z;
	
	//我们之前求出的3个轴不一定互相正交，用下面这张奇异值分解的时候求出一个近似正交的矩阵
	cv::Mat w, u, vt;
	cv::SVDecomp(rotation, w, u, vt);
    cv::Mat I = cv::Mat::eye(3,3, rotation.type());
	cv::Mat new_rotation = u * I * vt;

	return new_rotation;
}
/*不考虑z轴*/
float calAngle(const vector<Point>& R, int h, int w, UINT16 depth)
{
	cv::Mat point2D(3, 1, CV_32F, cv::Scalar(0));
	point2D.at<float>(2, 0) = 1;
	cv::Mat point3D(3, 1, CV_32F, cv::Scalar(0));
	cv::Point3f a, b, c;//长方形三个顶点的真实坐标
	/*如果某一个点超出边界，则顺移点。毛想想应该最多只有一个点超出边界*/
	int k;
	for (int i = 0; i < R.size(); i++)
	{
		k = i;
		if (R[k].x < 0 || R[k].x >= w || R[k].y < 0 || R[k].y >= h)
			continue;
		point2D.at<float>(0, 0) = R[k].x;
		point2D.at<float>(1, 0) = R[k].y;
		calPoint3D(point2D, a, depth);

		k = (i + 1) % R.size();
		if (R[k].x < 0 || R[k].x >= w || R[k].y < 0 || R[k].y >= h)
			continue;
		point2D.at<float>(0, 0) = R[k].x;
		point2D.at<float>(1, 0) = R[k].y;
		calPoint3D(point2D, b, depth);

		k = (i + 2) % R.size();
		if (R[k].x < 0 || R[k].x >= w || R[k].y < 0 || R[k].y >= h)
			continue;
		point2D.at<float>(0, 0) = R[k].x;
		point2D.at<float>(1, 0) = R[k].y;
		calPoint3D(point2D, c, depth);
		break;
	}
	float angle;
	cout << "边长1" << getDistance(a, b) << ":边长2" << getDistance(b, c) << endl;
	cout << endl;
	if(getDistance(a, b) > getDistance(b, c))
	{
		if (abs(a.x - b.x) < 30)
		{
			angle = 90;
		}
		else 
		{
			float k = (a.y - b.y) / (a.x - b.x);
			angle = (tanh(k) / PI * 180);
		}
	}
	else
	{
		if (abs(b.x - c.x) < 30)
		{
			angle = 90;
		}
		else 
		{
			float k = (b.y - c.y) / (b.x - c.x);
			angle = (tanh(k) / PI * 180);
		}
	}
	cout << "oldangle" << endl;
	cout << angle << endl;
	if (angle > 90)
		angle -= 180;
	if (angle < -90)
		angle += 180;
	return angle;
}

//TODO:删除那些重合的物体
void DeleteBadObejct(Object *ObjectRes, int &iObj_num) 
{
	//按面积从大到小排序
	sort(ObjectRes, ObjectRes + iObj_num, compareByArea);
	for (int i = 0; i < iObj_num; i++)
	{
		for (int j = i + 1; j < iObj_num; j++)
		{
			//找到j的中心点
			float y = (ObjectRes[j].R[0].y + ObjectRes[j].R[1].y + ObjectRes[j].R[2].y + ObjectRes[j].R[3].y)/4;
			float x = (ObjectRes[j].R[0].x + ObjectRes[j].R[1].x + ObjectRes[j].R[2].x + ObjectRes[j].R[3].x)/4;

			vector<cv::Point> contours(4);
			contours[0].x = ObjectRes[i].R[0].x;
			contours[0].y = ObjectRes[i].R[0].y;
			contours[1].x = ObjectRes[i].R[1].x;
			contours[1].y = ObjectRes[i].R[1].y;
			contours[2].x = ObjectRes[i].R[2].x;
			contours[2].y = ObjectRes[i].R[2].y;
			contours[3].x = ObjectRes[i].R[3].x;
			contours[3].y = ObjectRes[i].R[3].y;

			float isIn = cv::pointPolygonTest(contours, cv::Point2f(x, y), true);

			//
			if (isIn > 0)
			{
				for (int k = j; k < iObj_num - 1; k++)
					swap(ObjectRes[k], ObjectRes[k+1]);
				iObj_num--;
				cout << "删除一个物体，xy为：" << x << " " << y << endl;
			}
		}
	}
}
//把中心点涂黑，方便找到桌面上的（0,0）点
void DrawCenterPoints(cv::Mat& colorMat);
//测试GetXYZAtCameraView函数
void TestGetXYZAtCameraView();
k4a::KinectAPI kinect;
int main1()
{
	
	cv::Mat cameraMatrix, disCoeffs;
	kinect.GetIntrinsicParam(cameraMatrix,disCoeffs, "color");
	cout << cameraMatrix << endl;
	try {
		//pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud < pcl::PointXYZRGB>());
		//opencv图像
		int img_cnt = 100;
		int real_cnt = 0;
		for(int i = 0; i < img_cnt; i++)
		{
			printf("第%d张图片\n", real_cnt);
			cv::Mat depthMat, colorMat, depthcolorMat ,irMat, ircolorMat;
			kinect.GetOpenCVImage(colorMat, depthMat, depthcolorMat, irMat, TRUE);
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
			kinect.ShowOpenCVImage(depthcolorMat, "depthcolor");
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
	bool useRobot = FALSE;
	
	string caliberation_camera_file = "caliberation_camera.xml";
	string Homo_cam2base_file = "Homo_cam2base.xml";
	cv::FileStorage fs(caliberation_camera_file, cv::FileStorage::READ); //读取标定XML文件  
	fs["cameraMatrix"] >> depthCameraMatrix;
	depthCameraMatrix.convertTo(depthCameraMatrix, CV_32F);
	cout << depthCameraMatrix.type() << endl;
	fs["distCoeffs"] >> depthDistCoeffs;
	cout << "depthCameraMatrix" << depthCameraMatrix << endl;
	cout << "disCoeffs" << depthDistCoeffs << endl;
	fs.release();
	cv::FileStorage fs2(Homo_cam2base_file, cv::FileStorage::READ); //读取相机与基座的转化关系XML文件  
	cv::Mat Homo_cam2base;
	fs2["Homo_cam2base"] >> Homo_cam2base;
	cout << "Homo_cam2base" << Homo_cam2base << endl;
	/*为什么要求逆？*/
	Homo_cam2base = Homo_cam2base.inv();
	fs2.release();
	/*将单应矩阵转化为旋转矩阵和平移向量方便接下来运算*/
	HomogeneousMtr2RT(Homo_cam2base, R_cam2base, t_cam2base);

	SocketRobot* sr = NULL;
	if (useRobot)
		sr = new SocketRobot();
	
	//cout << "按任意键开始" << endl;
	//system("pause");
	//Sleep(2000);


	int iDistance = 1280;//相机到桌面的距离
	cv::Mat point2D(3, 1, CV_32F, cv::Scalar(0)); 
	point2D.at<float>(2, 0) = 1;
	cv::Mat point3D;
	cv::Mat depthCameraMatrixInv = depthCameraMatrix.inv();

	int minh = 0, maxh = 400;
	int minw = 200, maxw = 550;
	cv::Mat point2DSet(3, (maxh - minh) * (maxw - minw), CV_32F, cv::Scalar(0));
	cv::Mat pointDepthSet(3, (maxh - minh) * (maxw - minw), CV_32F, cv::Scalar(0));
	cv::Mat t_cam2baseSet(3, (maxh - minh) * (maxw - minw), CV_32F, cv::Scalar(0));
	int tmp = 0;
	//576,640是深度图的行数和列数
	for (int h = 0; h < 576; h++)
		for (int w = 0; w < 640; w++)
			if (h >= minh && h < maxh && w >= minw && w < maxw) {
				point2DSet.at<float>(0, tmp) = w;
				point2DSet.at<float>(1, tmp) = h;
				point2DSet.at<float>(2, tmp) = 1;
				t_cam2baseSet.at<float>(0, tmp) = t_cam2base.at<float>(0, 0);
				t_cam2baseSet.at<float>(1, tmp) = t_cam2base.at<float>(1, 0);
				t_cam2baseSet.at<float>(2, tmp) = t_cam2base.at<float>(2, 0);
				tmp++;
			}
	cout << "tmp" << tmp << endl;
	cv::Mat depthAndpointSet = depthCameraMatrixInv * point2DSet;
	int cnt = 0;
	while(1)
	{ 
		{
			kinect.GetOpenCVImage(colorMat, depthMatOld, depthcolorMatOld, irMat, FALSE);
			//kinect.ShowOpenCVImage(depthcolorMatOld, "old");
			/*根据内参和畸变系数校正图像*/
			undistort(depthMatOld,depthMat,depthCameraMatrix,depthDistCoeffs);  
			undistort(depthcolorMatOld,depthcolorMat,depthCameraMatrix,depthDistCoeffs);  
			depthMatRevise = depthMat.clone();
			int tmp = 0;
			for (int h = 0; h < depthMat.rows; h++)
				for (int w = 0; w < depthMat.cols; w++)
					if (h >= minh && h < maxh && w >= minw && w < maxw) {
						pointDepthSet.at<float>(0, tmp) = depthMatRevise.at<UINT16>(h, w);
						pointDepthSet.at<float>(1, tmp) = depthMatRevise.at<UINT16>(h, w);
						pointDepthSet.at<float>(2, tmp) = depthMatRevise.at<UINT16>(h, w);
						tmp++;
					}
					else
					{

							//把其他无关区域深度设0
							depthMatRevise.at<UINT16>(h, w) = 0;
							//depthcolorMat.at<cv::Vec4b>(h, w)[0] = 0;
							//depthcolorMat.at<cv::Vec4b>(h, w)[1] = 0;
							//depthcolorMat.at<cv::Vec4b>(h, w)[2] = 0;
					}

			//计算每个点经过外参校正后的深度值(机械臂坐标系下)
			cv::Mat point3DSet = R_cam2base.t() * ((depthAndpointSet).mul(pointDepthSet) - t_cam2baseSet);
			tmp = 0;
			for(int h = minh; h < maxh; h++)
				for (int w = minw; w < maxw; w++)
				{
					//point3D = R_cam2base.t() * (depthCameraMatrixInv * depthMatRevise.at<UINT16>(h, w) * point2D - t_cam2base);
					if(depthMatRevise.at<UINT16>(h, w) != 0)
						depthMatRevise.at<UINT16>(h, w) = iDistance - point3DSet.at<float>(2, tmp);
					tmp++;
				}

			//kinect.ShowOpenCVImage(depthMatRevise, "img_color");
			int iObj_num = ObjectLocation(depthCameraMatrix, (UINT16*)depthMatRevise.data, iDistance, depthMatRevise.cols, depthMatRevise.rows,0, depthMatRevise.rows, ObjectRes);
			DeleteBadObejct(ObjectRes, iObj_num);
			//按高度从高到低排序
			sort(ObjectRes, ObjectRes + iObj_num, compareByHeight);
			//TODO:算iou，去掉重叠大的
			//std::sort(ObjectRes, ObjectRes + iObj_num, compare);
			cout << "检测到" << iObj_num << "个物体" << endl;
			if (iObj_num == 0)
			{
				Sleep(1000);
				//if(useRobot)
				//	sr->close();
				//cout << "退出" << endl;
				//break;
			}
			for (int i = 0; i < iObj_num; i++)
			{
				if(i == 0)
					Draw_Polygon(depthcolorMat.data, depthcolorMat.cols, depthcolorMat.rows, 4,  ObjectRes[i].R, 0, 0, 255);
				else
					Draw_Polygon(depthcolorMat.data, depthcolorMat.cols, depthcolorMat.rows, 4,  ObjectRes[i].R, 255, 0, 0);
			}
			kinect.ShowOpenCVImage(depthcolorMat, "depthcolor");
			//cout << "iObj_num:"<<iObj_num << endl;
			for (int i = 0; i < iObj_num; i++)
			{
				if (i > 0) break;
//				if(i == 0)
//					Draw_Convex(depthcolorMat, depthcolorMat.cols, depthcolorMat.rows, ObjectRes[i].R);
				//kinect.ShowOpenCVImage(depthcolorMat, "depthcolor");
				float x=0, y=0;
				for (int j = 0; j < 4; j++)
				{
					x += ObjectRes[i].R[j].x;
					y += ObjectRes[i].R[j].y;
				}
				x /= 4, y /= 4;
				/*齐次坐标*/
				point2D.at<float>(0, 0) = x;
				point2D.at<float>(1, 0) = y;
				point2D.at<float>(2, 0) = 1;
				/*平均周围深度，减少误差*/
				double Zc = (depthMat.at<UINT16>(y, x)
					+depthMat.at<UINT16>(y+5, x)
					+depthMat.at<UINT16>(y-5, x)
					+depthMat.at<UINT16>(y, x-5)
					+depthMat.at<UINT16>(y, x+5)
					)/5;
				float angle = calAngle(ObjectRes[i].R, depthMat.rows, depthMat.cols, Zc);
				//平面法向量
				cv::Mat rotationMatrix = calRotationMatrix(ObjectRes[i].R, 0.6);
				cv::Vec3f eulerAngles = rotationMatrixToEulerAngles(rotationMatrix);
				cout << "物体的旋转矩阵为" << endl;
				cout << rotationMatrix << endl;


				cv::Mat point3D = R_cam2base.t() * (depthCameraMatrix.inv() * Zc * point2D - t_cam2base);
				
				cout << "坐标为:" << endl;
				cout << point3D << endl;
				cout << "物体的欧拉角为" << endl;
				cout << eulerAngles << endl;
				
				float coords[12];
				coords[0] = point3D.at<float>(0, 0);
				coords[1] = point3D.at<float>(1, 0);
				coords[2] = point3D.at<float>(2, 0) - 2;
				coords[3] = eulerAngles[0];
				coords[4] = eulerAngles[1];
				coords[5] = eulerAngles[2];
				coords[6] = 698 + 50;
				coords[7] = 342 - 750 + cnt * 220;
				cnt++;
			
				if(coords[2] > 160)
					coords[8] = 160 - 50;
				else
					coords[8] = coords[2] - 50;
				coords[9] = 0;
				coords[10] = 0;
				coords[11] = 0;

				if(useRobot)
					sr->moveRobot(coords);

			}
			//kinect.ShowOpenCVImage(depthcolorMat, "depthcolor");
		}
	}
return 0;
}
int mainold()
{
	string caliberation_camera_file = "caliberation_camera.xml";
	string Homo_cam2base_file = "Homo_cam2base.xml";
	cv::FileStorage fs(caliberation_camera_file, cv::FileStorage::READ); //读取标定XML文件  
	cv::Mat depthCameraMatrix, depthDistCoeffs;
	fs["cameraMatrix"] >> depthCameraMatrix;
	fs["distCoeffs"]>> depthDistCoeffs;
	cout << "depthCameraMatrix" << depthCameraMatrix << endl;
	cout << "disCoeffs" << depthDistCoeffs << endl;
	fs.release();
	cv::FileStorage fs2(Homo_cam2base_file, cv::FileStorage::READ); //读取相机与基座的转化关系XML文件  
	cv::Mat Homo_cam2base;
	fs2["Homo_cam2base"] >> Homo_cam2base;
	cout << "Homo_cam2base" << Homo_cam2base << endl;
	Homo_cam2base = Homo_cam2base.inv();
	fs2.release();
	/*讲单应矩阵转化为旋转矩阵和平移向量方便接下来运算*/
	cv::Mat R_cam2base, t_cam2base;
	HomogeneousMtr2RT(Homo_cam2base, R_cam2base, t_cam2base);


	while(1)
	{ 
		cout << "shang" << shang << endl;
		for (int i = 0; i < 3; i++)
		{
			cv::Mat depthMatOld, depthMat, colorMat, depthcolorMat,  depthcolorMatOld, irMat, ircolorMat;
			kinect.GetOpenCVImage(colorMat, depthMatOld, depthcolorMatOld, irMat, FALSE);
			//kinect.ShowOpenCVImage(depthcolorMatOld, "old");
			/*根据内参和畸变系数校正图像*/
			//cv::Size image_size;
			//cv::Mat mapx = cv::Mat(image_size, CV_32FC1);
			//cv::Mat mapy = cv::Mat(image_size, CV_32FC1);
			//cv::Mat R = cv::Mat::eye(3, 3, CV_32F);
			//cout << "保存矫正图像" << endl;
			//initUndistortRectifyMap(depthCameraMatrix, depthDisCoeffs, R, depthCameraMatrix, image_size, CV_32FC1, mapx, mapy);
			undistort(depthMatOld,depthMat,depthCameraMatrix,depthDistCoeffs);  
			undistort(depthcolorMatOld,depthcolorMat,depthCameraMatrix,depthDistCoeffs);  
			DrawCenterPoints(depthcolorMat);
			//kinect.ShowOpenCVImage(depthcolorMat, "new");


			//把机械臂部分深度设为0，这样就不会检测到机械臂了
			for (int h = 0; h < 250; h++)
				for (int w = 0; w < 250; w++)
				{
					depthMat.at<UINT16>(h, w) = 0;
					//depthcolorMat.at<cv::Vec4b>(h, w)[0] = 0;
					//depthcolorMat.at<cv::Vec4b>(h, w)[1] = 0;
					//depthcolorMat.at<cv::Vec4b>(h, w)[2] = 0;
				}
			
			//kinect.ShowOpenCVImage(colorMat, "img_color");
			//
			float robot_x = 411.984, robot_y = 348.392, robot_z = 114.206, robot_len = 215;
			kinect.GetIntrinsicParam(depthCameraMatrix, depthDistCoeffs, "depth");
			int iDistance = 1320;
			int iObj_num = ObjectLocation(depthCameraMatrix, (UINT16*)depthMat.data, iDistance, depthMat.cols, depthMat.rows,0, depthMat.rows, ObjectRes);
			std::sort(ObjectRes, ObjectRes + iObj_num, compare);
			cout << "iObj_num:"<<iObj_num << endl;
			for (int i = 0; i < iObj_num; i++)
			{
				if(i == 0)
				Draw_Polygon(depthcolorMat.data, depthcolorMat.cols, depthcolorMat.rows, 4,  ObjectRes[i].R, 0, 0, 255);
				float x=0, y=0;
				for (int j = 0; j < 4; j++)
				{
					x += ObjectRes[i].R[j].x;
					y += ObjectRes[i].R[j].y;
				}
				x /= 4, y /= 4;
				///*齐次坐标*/
				//cv::Mat point2D(3, 1, CV_64F, cv::Scalar(0));
				//point2D.at<double>(0, 0) = x;
				//point2D.at<double>(1, 0) = y;
				//point2D.at<double>(2, 0) = 1;
				///*平均周围深度，减少误差*/
				//double Zc = depthMat.at<UINT16>(y, x);
				////cout << R_cam2base.type() << depth+ depthMat.at<UINT16>(y - 5, x + 5)CameraMatrix.type() << point2D.type() << t_cam2base.type() << endl;
				//depthCameraMatrix.convertTo(depthCameraMatrix, CV_64F);

				//cv::Mat point3D = R_cam2base.t() * (depthCameraMatrix.inv() * Zc * point2D - t_cam2base);
				//cout << R_cam2base.t() << endl;
				//cout << depthCameraMatrix.inv() << endl;
				//cout << point2D << endl;
				//cout << t_cam2base << endl;
				//cout << depthCameraMatrix.inv() * Zc << endl;
				//cout << depthCameraMatrix.inv() * Zc * point2D << endl;
				//cout << (depthCameraMatrix.inv() * Zc * point2D - t_cam2base) << endl;
				//cout << "坐标为:" << endl;
				//cout << point3D << endl;

				//kinect.GetXYZAtCameraView(point2D, img_depth.at<UINT16>(point2D.x, point2D.y), point3D);

				//机械臂基座坐标系和相机坐标系xy轴是对调的。
				cv::Point3f Point3D;
				Point3D.x = x;
				Point3D.y = y;
				Point3D.z = (depthMat.at<UINT16>(y, x)
					+depthMat.at<UINT16>(y+5, x)
					+depthMat.at<UINT16>(y-5, x)
					+depthMat.at<UINT16>(y, x-5)
					+depthMat.at<UINT16>(y, x+5)
					)/5;
				
				Point3D.z = 1320 - Point3D.z;
				Point3D.x -= 320;
				Point3D.y -= 288;
				//Point3D.x /= depthCameraMatrix.at<float>(0, 0) ;
				//Point3D.y /= depthCameraMatrix.at<float>(1, 1) ;

				std::swap(Point3D.x, Point3D.y);
				//cout << "pixel:"<<x << " " << y << endl;
				Point3D.x += robot_x;
				Point3D.y += robot_y;
				Point3D.z += -robot_z + robot_len;
				cout <<"real_robot:"<< "x:" << Point3D.x << " " << "y:" << Point3D.y << " " << "z:" << Point3D.z << endl;
			}
				kinect.ShowOpenCVImage(depthcolorMat, "depthcolor");
		}
		shang = !shang;
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
			Draw_Polygon(img_depthcolor.data, img_depthcolor.cols, img_depthcolor.rows, 4, ObjectRes[i].R, 0, 0, 255);
		}
		kinect.ShowOpenCVImage(img_depthcolor, "depthcolor");
	}
}
void DrawCenterPoints(cv::Mat& colorMat)
{
	colorMat.at<cv::Vec4b>(288, 320)[0] = 0;
	colorMat.at<cv::Vec4b>(288, 320)[1] = 0;
	colorMat.at<cv::Vec4b>(288, 320)[2] = 0;
	colorMat.at<cv::Vec4b>(288, 321)[0] = 0;
	colorMat.at<cv::Vec4b>(288, 321)[1] = 0;
	colorMat.at<cv::Vec4b>(288, 321)[2] = 0;
	colorMat.at<cv::Vec4b>(288, 319)[0] = 0;
	colorMat.at<cv::Vec4b>(288, 319)[1] = 0;
	colorMat.at<cv::Vec4b>(288, 319)[2] = 0;
	colorMat.at<cv::Vec4b>(289, 320)[3]= 0;
	colorMat.at<cv::Vec4b>(289, 320)[0] = 0;
	colorMat.at<cv::Vec4b>(289, 320)[1] = 0;
	colorMat.at<cv::Vec4b>(289, 320)[2] = 0;
}

