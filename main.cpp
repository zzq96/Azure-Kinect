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
#include"Function_SHUJING.h"
#include"SocketRobot.h"

const double PI = 3.1415926;
cv::Mat depthMat,colorMat, depthcolorMat, irMat, ircolorMat;
cv::Mat depthMatOld,colorMatOld, depthcolorMatOld, irMatOld, ircolorMatOld;
cv::Mat Depth2ColorRotation, Depth2ColorTranslation;
//���ֵ�������У��������ͼ��
cv::Mat depthMatRevise;
Object ObjectRes[10];
cv::Mat depthCameraMatrix, depthDistCoeffs;
cv::Mat colorCameraMatrix, colorDistCoeffs;
cv::Mat color_R_cam2base, color_t_cam2base;
cv::Mat depth_R_cam2base, depth_t_cam2base;
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
    //assert(isRotationMatrix(R));

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
	//TODO:����߽��ж�
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
	float ay = (a.R[0].y + a.R[1].y + a.R[2].y + a.R[3].y)/4;
	float by = (b.R[0].y + b.R[1].y + b.R[2].y + b.R[3].y)/4;
	if (shang == TRUE)
	{
		//float ay = -1000000, by = -100000;
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
		float ax = -1000000, bx = -100000;
		ax = max(a.R[0].x, max(a.R[1].x, max(a.R[2].x, a.R[3].x)));
		bx = max(b.R[0].x, max(b.R[1].x, max(b.R[2].x, b.R[3].x)));

		if (ay < 400)
			return FALSE;
		if (by < 400)
			return TRUE;
		return ax < bx;
	}
}
//TODO:û���Թ�
//�Ը߶ȴ��ϵ�������
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
	//������μ��㵽��е������ϵ
	cv::Mat a3D = depth_R_cam2base.t() * (depthCameraMatrix.inv() * aDepth * aM - depth_t_cam2base);
	cv::Mat b3D = depth_R_cam2base.t() * (depthCameraMatrix.inv() * bDepth * bM - depth_t_cam2base);
	return a3D.at<float>(2, 0) > b3D.at<float>(2, 0);
}
bool compareByArea(Object a, Object b)
{
	float aArea = abs((a.R[0] - a.R[1]) ^ (a.R[1] - a.R[2]));
	float bArea = abs((b.R[0] - b.R[1]) ^ (b.R[1] - b.R[2]));
	return aArea > bArea;
}
void HomogeneousMtr2RT(cv::Mat& HomoMtr, cv::Mat& R, cv::Mat& T)
{
	//Mat R_HomoMtr = HomoMtr(Rect(0, 0, 3, 3)); //ע��Rectȡֵ
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

cv::Mat RT2HomogeneousMatrix(const cv::Mat& R,const cv::Mat& T)
{
	cv::Mat HomoMtr;
	cv::Mat_<float> R1 = (cv::Mat_<float>(4, 3) << 
										R.at<float>(0, 0), R.at<float>(0, 1), R.at<float>(0, 2),
										R.at<float>(1, 0), R.at<float>(1, 1), R.at<float>(1, 2),
										R.at<float>(2, 0), R.at<float>(2, 1), R.at<float>(2, 2),
										0, 0, 0);
	cv::Mat_<float> T1 = (cv::Mat_<float>(4, 1) <<
										T.at<float>(0,0),
										T.at<float>(1,0),
										T.at<float>(2,0),
										1);
	cv::hconcat(R1, T1, HomoMtr);		//����ƴ��
	return HomoMtr;
}
//������Χ5�����ƽ��ֵ
void calPoint3D(cv::Mat point2D, cv::Point3f & real, UINT16 Zc)
{
	assert(Zc != 0);
	cv::Mat point3D = depth_R_cam2base.t() * (depthCameraMatrix.inv() * Zc * point2D - depth_t_cam2base);
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
	//����4���ߵ�����
	cv::Point3f ab = raa - rbb;
	cv::Point3f dc = rdd - rcc;
	cv::Point3f ad = raa - rdd;
	cv::Point3f bc = rbb - rcc;
	//���ߵķ��򵱳�x���y��ƽ��һ��
	cv::Point3f v1 = (ab + dc) / 2;
	cv::Point3f v2 = (ad + bc) / 2;
	//�ѽϳ��ıߵ���x�ᣬ��Ϊ2��������x�᷽���
	cv::Point3f X, Y;
	if (v1.dot(v1) > v2.dot(v2))
	{
		X = v1 / sqrt(v1.dot(v1));//��λ��
		Y = v2 / sqrt(v2.dot(v2));
	}
	else 
	{
		X = v2 / sqrt(v2.dot(v2));//��λ��
		Y = v1 / sqrt(v1.dot(v1));
	}

	if (X.x < 0)
		X *= -1;

	//��������������Ȼ��ƽ��һ�£�����
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

	//��֪x���z�ᣬ���ݲ�����y��,��֮ǰ�õ�y��ͳһ����
	if (normal.cross(X).x * Y.x < 0)
		Y *= -1;
	cout << "����" << endl;
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
	
	//����֮ǰ�����x,y,z��3���᲻һ��������������������������ֵ�ֽ�ķ������һ�����������ľ���
	cv::Mat w, u, vt;
	cv::SVDecomp(rotation, w, u, vt);
    cv::Mat I = cv::Mat::eye(3,3, rotation.type());
	cv::Mat new_rotation = u * I * vt;

	return new_rotation;
}
/*������z��*/
float calAngle(const vector<Point>& R, int h, int w, UINT16 depth)
{
	cv::Mat point2D(3, 1, CV_32F, cv::Scalar(0));
	point2D.at<float>(2, 0) = 1;
	cv::Mat point3D(3, 1, CV_32F, cv::Scalar(0));
	cv::Point3f a, b, c;//�����������������ʵ����
	/*���ĳһ���㳬���߽磬��˳�Ƶ㡣ë����Ӧ�����ֻ��һ���㳬���߽�*/
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
	cout << "�߳�1" << getDistance(a, b) << ":�߳�2" << getDistance(b, c) << endl;
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

//TODO:ɾ����Щ�غϵ�����
void DeleteBadObejct(Object *ObjectRes, int &iObj_num) 
{
	//������Ӵ�С����
	sort(ObjectRes, ObjectRes + iObj_num, compareByArea);
	for (int i = 0; i < iObj_num; i++)
	{
		for (int j = i + 1; j < iObj_num; j++)
		{
			//�ҵ�j�����ĵ�
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
				cout << "ɾ��һ�����壬xyΪ��" << x << " " << y << endl;
			}
		}
	}
}
//�����ĵ�Ϳ�ڣ������ҵ������ϵģ�0,0����
//����GetXYZAtCameraView����
k4a::KinectAPI kinect;
int main5()
{
	
	string caliberation_camera_file = "caliberation_camera.xml";
	string Homo_cam2base_file = "Homo_cam2base.xml";
	cv::FileStorage fs(caliberation_camera_file, cv::FileStorage::READ); //��ȡ�궨XML�ļ�  
	//��ȡ���ͼ���ڲξ���
	fs["depth_cameraMatrix"] >> depthCameraMatrix;
	depthCameraMatrix.convertTo(depthCameraMatrix, CV_32F);
	cout << depthCameraMatrix.type() << endl;
	fs["depth_distCoeffs"] >> depthDistCoeffs;
	cout << "depthCameraMatrix" << depthCameraMatrix << endl;
	cout << "depthdisCoeffs" << depthDistCoeffs << endl;
	//��ȡcolorͼ���ڲξ���
	fs["color_cameraMatrix"] >> colorCameraMatrix;
	colorCameraMatrix.convertTo(colorCameraMatrix, CV_32F);
	cout << colorCameraMatrix.type() << endl;
	fs["color_distCoeffs"] >> colorDistCoeffs;
	cout << "colorCameraMatrix" << colorCameraMatrix << endl;
	cout << "colordisCoeffs" << colorDistCoeffs << endl;
	fs.release();

	cv::FileStorage fs2(Homo_cam2base_file, cv::FileStorage::READ); //��ȡ����������ת����ϵXML�ļ�  
	cv::Mat color_Homo_cam2base;
	fs2["color_Homo_cam2base"] >> color_Homo_cam2base;
	cout << "color_Homo_cam2base" << color_Homo_cam2base << endl;
	/*ΪʲôҪ���棿*/
	color_Homo_cam2base = color_Homo_cam2base.inv();
	fs2.release();
	/*����Ӧ����ת��Ϊ��ת�����ƽ�������������������*/
	HomogeneousMtr2RT(color_Homo_cam2base, color_R_cam2base, color_t_cam2base);

	

	kinect.GetRotationAndTranslationFromDepth2Color(Depth2ColorRotation, Depth2ColorTranslation);

	cv::Mat Homo_depth2color = RT2HomogeneousMatrix(Depth2ColorRotation, Depth2ColorTranslation);


	cv::Mat point2D(3, 1, CV_32F, cv::Scalar(0)); 
	point2D.at<float>(2, 0) = 1;
	cv::Mat point3D;
	//cv::Mat point3D = color_R_cam2base.t() * (depthCameraMatrix.inv() * Zc * point2D - t_cam2base);

	int cnt = 0;
	try {
		//pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud < pcl::PointXYZRGB>());
		//opencvͼ��
		int img_cnt = 100;
		int real_cnt = 0;
		for(int i = 0; i < img_cnt; i++)
		{
			printf("��%d��ͼƬ\n", real_cnt);
			kinect.GetOpenCVImage(colorMatOld, depthMatOld, depthcolorMatOld, irMat, FALSE);
			///*�����ڲκͻ���ϵ��У��ͼ��*/
			undistort(depthMatOld,depthMat,depthCameraMatrix,depthDistCoeffs);  
			undistort(depthcolorMatOld,depthcolorMat,depthCameraMatrix,depthDistCoeffs);  
			undistort(colorMatOld,colorMat,colorCameraMatrix,colorDistCoeffs);  
			//kinect.ShowOpenCVImage(colorMat, "color");
			//kinect.ShowOpenCVImage(colorMatOld, "color");
			//kinect.ShowOpenCVImage(depthcolorMat, "depthcolor");
			//kinect.ShowOpenCVImage(depthcolorMatOld, "depthcolor");
			//��depthת��Ϊcolor�ӽ�
			cv::Mat colorMatRevise(depthMat.rows, depthMat.cols, CV_8UC4, cv::Scalar(0));

			for(int i = 0; i < depthMat.rows;i++)
				for (int j = 0; j < depthMat.cols; j++)
				if(depthMat.at<UINT16>(i,j)!=0)
				{
					point2D.at<float>(0, 0) = j;
					point2D.at<float>(1, 0) = i;
					//��depth��ͼ������ϵת��Ϊdepth�������ϵ
					point3D = depthCameraMatrix.inv() * point2D * depthMat.at<UINT16>(i, j);
					point3D = Depth2ColorRotation * point3D + Depth2ColorTranslation;
					//cout << j << " " << i << " " << depthMat.at<UINT16>(i, j) << endl;
					point2D = colorCameraMatrix * point3D / point3D.at<float>(2, 0);
					int new_i = point2D.at<float>(0, 1), new_j = point2D.at<float>(0, 0);
					if (new_i >= 0 && new_j >= 0 && new_i < colorMat.rows && new_j < colorMat.cols)
					{
						colorMatRevise.at<UINT32>(i, j) = colorMat.at<UINT32>(new_i, new_j);
						//cout << depthcolorMat.at<UINT32>(new_i, new_j) << endl;
					}
				}
			//cv::Mat irMat_ = irMat.clone();

			//for(int x = 0; x < irMat.rows; x ++)
			//	for (int y = 0; y < irMat.cols; y++)
			//	{
			//		irMat_.at<UINT16>(x, y) /= 5;
			//		cout << irMat_.at<UINT16>(x, y) << endl;
			//	}

			//cout << endl;
			//cout << "finished ori" << endl;

			kinect.ShowOpenCVImage(colorMatRevise, "color", 0);
			kinect.ShowOpenCVImage(depthcolorMat, "depthcolor", 0);
			//kinect.ShowOpenCVImage(irMat_, "ir");
			while (1)
			{
				printf("�Ƿ񱣴�ͼƬ������y����n\n");
				string key;
				getline(cin, key);

				if (key == "y") {}
				else if (key == "n") 
				{ 
					printf("ͼƬδ���棬���������������\n");
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
				cv::imwrite(name + "_color.png", colorMatRevise);
				real_cnt++;
				printf("��%d��ͼƬ����ɹ������������������\n", real_cnt);
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

	//	//ת��Ϊ����
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
	//	pass.setFilterFieldName("z");          //���ù���ʱ����Ҫ�������͵�Z�ֶ�
	//	pass.setFilterLimits(0.2, 10);         //�����ڹ����ֶεķ�Χ
	//	pass.setFilterLimitsNegative(false);   //�������ǹ��˵���Χ�ڵĵ�
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
	
	try {
		//opencvͼ��
		int img_cnt = 100;
		int real_cnt = 0;
		for(int i = 0; i < img_cnt; i++)
		{
			printf("��%d��ͼƬ\n", real_cnt);
			cv::Mat depthMat, colorMat, depthcolorMat ,irMat, ircolorMat;
			kinect.GetOpenCVImage(colorMat, depthMat, depthcolorMat, irMat, false);
			//cv::Mat irMat_ = irMat.clone();

			//for(int x = 0; x < irMat.rows; x ++)
			//	for (int y = 0; y < irMat.cols; y++)
			//	{
			//		irMat_.at<UINT16>(x, y) /= 5;
			//		cout << irMat_.at<UINT16>(x, y) << endl;
			//	}

			//cout << endl;
			//cout << "finished ori" << endl;

			kinect.ShowOpenCVImage(colorMat, "color", 0);
			kinect.ShowOpenCVImage(depthcolorMat, "depthcolor", 0);
			//kinect.ShowOpenCVImage(irMat_, "ir");
			while (1)
			{
				printf("�Ƿ񱣴�ͼƬ������y����n\n");
				string key;
				getline(cin, key);

				if (key == "y") {}
				else if (key == "n") 
				{ 
					printf("ͼƬδ���棬���������������\n");
					system("pause");
					break; 
				}
				else { continue; }
				std::string name;
				if(real_cnt < 10)
					name = "Data/imgs/img0" + std::to_string(real_cnt);
				else 
					name = "Data/imgs/img" + std::to_string(real_cnt);
				cv::imwrite(name + "_depth.png", depthMat);
				cv::imwrite(name + "_depthcolor.png", depthcolorMat);
				cv::imwrite(name + "_color.png", colorMat);
				cv::imwrite(name + "_ir.png", irMat);
				real_cnt++;
				printf("��%d��ͼƬ����ɹ������������������\n", real_cnt);
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

	//	//ת��Ϊ����
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
	//	pass.setFilterFieldName("z");          //���ù���ʱ����Ҫ�������͵�Z�ֶ�
	//	pass.setFilterLimits(0.2, 10);         //�����ڹ����ֶεķ�Χ
	//	pass.setFilterLimitsNegative(false);   //�������ǹ��˵���Χ�ڵĵ�
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
