/*
Author: Zhequan Zhou 
TIME:2019-07-29
*/
#ifndef Function_ZZQ
#define Function_ZZQ

#include<iostream>
#include<cstdio>
#include<math.h>
#include<algorithm>
#include<vector>
#include <windows.h>
#include<minwindef.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui_c.h>
#define INF 0x3f3f3f3f
#define INFLL 0x3f3f3f3f3f3f3f3f
#define mem(x,y) memset(x,y,sizeof(x))
typedef unsigned long long ULL;
typedef long long LL;
using namespace std;

struct Point {
	double x, y;
	Point() {}
	Point(double x, double y) :x(x), y(y) {}
	bool operator<(const Point& b) const {
		if (x != b.x) return x < b.x;
		return y < b.y;
	}
	Point operator-(const Point& P) const {
		return Point(x - P.x, y - P.y);
	}
	Point operator+(const Point& P) const {
		return Point(x + P.x, y + P.y);
	}
	double operator^(const Point& P) const { //叉积
		return x * P.y - y * P.x;//如果改成整形记得加LL
	}
	double operator*(const Point& P) const { //点积
		return x * P.x + y * P.y;//如果改成整形记得加LL
	}
	Point operator*(const double b)const {
		return Point(x * b, y * b);
	}
	Point operator/(const double b)const {
		return Point(x / b, y / b);
	}
	double len()
	{
		return sqrt(x*x+y*y);
	}

};
typedef Point Vector;
//求点集的凸包，P是输入点集，R是凸包顶点集合
void Convex(std::vector<Point>& P, std::vector<Point>& R);
//channel是输入图像的通道数3或者4。R是想在图上画出的多边形。 cb，cg，cr是BGR颜色,
void Draw_Polygon(BYTE* image, LONG lwidth, LONG lheight, int channels, const std::vector<Point>& R, int cb, int cg, int cr);

//求出点集p的外接矩阵并画出框
//image：需要画图的图像。lwidth，lheight：图像宽和高。channels：图像通道数（3或4）。p：点集。cb，cg，cr：BGR颜色值
void Draw_MBROfPoints(BYTE* image, long lwidth, long lheight, int channels, std::vector<Point>& p, int cb, int cg, int cr);
//求出多个点集p的外接矩阵并画出框
//image：需要画图的图像。lwidth，lheight：图像宽和高。channels：图像通道数（3或4）。p：点集。cb，cg，cr：BGR颜色值
void Draw_MBRsOfPoints(BYTE* image, long lwidth, long lheight, int channels, std::vector<vector<Point>>& p, int cb, int cg, int cr);
//求点集的外接四边形。P是输入点集。t是外接四边形顶点。
void RC(std::vector<Point>& P, std::vector<Point> & t);
//判断一个点是否在凸多边形内部
bool pointIsInRect(Point p, const std::vector<Point> & R);
#endif
