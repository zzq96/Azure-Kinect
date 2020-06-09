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
void Convex(std::vector<Point>& P, std::vector<Point>& R);
void Draw_Convex(cv::Mat &image, LONG lwidth, LONG lheight, const std::vector<Point>& R, int cb, int cg, int cr);
void RC(std::vector<Point>& P, std::vector<Point> & t);
bool pointIsInRect(Point p, const std::vector<Point> & R);
#endif
