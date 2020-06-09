

#include "function_zzq.h"
#include<assert.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui_c.h>
double eps = 1e-8;
int sign(double x) {
	if (fabs(x) < eps)return 0;
	return x < 0 ? -1 : 1;
}
int dcmp(double a, double b) {
	if (fabs(a - b) < eps)return 0;
	else if (a > b)return 1;
	else return -1;
}
//求凸包，p是点集，r是输出的点
void Convex(std::vector<Point>& p, std::vector<Point>& r) {
	int n = p.size();
	int k;
	sort(p.begin(), p.end());

	for (int i = 0; i < n; i++) {
		while (r.size() > 1 && sign((r[r.size() - 1] - r[r.size() - 2]) ^ (p[i] - r[r.size() - 2])) <= 0) r.pop_back();
		r.push_back(p[i]);
	}
	k = r.size();
	for (int i = n - 2; i >= 0; i--) {
		while (r.size() > k && sign((r[r.size() - 1] - r[r.size() - 2]) ^ (p[i] - r[r.size() - 2])) <= 0) r.pop_back();
		r.push_back(p[i]);
	}
	if (n > 1) r.pop_back();
}
//在image画p1到p2的一条线，y=kx+b
//RGBA格式
void Draw_Line(BYTE *image, long lwidth, long lheight, const Point& p1, const Point& p2, int channelNum, int b, int g, int r)
{
	int x1=p1.x, y1=p1.y, x2=p2.x, y2=p2.y;
	int dx, dy;
	dx = x2 - x1;
	dy = y2 - y1;
	//if (min(x2, x1) < 0 || max(x2, x1) > lwidth || min(y2, y1) < 0 || max(y2, y1) > lheight)
	//	return;
	int x, y, p, i;
	if (abs(dx) > abs(dy))
	{
		if (x1 < x2)x = x1, y = y1;
		else x = x2, y = y2;
		if (x< 0 || x + 2 > lwidth || y < 0 || y + 2> lheight)
		{
			;
		}
		else
		{
			image[y * lwidth * channelNum + x * channelNum] = b;
			image[y * lwidth * channelNum + x * channelNum + 1] = g;
			image[y * lwidth * channelNum + x * channelNum + 2] = r;

			image[y * lwidth * channelNum + (x + 1) * channelNum] = b;
			image[y * lwidth * channelNum + (x + 1) * channelNum + 1] = g;
			image[y * lwidth * channelNum + (x + 1) * channelNum + 2] = r;

			image[(y + 1) * lwidth * channelNum + x * channelNum] = b;
			image[(y + 1) * lwidth * channelNum + x * channelNum + 1] = g;
			image[(y + 1) * lwidth * channelNum + x * channelNum + 2] = r;
		}
		p = 2 * abs(dy) - abs(dx);
		for (i = min(x1, x2); i < max(x1, x2); i++)
		{
			x++;
			if (p >= 0)
			{
				if (dx*dy >= 0)
					y++;
				else y--;
				p = p + 2 * (abs(dy) - abs(dx));
			}
			else {
				y = y;
				p = p + 2 * abs(dy);
			}
			if (x< 0 || x+2 > lwidth || y < 0 || y+2> lheight)
					continue;
			image[y * lwidth * channelNum + x * channelNum] = b;
			image[y * lwidth * channelNum + x * channelNum + 1] = g;
			image[y * lwidth * channelNum + x * channelNum + 2] = r;

			image[y * lwidth * channelNum + (x + 1) * channelNum] = b;
			image[y * lwidth * channelNum + (x + 1) * channelNum + 1] = g;
			image[y * lwidth * channelNum + (x + 1) * channelNum + 2] = r;

			image[(y + 1) * lwidth * channelNum + x * channelNum] = b;
			image[(y + 1) * lwidth * channelNum + x * channelNum + 1] = g;
			image[(y + 1) * lwidth * channelNum + x * channelNum + 2] = r;
		}
	}
	else {
		if (y1 < y2)
		{
			x = x1;
			y = y1;
		}
		else {
			x = x2;
			y = y2;
		}
		if (x< 0 || x + 2 > lwidth || y < 0 || y + 2> lheight)
		{
			;
		}
		else
		{
			image[y * lwidth * channelNum + x * channelNum] = b;
			image[y * lwidth * channelNum + x * channelNum + 1] = g;
			image[y * lwidth * channelNum + x * channelNum + 2] = r;

			image[y * lwidth * channelNum + (x + 1) * channelNum] = b;
			image[y * lwidth * channelNum + (x + 1) * channelNum + 1] = g;
			image[y * lwidth * channelNum + (x + 1) * channelNum + 2] = r;

			image[(y + 1) * lwidth * channelNum + x * channelNum] = b;
			image[(y + 1) * lwidth * channelNum + x * channelNum + 1] = g;
			image[(y + 1) * lwidth * channelNum + x * channelNum + 2] = r;
		}
		p = 2 * abs(dx) - abs(dy);
		for (i = min(y1, y2); i < max(y1, y2); i++)
		{
			y++;
			if(p >= 0)
			{
				if (dx*dy >= 0)x++;
				else x--;
				p = p + 2 * (abs(dx) - abs(dy));
			}
			else {
				x = x;
				p += 2 * abs(dx);
			}
			if (x< 0 || x+2 > lwidth || y < 0 || y+2> lheight)
				continue;
			image[y * lwidth * channelNum + x * channelNum] = b;
			image[y * lwidth * channelNum + x * channelNum + 1] = g;
			image[y * lwidth * channelNum + x * channelNum + 2] = r;

			image[y * lwidth * channelNum + (x + 1) * channelNum] = b;
			image[y * lwidth * channelNum + (x + 1) * channelNum + 1] = g;
			image[y * lwidth * channelNum + (x + 1) * channelNum + 2] = r;

			image[(y + 1) * lwidth * channelNum + x * channelNum] = b;
			image[(y + 1) * lwidth * channelNum + x * channelNum + 1] = g;
			image[(y + 1) * lwidth * channelNum + x * channelNum + 2] = r;
		}
	}

}
//在image上画凸包
int flag = 0;
void Draw_Convex(cv::Mat & image, long lwidth, long lheight, const std::vector<Point>& r, int cb, int cg, int cr)
{
	//if (flag >=2 )return;
	int n = r.size();
	for (int i = 0; i < n; i++)
	{
	/*	int x = r[i].x, y = r[i].y;
		image[y * lwidth * 3 + x * 3] = 0;
		image[y * lwidth * 3 + x * 3 + 1] = 255;
		image[y * lwidth * 3 + x * 3 + 2] = 0;

		x = x, y = y+1;
		image[y * lwidth * 3 + x * 3] = 0;
		image[y * lwidth * 3 + x * 3 + 1] = 255;
		image[y * lwidth * 3 + x * 3 + 2] = 0;

		x = x, y = y - 1;
		image[y * lwidth * 3 + x * 3] = 0;
		image[y * lwidth * 3 + x * 3 + 1] = 255;
		image[y * lwidth * 3 + x * 3 + 2] = 0;*/
		Draw_Line(image.data, lwidth, lheight, r[i], r[(i + 1) % n], image.channels(), cb, cg, cr);
	}
	//flag ++;
}
//求最小面积外界矩形
void RC(std::vector<Point>& v, std::vector<Point>& ans)
{
	ans.resize(4);
	double min_s = 1e18;
	int cnt = v.size();
	assert(cnt >= 4);
	v.push_back(v[0]);
	int u = 1, r = 1, l = 1;
	for (int i = 0; i < cnt; ++i) {
		// 最上面的点
		while (dcmp(fabs((v[u] - v[i]) ^ (v[i + 1] - v[i])), fabs((v[u + 1] - v[i]) ^ (v[i + 1] - v[i]))) <= 0) {
			u = (u + 1) % cnt;
		}

		// 最右边的点
		while (dcmp((v[r] - v[i])*(v[i + 1] - v[i]), (v[r + 1] - v[i])*(v[i + 1] - v[i])) <= 0) {
			r = (r + 1) % cnt;
		}

		if (!i) l = r;

		// 最左边的点
		while (dcmp((v[l] - v[i])*(v[i + 1] - v[i]), (v[l + 1] - v[i])*(v[i + 1] - v[i])) >= 0) {
			l = (l + 1) % cnt;
		}
		double d = (v[i] - v[i + 1]).len();
		assert(fabs(d)>1e-6);
		double R = (v[r] - v[i])*(v[i + 1] - v[i]) / d;
		double L = (v[l] - v[i])*(v[i + 1] - v[i]) / d;
		double ll = R - L;
		double dd = fabs((v[u] - v[i]) ^ (v[i + 1] - v[i])) / d;
		double s = ll * dd;
		if (s < min_s) {
			min_s = s;
			ans[0] = v[i] + (v[i + 1] - v[i]) * (R / d);
			ans[1] = ans[0] + Point(-(v[i + 1] - v[i]).y, (v[i + 1] - v[i]).x) / Point(-(v[i + 1] - v[i]).y, (v[i + 1] - v[i]).x).len() *dd;
			//ans[1] = ans[0] + (v[r] - ans[0]) * (dd / (v[r] - ans[0]).len());
			//assert((v[r] - ans[0]).len()>1e-6);
			ans[2] = ans[1] + (v[i] - ans[0]) * (ll / R);
			ans[3] = ans[2] + (ans[0] - ans[1]);
		}

	}
	//sort(ans.begin(), ans.end());
}
bool pointIsInRect(Point p, const std::vector<Point> & R)
{
	//面积
	double area1=0, area2=0;
	int j;
	for (int i = 0; i < R.size(); i++)
	{
		j = (i + 1) % R.size();
		area1 = area1 + ((R[i] - p) ^ (R[j] - p));
		area2  = area2+abs((R[i] - p)^(R[j] - p));
	}
	//如果两种面积一样，说明p在R中
	return abs(abs(area1)- abs(area2))<1e-6;
}
