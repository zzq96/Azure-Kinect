/*
Author: SHUJING LYU
TIME:2019-05-17


*/
#ifndef _INC_Function_SHUJINGAPI
#define _INC_Function_SHUJINGAPI
#include "PicoZense_api2.h"
#include "Function_ZZQ.h"
typedef struct aa
{
	int minx_p;
	int miny_p;
	int minx;
	int maxx;
	int miny;		   
	int maxy;
	int len;
	int wid;
	int hei;
	int vol;
	std::vector<Point> R;//最小外界矩形，ZZQ加
}Object;

void Image_Binary(BYTE* image, int wid, int hei);
int ObjectLocation(PsCameraParameters cameraParameters, PsDepthPixel * DepthFrameData, BYTE* DepthImage, int iDistance, int iWid, int iHei, int iTop, int iBottom, Object* ObjectRes);
int iHis_part(int* iHis, int* iBegin, int* iEnd,int inum);
#endif