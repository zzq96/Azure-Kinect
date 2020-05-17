/*
Author: SHUJING LYU
TIME:2019-05-17


*/
//#include "stdafx.h"
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <stdlib.h>
#include "Function_SHUJING.h"
#include "PicoZense_api2.h"
#include "ConComponent.h"


void Image_Binary(unsigned char* image, int wid, int hei)
{
	int i, j;
	for (i = 0; i < hei; i++)
	{
		for (j = 0; j < wid; j++)
		{
			if (image[i*wid + j] < 100)
				image[i*wid + j] = 0;
			else
				image[i*wid + j] = 255;
		}
	}

}

int ObjectLocation(PsCameraParameters cameraParameters, PsDepthPixel * DepthFrameData, BYTE* DepthImage, int iDistance, int iWid, int iHei, int iTop, int iBottom, Object* ObjectRes)
{
	int i, j, k;
	int iCen = 0;
	int iHis[256];
	float fx = cameraParameters.fx;
	float fy = cameraParameters.fy;
	float cx = cameraParameters.cx;
	float cy = cameraParameters.cy;
	//处理二值化图
	BYTE* image_t1; //深度数据
	image_t1 = (BYTE*)malloc(1000 * 1000 * sizeof(BYTE));
	memset(image_t1, 255, iWid*iHei);
	//统计直方图
	memset(iHis, 0, 256 * sizeof(int));
	for (i = iTop; i < iBottom; i++)
	{
		for (j = 20; j < iWid; j++)
		{
			iHis[DepthImage[i* iWid + j]]++;
		}
	}
	//对不同高度物体按深度进行分层处理
	int inum = 5;
	int iBegin[7];
	int iEnd[7];
	iCen = iHis_part(iHis, iBegin, iEnd,inum);//返回分层数，iBegin,iEnd是每一层在直方图的直方图范围
	//逐层进行判断
	int num_com;
	int kk;
	int minx, maxx, miny, maxy;
	int vsum = 0;
	int vi = 0;
	int vlength[10];
	int vwideth[10];
	int vheight[10];
	int volume[10];
	std::vector<Point> rects[10];//ZZQ加，最小外接矩形
	int minx1[10];
	int maxx1[10];
	int miny1[10];
	int maxy1[10];
	int minx_p[10];
	int miny_p[10];
	memset(vlength, 0, sizeof(int)* 10);
	memset(vwideth, 0, sizeof(int)* 10);
	memset(vheight, 0, sizeof(int)* 10);
	memset(volume, 0, sizeof(int)* 10);
	memset(minx_p, 0, sizeof(int)* 10);
	memset(miny_p, 0, sizeof(int)* 10);
	memset(minx1, 0, sizeof(int)* 10);
	memset(maxx1, 0, sizeof(int)* 10);
	memset(miny1, 0, sizeof(int)* 10);
	memset(maxy1, 0, sizeof(int)* 10);
	int gao1 = 0;
	int inum1[1400];
	int inum_all = 0;
	int iobj_num=0;//检测到的快递数量
	CodeComponent rescomponent[20]; //连通元
	for (kk = 0; kk < iCen; kk++)
	{
		for (int i = 0; i < 20; i++)
			rescomponent[i].P.clear(), rescomponent[i].R.clear();//ZZQ
		memset(image_t1, 255, iWid*iHei*sizeof(BYTE));
		for (i = iTop; i < iBottom; i++)
		{
			for (j = 20; j < iWid; j++)
			{
				if (DepthImage[i* iWid + j] >iBegin[kk] - 1 && DepthImage[i* iWid + j] < iEnd[kk]+1)//选择置为0的阈值
				{
					image_t1[i*iWid + j] = 0;
				}
			}
		}
		

		//对image_t1做黑色画框
		num_com = 0;
		num_com = ConCompLabelling8_label(image_t1, iWid, iHei, rescomponent, FALSE, 20);//返回该层的快递数量
		if (num_com < 1)
			break;
		int pixelnum = 500;
		
		for (k = 0; k < num_com; k++)
		{
			if (pixelnum > rescomponent[k].pixelnum)
			{
				rescomponent[k].sign = true;
				continue;
			}
			if (rescomponent[k].compshape.maxx - rescomponent[k].compshape.minx < 40
				|| rescomponent[k].compshape.maxy - rescomponent[k].compshape.miny < 40)
			{
				rescomponent[k].sign = true;
				continue;
			}

			/*minx = rescomponent[k].compshape.minx;
			miny = rescomponent[k].compshape.miny;
			maxx = rescomponent[k].compshape.maxx;
			maxy = rescomponent[k].compshape.maxy;
			*/
			//找到外界矩形的上下左右边界
			minx = min(rescomponent[k].R[0].x, min(rescomponent[k].R[1].x, min(rescomponent[k].R[2].x, rescomponent[k].R[3].x)));
			miny = min(rescomponent[k].R[0].y, min(rescomponent[k].R[1].y, min(rescomponent[k].R[2].y, rescomponent[k].R[3].y)));
			maxx = max(rescomponent[k].R[0].x, max(rescomponent[k].R[1].x, max(rescomponent[k].R[2].x, rescomponent[k].R[3].x)));
			maxy = max(rescomponent[k].R[0].y, max(rescomponent[k].R[1].y, max(rescomponent[k].R[2].y, rescomponent[k].R[3].y)));
			//计算这个物体的体积
			vsum = 0;
			vi = 0;
	/*		for (i = miny; i < maxy; i++)
			{
				for (j = minx; j < maxx; j++)
				{
					// 如果点（ｉ，ｊ）不在外界矩形内就跳过
					if (pointIsInRect(Point(j,i), rescomponent[k].R) == 0)continue;//ZZQ
					vi = DepthFrameData[i*iWid + j] * DepthFrameData[i*iWid + j] * (iDistance - DepthFrameData[i*iWid + j]) / fx / fy;//计算体积公式
					vsum = vsum + vi;
				}
			}*/
			volume[iobj_num] = vsum;// / 1000;
			minx_p[iobj_num] = minx;
			miny_p[iobj_num] = miny;
			minx1[iobj_num] = minx;
			maxx1[iobj_num] = maxx;
			miny1[iobj_num] = miny;
			maxy1[iobj_num] = maxy;
			rects[iobj_num] = rescomponent[k].R;//ZZQ加
			//DepthImage[maxy*depthFrame.width + maxx] = 0;

			//计算物品高度
			gao1 = 0;			
			inum_all = 0;
			memset(inum1, 0, 1400 * sizeof(int));//统计高度
			for (i = miny; i < maxy; i++)
			{
				for (j = minx; j < maxx; j++)
				{
					if (DepthFrameData[iWid*i + j]<1400)
						inum1[DepthFrameData[iWid*i + j]]++;
				}
			}
			for (i = 20; i<1400; i++)
			{
				inum_all = inum_all + inum1[i];
				if (inum_all>(maxx - minx)*(maxy - miny) / 5)
				{
					gao1 = iDistance - i;
					break;
				}

			}
			//	gao1 = iDistance - DepthFrameData[depthFrame.width*(maxy + miny) / 2 + (maxx + minx) / 2];
			vheight[iobj_num] = gao1;

			if ((maxx - minx) > (maxy - miny))
			{

				vlength[iobj_num] = abs((maxx - cx) / fx*(iDistance - gao1) - (minx - cx) / fx*(iDistance - gao1));
				vwideth[iobj_num] = abs((maxy - cy) / fy*(iDistance - gao1) - (miny - cy) / fy*(iDistance - gao1));
			}
			else
			{
				
				vwideth[iobj_num] = abs((maxx - cx) / fx*(iDistance - gao1) - (minx - cx) / fx*(iDistance - gao1));
				vlength[iobj_num] = abs((maxy - cy) / fy*(iDistance - gao1) - (miny - cy) / fy*(iDistance - gao1));

			}
			minx1[iobj_num] = abs((minx - cx) / fx*(iDistance -0) - (0 - cx) / fx*(iDistance - 0));
			maxx1[iobj_num] = abs((maxx - cx) / fx*(iDistance - 0) - (0 - cx) / fx*(iDistance - 0));
			miny1[iobj_num] = abs((miny - cy) / fy*(iDistance - 0) - (0 - cy) / fy*(iDistance - 0));
			maxy1[iobj_num] = abs((maxy - cy) / fy*(iDistance - 0) - (0 - cy) / fy*(iDistance - 0));
			iobj_num++;
			if (iobj_num > 9)
				break;
		}

		

	}
	//目标从左到右排序
	int sn[10];
	
	double minx1_d[10];
	for (i = 0; i < 10; i++)
	{
		minx1_d[i] = minx1[i];
		sn[i] = i;
	}
	Quick_Order(minx1_d, 0, iobj_num - 1, sn);
	for (k = 0; k < iobj_num; k++)
	{
		ObjectRes[k].hei = vheight[sn[k]];
		ObjectRes[k].len = vlength[sn[k]];
		ObjectRes[k].wid = vwideth[sn[k]];

		//ObjectRes[k].len = (rects[sn[k]][0] - rects[sn[k]][1]).len();
		//ObjectRes[k].wid = (rects[sn[k]][1] - rects[sn[k]][2]).len();
		ObjectRes[k].vol = volume[sn[k]];
		ObjectRes[k].minx = minx1[sn[k]];
		ObjectRes[k].maxx = maxx1[sn[k]];
		ObjectRes[k].miny = miny1[sn[k]];
		ObjectRes[k].maxy = maxy1[sn[k]];
		ObjectRes[k].minx_p = minx_p[sn[k]];
		ObjectRes[k].miny_p = miny_p[sn[k]];
		ObjectRes[k].R = rects[sn[k]];
		
	}
	free(image_t1);
	return iobj_num;

}
int iHis_part(int* iHis, int* iBegin, int* iEnd, int inum)
{
	int i,j,k;
	int iBeg_t[20];
	int iEnd_t[20];
	int iPixel_num;
	bool bflg = false;
	k = 0;
	for (i = 1; i <255;i++)
	{
		if (bflg==false&&iHis[i] > 500)
		{
			iBeg_t[k] = i;
			bflg = true;
		}
		else if (bflg == true && iHis[i] < 500)
		{
			iEnd_t[k] = i - 1;
			bflg = false;
			k++;
		}
	}
	if (k < 1)
		return 0;
	int kk = 0;
	for (i = 0; i < k-1; i++)
	{
		iPixel_num = 0;
		for (j = iBeg_t[i]; j < iEnd_t[i] + 1; j++)
		{
			iPixel_num = iPixel_num + iHis[j];
		}
		if (iPixel_num>1000 )//&& iBeg_t[i + 1] - iEnd_t[i]>5)
		{
			iBegin[kk] = iBeg_t[i];
			iEnd[kk] = iEnd_t[i];
			kk++;
			if (kk > inum-1)
				break;
		}
	}
	//最后一层判断
	iPixel_num = 0;
	for (j = iBeg_t[k-1]; j < iEnd_t[k-1] + 1; j++)
	{
		iPixel_num = iPixel_num + iHis[j];
	}
	if (iPixel_num>500)
	{
		iBegin[kk] = iBeg_t[k-1];
		iEnd[kk] = iEnd_t[k-1];
		kk++;
	}
	return kk;
}