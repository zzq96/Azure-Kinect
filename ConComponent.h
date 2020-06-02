/*********************************/
/*Lv Shujing**********************/ 
/*功能： 连通元快速标定
//20170221-添加应用矩计算倾斜角度函数
/*********************************/
#ifndef _INC_ConComponentAPI
#define _INC_ConComponentAPI
#include "Function_ZZQ.h"//ZZQ加，2019.7.29
//---begin---连通元算法结构体---begin----//
typedef struct a{
	long y;//游程的Y座标值
	long beginx;//游程开始处的X座标值
	long endx;//游程末端的X座标值
	struct a *next;//指向连通元中的下一个游程
	struct a *linenext;//指向该行中的下一个游程
}runnode;//游程节点
typedef struct b{
	long minx;//连通元的最左边像素的坐标
	long maxx;//连通元的最右边像素的坐标
	long miny;//连通元的最上边像素的坐标
	long maxy;//连通元的最下边像素的坐标
}shape;
typedef struct d{
	runnode *firstrunnode;//连通元中的第一个游程
	runnode *lastrunnode;//指向连通元中的最后一个游程
	int value;//连通元的标签值
	shape compshape;//连通元的形状信息
	LONG pixelnum;//连通元中的前景像素数
	BOOL sign;//若sign为TRUE，则表示value值表示该component所在连通元的根的索引值，若sign为FALSE,则表示该component所在连通元的根，其value值表示其颜色值
	POINT Barycenter;//连通元的重心
}component;

typedef struct e{
	int value;//component的标签值
	shape compshape;//component的外界矩阵信息
	LONG pixelnum;//component的前景像素数
	BOOL sign;//若sign为TRUE，则表示value值表示该component所在连通元的根的索引值，若sign为FALSE,则表示该component所在连通元的根，其value值表示其颜色值
	unsigned int 
		distance;//蓄水池的方向，若为1则蓄水池方向向上，若为-1则蓄水池方向向下，若为0则是一个空洞，若为其他值，则说明不是一个蓄水池
	POINT Barycenter;//component的重心
	std::vector<Point> R;//最小外接矩形，ZZQ加，2019.7.29
	std::vector<Point>P;//边界，ZZQ
}CodeComponent;
//---end---连通元算法结构体----end------//

//标记连通元
int ConCompLabelling8(BYTE *lpDIB,LONG	lWidth,LONG	lHeight,CodeComponent *rescomponent,BOOL imageflag,int max_num);
void ReleaseList(runnode **HeadRun,int Length);//释放连通元列表内存
void Calculate_centroidc(CodeComponent* component, int comp_num);
int ConCompLabelling8_label(BYTE *lpDIB, LONG lWidth, LONG lHeight, CodeComponent *rescomponent, BOOL imageflag,int max_num);
int Cal_Ang(BYTE *pImage, int iWidth, int iHeight, int iTop, int iBottom, int iLeft, int iRight, int iValue);
//int Cal_Round(BYTE *pImage, int iWidth, int iHeight, int iTop, int iBottom, int iLeft, int iRight,CPoint Round[8], int iValue);//计算外包围点
//连通元从左到右排序
void Comp_Order(CodeComponent *rescomponent, int Comp_num);
//快速排序
int Partition(double * aaa, int low, int high, int * sn);
void Quick_Order(double * aaa, int left, int right, int * sn);
//计算图像的灰度范围
void Gray_Area(BYTE *proImage, int lWidth, int lHeight,int &gray1,int &gray2);
//全局阈值与局部阈值相接合的二值化方法
void Otsu_Bernsen(BYTE* proImage, int lWidth, int lHeight,int iValue);
#endif
