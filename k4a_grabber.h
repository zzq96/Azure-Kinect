#pragma once
#include <k4a/k4a.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui_c.h>

//TODO: change namespace from k4a to robot
namespace k4a {

struct Pixel {
	uint8_t Blue;
	uint8_t Green;
	uint8_t Red;
	uint8_t Alpha;
};
using DepthPixel = uint16_t;
class KinectAPI {
public:
	cv::Mat depthCameraMatrix, depthDistCoeffs;
	cv::Mat colorCameraMatrix, colorDistCoeffs;
	cv::Mat Depth2ColorRotation, Depth2ColorTranslation;
	//初始化函数
	KinectAPI(const std::string &caliberation_camera_file, bool verbose);
    k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
	//get images in opencv Mat format
	//colorMat is a color image in 8UC4 format belonging to RGBA
	//depthMat is a depth image in 16UC1 format, 每个元素表示物体在相机视角下的深度，单位mm
	//depthcolor is a pseudo-color image in 8UC4，将深度图转化为伪彩色图像以便可视化
	void GetOpenCVImage(cv::Mat& colorMat, cv::Mat& depthMat, cv::Mat& depthcolorMat, cv::Mat& irMat, bool isDepth2Color);
	void GetRotationAndTranslationFromDepth2Color(cv::Mat& Depth2ColorRotation, cv::Mat& Depth2ColorTranslation);
	void ShowOpenCVImage(cv::Mat &Img, std::string name, int waitkey);
	void undistort(cv::Mat& ImgOld, cv::Mat& Img, const std::string& type);
	void ConvertColor2Depth(cv::Mat& colorMat, cv::Mat& depthMat, cv::Mat& colorMatRevise);
	//得到Mat格式的相机内参
	//cameraType是"depth"or"color"
	//discoeffs:k1, k2, p1, p2, k3
	void GetIntrinsicParam(cv::Mat& cameraMatrix, cv::Mat& disCoeffs, const std::string cameraType);
	void SetIntrinsicParam(cv::Mat& depthcameraMatrix, cv::Mat& depthdisCoeffs, cv::Mat& colorcameraMatrix, cv::Mat& colordisCoeffs);

	//TODO(zzq):目标坐标系下的XYZ值
	//得到目标坐标系下的xyz值
	//输入图像上的像素坐标，输出目标坐标系下的xyz值
	void GetXYZAfterTransformation(const cv::Point2i coord, cv::Point3f &coord3D, const cv::Mat transformation);
	//TODO(zzq):相机坐标系下的xyz值
	//得到相机坐标系下的xyz值
	//输入图像上的像素坐标，输出相机坐标系下的xyz值
	void GetXYZAtCameraView(const cv::Point2i point2D, double depth, cv::Point3f &point3D);
	//用完相机后，将相机资源释放。
	void ReleaseDevice();

private:
	k4a_calibration_t calibration;
	// TODO(zzq):确定配置文件格式，写读配置的函数
	void ReadConfig(std::string dir);
	//相机数量
	uint32_t device_count;
	k4a_device_t device = NULL;
	//对红外图像着色。“亮”趋近于白色，“暗”趋近于黑色
	Pixel ColorizeGreyScale(const DepthPixel& depthpixel,
		const DepthPixel& min,
		const DepthPixel& max);
	//找到深度值的最大的最小值
	inline std::pair<uint16_t, uint16_t> GetDepthModeRange(const k4a_depth_mode_t depthMode);
	inline void ColorConvertHSVToRGB(double h, double s, double v, double& r, double& g, double& b);
	//返回明亮程度的阈值，正常时IR图的输出范围是[0,100]
	std::pair<uint16_t, uint16_t> GetIrLevels(const k4a_depth_mode_t depthMode);
	//将深度图伪彩色化，方便观察
	void ColorizeDepthImage(const k4a_image_t& depthImage,
		Pixel(KinectAPI::*visualizationFn)(const DepthPixel&, const DepthPixel&, const DepthPixel&),
		std::pair<uint16_t, uint16_t> expectedValueRange,
		std::vector<Pixel>* buffer);
	//承接上面函数，这个函数将一个深度值转化为RGBA值
	Pixel ColorizeDepthToRGB(const DepthPixel& depthpixel,
		const DepthPixel& min,
		const DepthPixel& max);

};
}