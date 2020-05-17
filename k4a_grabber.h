#pragma once
#include <k4a/k4a.hpp>

#include <pcl/io/boost.h>
#include <pcl/io/grabber.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui_c.h>

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
    k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
	KinectAPI();
	//get images in opencv Mat format
	//colorMat is a color image in 8UC4 format belonging to RGBA
	//depthMat is a depth image in 16UC1 format, 每个元素表示物体在相机视角下的深度，单位mm
	//depthcolor is a pseudo-color image in 8UC4，将深度图转化为伪彩色图像以便可视化
	void GetOpenCVImage(cv::Mat &colorMat, cv::Mat &depthMat, cv::Mat& depthcolorMat);
	void ShowOpenCVImage(cv::Mat Img, std::string name);
	void GetPointCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud);
	void ReleaseDevice();

private:
	uint32_t device_count;
	k4a_device_t device = NULL;
	Pixel ColorizeDepthToRGB(const DepthPixel& depthpixel,
		const DepthPixel& min,
		const DepthPixel& max);
	//对红外图像着色。“亮”趋近于白色，“暗”趋近于黑色
	Pixel ColorizeGreyScale(const DepthPixel& depthpixel,
		const DepthPixel& min,
		const DepthPixel& max);
	inline std::pair<uint16_t, uint16_t> GetDepthModeRange(const k4a_depth_mode_t depthMode);
	inline void ColorConvertHSVToRGB(float h, float s, float v, float& r, float& g, float& b);
	//返回明亮程度的阈值，正常时IR图的输出范围是[0,100]
	std::pair<uint16_t, uint16_t> GetIrLevels(const k4a_depth_mode_t depthMode);
	void ColorizeDepthImage(const k4a_image_t& depthImage,
		Pixel(KinectAPI::*visualizationFn)(const DepthPixel&, const DepthPixel&, const DepthPixel&),
		std::pair<uint16_t, uint16_t> expectedValueRange,
		std::vector<Pixel>* buffer);

};
}