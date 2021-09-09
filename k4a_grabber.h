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
	//���������ڲκͻ���ϵ��
	cv::Mat depthCameraMatrix, depthDistCoeffs;
	//RGB������ڲκͻ���ϵ��
	cv::Mat colorCameraMatrix, colorDistCoeffs;
	//��������RGB�������ת�����ƽ������
	cv::Mat Depth2ColorRotation, Depth2ColorTranslation;
	//��ʼ������
	KinectAPI(const std::string &caliberation_camera_file, bool verbose);
    k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
	//get images in opencv Mat format
	//colorMat is a color image in 8UC4 format belonging to RGBA
	//depthMat is a depth image in 16UC1 format, ÿ��Ԫ�ر�ʾ����������ӽ��µ���ȣ���λmm
	//depthcolor is a pseudo-color image in 8UC4�������ͼת��Ϊα��ɫͼ���Ա���ӻ�
	void GetOpenCVImage(cv::Mat& colorMat, cv::Mat& depthMat, cv::Mat& depthcolorMat, cv::Mat& irMat, bool isDepth2Color);
	//��kinect���api�ж�ȡ��������RGB�������ת�����ƽ������
	void GetRotationAndTranslationFromDepth2Color(cv::Mat& Depth2ColorRotation, cv::Mat& Depth2ColorTranslation);
	void ShowOpenCVImage(cv::Mat &Img, std::string name, int waitkey);
	//�����ڲκͻ���ϵ������ͼƬ
	void undistort(cv::Mat& ImgOld, cv::Mat& Img, const std::string& type);
	//������������RGB�������ת�����ƽ��������RGBͼת�������ͼ�ӽ�
	void ConvertColor2Depth(cv::Mat& colorMat, cv::Mat& depthMat, cv::Mat& colorMatRevise);
	//�õ�Mat��ʽ������ڲ�
	//cameraType��"depth"or"color"
	//discoeffs:k1, k2, p1, p2, k3
	void GetIntrinsicParam(cv::Mat& cameraMatrix, cv::Mat& disCoeffs, const std::string cameraType);
	//��������ڲ�, �����е�����, ����û��
	void SetIntrinsicParam(cv::Mat& depthcameraMatrix, cv::Mat& depthdisCoeffs, cv::Mat& colorcameraMatrix, cv::Mat& colordisCoeffs);

	//TODO(zzq):Ŀ������ϵ�µ�XYZֵ
	//�õ�Ŀ������ϵ�µ�xyzֵ
	//����ͼ���ϵ��������꣬���Ŀ������ϵ�µ�xyzֵ,
	void GetXYZAfterTransformation(const cv::Point2i coord, cv::Point3f &coord3D, const cv::Mat transformation);
	//TODO(zzq):�������ϵ�µ�xyzֵ
	//�õ��������ϵ�µ�xyzֵ
	//����ͼ���ϵ��������꣬����������ϵ�µ�xyzֵ
	void GetXYZAtCameraView(const cv::Point2i point2D, double depth, cv::Point3f &point3D);
	//��������󣬽������Դ�ͷš�
	void ReleaseDevice();

private:
	k4a_calibration_t calibration;
	// TODO(zzq):ȷ�������ļ���ʽ��д�����õĺ���
	void ReadConfig(std::string dir);
	//�������
	uint32_t device_count;
	k4a_device_t device = NULL;
	//�Ժ���ͼ����ɫ�������������ڰ�ɫ�������������ں�ɫ
	Pixel ColorizeGreyScale(const DepthPixel& depthpixel,
		const DepthPixel& min,
		const DepthPixel& max);
	//�ҵ����ֵ��������Сֵ
	inline std::pair<uint16_t, uint16_t> GetDepthModeRange(const k4a_depth_mode_t depthMode);
	inline void ColorConvertHSVToRGB(double h, double s, double v, double& r, double& g, double& b);
	//���������̶ȵ���ֵ������ʱIRͼ�������Χ��[0,100]
	std::pair<uint16_t, uint16_t> GetIrLevels(const k4a_depth_mode_t depthMode);
	//�����ͼα��ɫ��������۲�
	void ColorizeDepthImage(const k4a_image_t& depthImage,
		Pixel(KinectAPI::*visualizationFn)(const DepthPixel&, const DepthPixel&, const DepthPixel&),
		std::pair<uint16_t, uint16_t> expectedValueRange,
		std::vector<Pixel>* buffer);
	//�н����溯�������������һ�����ֵת��ΪRGBAֵ
	Pixel ColorizeDepthToRGB(const DepthPixel& depthpixel,
		const DepthPixel& min,
		const DepthPixel& max);

};
}