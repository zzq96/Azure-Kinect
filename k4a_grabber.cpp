#include <Eigen/Dense>

#include<assert.h>
#include "k4a_grabber.h"
using namespace std;

k4a::KinectAPI::KinectAPI()
{
    device_count = k4a_device_get_installed_count();
	if (device_count == 0)
		throw "No k4a devices attached!\n";
	else printf("Found %d connected devices\n", device_count);

    if (K4A_FAILED(k4a_device_open(K4A_DEVICE_DEFAULT, &device)))
		throw "Failed to open k4a device!\n";

    // Configure a stream of 4096x3072 BRGA color data at 15 frames per second
    config.camera_fps       = K4A_FRAMES_PER_SECOND_15;//设定每一秒帧数
    config.color_format     = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    config.color_resolution = K4A_COLOR_RESOLUTION_720P;
	config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
	//config.depth_mode = K4A_DEPTH_MODE_WFOV_UNBINNED;
	config.synchronized_images_only = true;

    // Start the camera with the given configuration
    if (K4A_FAILED(k4a_device_start_cameras(device, &config)))
    {
        k4a_device_close(device);
		throw "Failed to start cameras!\n";
    }
	if (K4A_RESULT_SUCCEEDED !=
		k4a_device_get_calibration(device, config.depth_mode, config.color_resolution, &calibration))
	{
		throw ("Failed to get calibration\n");
	}
}

void k4a::KinectAPI::SetIntrinsicParam(cv::Mat& depthcameraMatrix,cv::Mat &depthdisCoeffs,cv::Mat& colorcameraMatrix,cv::Mat &colordisCoeffs)
{

	k4a_calibration_camera_t *depthCameraCalibration = &calibration.depth_camera_calibration;
	k4a_calibration_camera_t *colorCameraCalibration = &calibration.color_camera_calibration;
	//kinect相机应该有4个传感器，下面的外参是4*4的一个矩阵，其中下标[i][j]应该是第i的传感器到第j个传感器的转化矩阵
	//[0][1]应该是depth到color的矩阵，但是没测试过，以后用到在测试。
	k4a_calibration_extrinsics_t extrinsics = calibration.extrinsics[0][0];
	//下面代码，当[0][0]时，输出分别为【1,0,0】和【0，0,0】就是本身到本身的转化矩阵
	//cout << extrinsics.rotation[0] << " " << extrinsics.rotation[1] << " " << extrinsics.rotation[2] << endl;
	//cout << extrinsics.translation[0] << " " << extrinsics.translation[1] << " " << extrinsics.translation[2] << endl;

	k4a_calibration_extrinsics_t *depthExtrinsics= &depthCameraCalibration->extrinsics;
	k4a_calibration_intrinsics_t *depthIntrinsics= &depthCameraCalibration->intrinsics;

	k4a_calibration_extrinsics_t *colorExtrinsics= &colorCameraCalibration->extrinsics;
	k4a_calibration_intrinsics_t *colorIntrinsics= &colorCameraCalibration->intrinsics;
	////opencv自带的标定是算出了cx,cy,fx,fy, k1, k2, k3,p1,p2
	//各个参数在Mat中的具体位置可以看opencv文档中关于calibrateCamera（）的介绍
	depthIntrinsics->parameters.param.fx = depthcameraMatrix.at<float>(0, 0);
	depthIntrinsics->parameters.param.cx = depthcameraMatrix.at<float>(0, 2);
	depthIntrinsics->parameters.param.fy = depthcameraMatrix.at<float>(1, 1);
	depthIntrinsics->parameters.param.cy = depthcameraMatrix.at<float>(1, 2);

	depthIntrinsics->parameters.param.k1 = depthdisCoeffs.at<float>(0);
	depthIntrinsics->parameters.param.k2 = depthdisCoeffs.at<float>(1);
	depthIntrinsics->parameters.param.p1 = depthdisCoeffs.at<float>(2);
	depthIntrinsics->parameters.param.p2 = depthdisCoeffs.at<float>(3);
	depthIntrinsics->parameters.param.k3 = depthdisCoeffs.at<float>(4);

	colorIntrinsics->parameters.param.fx = colorcameraMatrix.at<float>(0, 0);
	colorIntrinsics->parameters.param.cx = colorcameraMatrix.at<float>(0, 2);
	colorIntrinsics->parameters.param.fy = colorcameraMatrix.at<float>(1, 1);
	colorIntrinsics->parameters.param.cy = colorcameraMatrix.at<float>(1, 2);

	colorIntrinsics->parameters.param.k1 = colordisCoeffs.at<float>(0);
	colorIntrinsics->parameters.param.k2 = colordisCoeffs.at<float>(1);
	colorIntrinsics->parameters.param.p1 = colordisCoeffs.at<float>(2);
	colorIntrinsics->parameters.param.p2 = colordisCoeffs.at<float>(3);
	colorIntrinsics->parameters.param.k3 = colordisCoeffs.at<float>(4);
}
//得到Mat格式的相机内参
//cameraType是"depth"or"color"
//discoeffs:k1, k2, p1, p2, k3
void k4a::KinectAPI::GetIntrinsicParam(cv::Mat& cameraMatrix,cv::Mat &disCoeffs,const string cameraType)
{
	assert(cameraType == "depth" || cameraType == "color");

	k4a_calibration_camera_t CameraCalibration = 
		cameraType == "depth" ? calibration.depth_camera_calibration:calibration.color_camera_calibration;
	//kinect相机应该有4个传感器，下面的外参是4*4的一个矩阵，其中下标[i][j]应该是第i的传感器到第j个传感器的转化矩阵
	//[0][1]应该是depth到color的矩阵，但是没测试过，以后用到在测试。
	k4a_calibration_extrinsics_t extrinsics = calibration.extrinsics[0][0];
	//下面代码，当[0][0]时，输出分别为【1,0,0】和【0，0,0】就是本身到本身的转化矩阵
	//cout << extrinsics.rotation[0] << " " << extrinsics.rotation[1] << " " << extrinsics.rotation[2] << endl;
	//cout << extrinsics.translation[0] << " " << extrinsics.translation[1] << " " << extrinsics.translation[2] << endl;

	k4a_calibration_extrinsics_t Extrinsics= CameraCalibration.extrinsics;
	k4a_calibration_intrinsics_t Intrinsics= CameraCalibration.intrinsics;
	//cout << "depth resolution width,height:" << depthCameraCalibration.resolution_width 
	//	<< " " << depthCameraCalibration.resolution_height << endl;//640*576
	//测试得，下面rotation为单位阵，translation为0向量
	//cout << depthExtrinsics.rotation[0] << " " << depthExtrinsics.rotation[1] << " " << depthExtrinsics.rotation[2] << " "
	//	<< depthExtrinsics.rotation[3] << " " << depthExtrinsics.rotation[4] << " " << depthExtrinsics.rotation[5] << " "
	//	<< depthExtrinsics.rotation[6] << " " << depthExtrinsics.rotation[7] << " " << depthExtrinsics.rotation[8] << endl;
	//cout << depthExtrinsics.translation[0] << " " << depthExtrinsics.translation[1] << " " << depthExtrinsics.translation[2] << endl;

	//cout << "color resolution width,height:" << colorCameraCalibration.resolution_width 
	//	<< " " << colorCameraCalibration.resolution_height << endl;//1280*720
	//测试得，下面rotation和translation有数值, 等于上面[0][1]时候的数值，也就是深度相机在RGB相机下的位姿
	//0.999999 0.0015539 -8.61625e-05 -0.00153783 0.995126 0.098598 0.000238954 -0.0985978 0.995127
	//-31.9395 -1.91011 4.09401
	//cout << colorExtrinsics.rotation[0] << " " << colorExtrinsics.rotation[1] << " " << colorExtrinsics.rotation[2] << " "
	//	<< colorExtrinsics.rotation[3] << " " << colorExtrinsics.rotation[4] << " " << colorExtrinsics.rotation[5] << " "
	//	<< colorExtrinsics.rotation[6] << " " << colorExtrinsics.rotation[7] << " " << colorExtrinsics.rotation[8] << endl;
	//cout << colorExtrinsics.translation[0] << " " << colorExtrinsics.translation[1] << " " << colorExtrinsics.translation[2] << endl;

	////内参的排列顺序是：cx,cy,fx,fy,k1,k2,k3,k4,k5,k6,codx,cody,p2, p1, metic_radius 
	//cout << "depth camera intriParam count:" << depthIntrinsics.parameter_count << endl;//14
	//cout << "color camera intriParam count:" << colorIntrinsics.parameter_count << endl;//14
	////330.894 338.103 504.961 504.991 0.670492 0.0399937 -7.73619e-05 1.0134 0.189252 0.000535376 0 0 8.86593e-05 -8.28256e-05 0
	//cout << "depth intriParam:";
	//for (int i = 0; i < 15; i++)
	//	cout << depthIntrinsics.parameters.v[i] << " ";
	//cout << endl;
	////640.579 364.825 600.539 600.3 0.686572 -2.72132 1.53403 0.564911 -2.56053 1.46999 0 0 -0.000185118 0.000993749 0
	//cout << "color intriParam:";
	//for (int i = 0; i < 15; i++)
	//	cout << colorIntrinsics.parameters.v[i] << " ";
	//cout << endl;

	////opencv自带的标定是算出了cx,cy,fx,fy, k1, k2, k3,p1,p2
	//各个参数在Mat中的具体位置可以看opencv文档中关于calibrateCamera（）的介绍
	 cameraMatrix= cv::Mat(3, 3, CV_32FC1,cv::Scalar::all(0));
	 disCoeffs = cv::Mat(1, 5, CV_32FC1, cv::Scalar::all(0));
	 cameraMatrix.at<float>(0, 0) = Intrinsics.parameters.param.fx;
	 cameraMatrix.at<float>(0, 2) = Intrinsics.parameters.param.cx;
	 cameraMatrix.at<float>(1, 1) = Intrinsics.parameters.param.fy;
	 cameraMatrix.at<float>(1, 2) = Intrinsics.parameters.param.cy;
	 cameraMatrix.at<float>(2, 2) = 1;

	 disCoeffs.at<float>(0) = Intrinsics.parameters.param.k1;
	 disCoeffs.at<float>(1) = Intrinsics.parameters.param.k2;
	 disCoeffs.at<float>(2) = Intrinsics.parameters.param.p1;
	 disCoeffs.at<float>(3) = Intrinsics.parameters.param.p2;
	 disCoeffs.at<float>(4) = Intrinsics.parameters.param.k3;

}
void k4a::KinectAPI::GetRotationAndTranslationFromDepth2Color(cv::Mat& Depth2ColorRotation,cv::Mat &Depth2ColorTranslation)
{

	k4a_calibration_camera_t CameraCalibration = calibration.color_camera_calibration;

	k4a_calibration_extrinsics_t colorExtrinsics= CameraCalibration.extrinsics;

	//测试得，下面rotation和translation有数值, 等于上面[0][1]时候的数值，也就是深度相机在RGB相机下的位姿
	//0.999999 0.0015539 -8.61625e-05 -0.00153783 0.995126 0.098598 0.000238954 -0.0985978 0.995127
	//-31.9395 -1.91011 4.09401
	//cout << colorExtrinsics.rotation[0] << " " << colorExtrinsics.rotation[1] << " " << colorExtrinsics.rotation[2] << " "
	//	<< colorExtrinsics.rotation[3] << " " << colorExtrinsics.rotation[4] << " " << colorExtrinsics.rotation[5] << " "
	//	<< colorExtrinsics.rotation[6] << " " << colorExtrinsics.rotation[7] << " " << colorExtrinsics.rotation[8] << endl;
	//cout << colorExtrinsics.translation[0] << " " << colorExtrinsics.translation[1] << " " << colorExtrinsics.translation[2] << endl;

	Depth2ColorRotation= cv::Mat(3, 3, CV_32FC1,cv::Scalar::all(0));
	Depth2ColorTranslation= cv::Mat(3, 1, CV_32FC1,cv::Scalar::all(0));

	Depth2ColorRotation.at<float>(0, 0) = colorExtrinsics.rotation[0];
	Depth2ColorRotation.at<float>(0, 1) = colorExtrinsics.rotation[1];
	Depth2ColorRotation.at<float>(0, 2) = colorExtrinsics.rotation[2];
	Depth2ColorRotation.at<float>(1, 0) = colorExtrinsics.rotation[3];
	Depth2ColorRotation.at<float>(1, 1) = colorExtrinsics.rotation[4];
	Depth2ColorRotation.at<float>(1, 2) = colorExtrinsics.rotation[5];
	Depth2ColorRotation.at<float>(2, 0) = colorExtrinsics.rotation[6];
	Depth2ColorRotation.at<float>(2, 1) = colorExtrinsics.rotation[7];
	Depth2ColorRotation.at<float>(2, 2) = colorExtrinsics.rotation[8];

	Depth2ColorTranslation.at<float>(0, 0) = colorExtrinsics.translation[0];
	Depth2ColorTranslation.at<float>(1, 0) = colorExtrinsics.translation[1];
	Depth2ColorTranslation.at<float>(2, 0) = colorExtrinsics.translation[2];

}
void k4a::KinectAPI::ReleaseDevice() 
{

    k4a_device_stop_cameras(device);
    k4a_device_close(device);
}
//TODO:可选择
void k4a::KinectAPI::ShowOpenCVImage(cv::Mat Img, std::string name)
{
	cv::namedWindow("name", CV_WINDOW_NORMAL);  
	 cv::setWindowProperty("name", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
	cv::imshow("name", Img);
	cv::waitKey(0);
	cv::destroyAllWindows();
	//cv::waitKey(1);
	//cv::destroyAllWindows();
}
//depth已转到RGB相机视角
void k4a::KinectAPI::GetOpenCVImage(cv::Mat& colorMat, cv::Mat& depthMat, cv::Mat& depthcolorMat, cv::Mat& irMat, bool isDepth2Color)
{
	k4a_capture_t capture ;
	//TODO(zzq):这个capture是每调用一次捕捉一帧，还是之后可以一直通过get_color_image调用？
	switch (k4a_device_get_capture(device, &capture, 10000))
	{
		case K4A_WAIT_RESULT_SUCCEEDED:
			printf("get capture success\n");
			break;
		case K4A_WAIT_RESULT_TIMEOUT:
			throw "Timed out waiting for a capture";
		case K4A_WAIT_RESULT_FAILED:
			throw "Failed to read a capture";
	}

	k4a_image_t colorImage = k4a_capture_get_color_image(capture);
	int color_width = k4a_image_get_width_pixels(colorImage);
	int color_height = k4a_image_get_height_pixels(colorImage);

	k4a_image_t depthImageOld = k4a_capture_get_depth_image(capture);
	k4a_image_t depthImage = NULL;
	k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16,
		color_width,
		color_height,
		color_width * (int)sizeof(uint16_t),
		&depthImage);


	if (isDepth2Color == TRUE)
	{
		k4a_transformation_t transformation = k4a_transformation_create(&calibration);
		if (K4A_RESULT_SUCCEEDED == k4a_transformation_depth_image_to_color_camera(transformation, depthImageOld, depthImage))
		{
			printf(" | Depth16 res:%4dx%4d stride:%5d\n",
				k4a_image_get_height_pixels(depthImage),
				k4a_image_get_width_pixels(depthImage),
				k4a_image_get_stride_bytes(depthImage));
		}
		else throw "transform depth image failed!";
	}
	else depthImage = depthImageOld;
	k4a_image_t irImage = k4a_capture_get_ir_image(capture);

	if (colorImage != NULL)
	{
		printf(" | Color16 res:%4dx%4d stride:%5d\n",
			k4a_image_get_height_pixels(colorImage),
			k4a_image_get_width_pixels(colorImage),
			k4a_image_get_stride_bytes(colorImage));

	}
	else throw "Capture colorImage failed!";

	if (irImage!= NULL)
	{
		printf(" | Ir16 res:%4dx%4d stride:%5d\n",
			k4a_image_get_height_pixels(irImage),
			k4a_image_get_width_pixels(irImage),
			k4a_image_get_stride_bytes(irImage));

	}
	else throw "Capture irImage failed!";

	uint8_t* colorData = k4a_image_get_buffer(colorImage);
	//uint8_t* depthData = k4a_image_get_buffer(depthImage);
	uint16_t* depthData = reinterpret_cast<uint16_t*>(k4a_image_get_buffer(depthImage));
	uint16_t* irData = reinterpret_cast<uint16_t*>(k4a_image_get_buffer(irImage));
	std::vector<Pixel> depthTextureBuffer, irTextureBuffer;

	ColorizeDepthImage(depthImage, &KinectAPI::ColorizeDepthToRGB,
		GetDepthModeRange(config.depth_mode), &depthTextureBuffer);
	//ColorizeDepthImage(irImage, &Kinect_API::ColorizeGreyScale,
	//	GetIrLevels(K4A_DEPTH_MODE_PASSIVE_IR), &irTextureBuffer);

	cv::Mat depthcolorImg = cv::Mat(k4a_image_get_height_pixels(depthImage), k4a_image_get_width_pixels(depthImage),
		CV_8UC4, depthTextureBuffer.data());
	cv::Mat depthImg = cv::Mat(k4a_image_get_height_pixels(depthImage), k4a_image_get_width_pixels(depthImage),
		CV_16UC1, depthData);
	cv::Mat colorImg = cv::Mat(k4a_image_get_height_pixels(colorImage),
		k4a_image_get_width_pixels(colorImage), CV_8UC4, colorData);
	cv::Mat irImg = cv::Mat(k4a_image_get_height_pixels(irImage),
		k4a_image_get_width_pixels(irImage), CV_16UC1, irData);
	depthcolorMat = depthcolorImg.clone();
	depthMat = depthImg.clone();
	colorMat = colorImg.clone();
	irMat = irImg.clone();

	k4a_image_release(depthImage);
	k4a_image_release(colorImage);
	k4a_image_release(irImage);
	k4a_capture_release(capture);
}
k4a::Pixel  k4a::KinectAPI::ColorizeDepthToRGB(const DepthPixel& depthPixel,
	const DepthPixel& min,
	const DepthPixel& max) 
{
	const uint8_t PixelMax = std::numeric_limits<uint8_t>::max();
	Pixel result = { uint8_t(0),uint8_t(0),uint8_t(0), PixelMax};
	if (depthPixel == 0)
		return result;

	uint16_t clampedValue = depthPixel;
	clampedValue = std::min(clampedValue, max);
	clampedValue = std::max(clampedValue, min);
	
	float hue = (clampedValue - min) / static_cast<float>(max - min);

	const float range = 2.f / 3.f;
	hue *= range;
	hue = range - hue;

	float fRed = 0.f, fGreen = 0.f, fBlue = 0.f;
	ColorConvertHSVToRGB(hue, 1.f, 1.f, fRed, fGreen, fBlue);//HSI转化为RGB
	result.Red = static_cast<uint8_t>(fRed * PixelMax);
	result.Green = static_cast<uint8_t>(fGreen * PixelMax);
	result.Blue = static_cast<uint8_t>(fBlue * PixelMax);
	return result;
}
inline std::pair<uint16_t, uint16_t> k4a::KinectAPI::GetDepthModeRange(const k4a_depth_mode_t depthMode)
{
	//Kinect输出的深度值，单位为：mm。“近”的趋近于蓝色，“远”趋近于红色。不在这个范围的黑色
	switch (depthMode)
	{
	case K4A_DEPTH_MODE_NFOV_2X2BINNED:
		return { (uint16_t)500, (uint16_t)5800 };
	case K4A_DEPTH_MODE_NFOV_UNBINNED:
		return { (uint16_t)500, (uint16_t)4000 };
	case K4A_DEPTH_MODE_WFOV_2X2BINNED:
		return { (uint16_t)250, (uint16_t)3000 };
	case K4A_DEPTH_MODE_WFOV_UNBINNED:
		return { (uint16_t)250, (uint16_t)2500 };
	case K4A_DEPTH_MODE_PASSIVE_IR:
	default :
		throw "Invalid depth mode!";
	}
}
inline void k4a::KinectAPI::ColorConvertHSVToRGB(float h, float s, float v, float& out_r, float& out_g, float& out_b)
{
	if (s == 0.f)
	{
		out_r = out_g = out_b = v;
		return;
	}
	h = fmodf(h, 1.0f) / (60.f / 360.f);
	int i = (int)h;
	float f = h - (float)i;
	float p = v * (1.f - s);
	float q = v * (1.f - s * f);
	float t = v * (1.f - s * (1.f - f));
	switch (i)
	{
	case 0:out_r = v; out_g = t; out_b = p; break;
	case 1:out_r = q; out_g = v; out_b = p; break;
	case 2:out_r = p; out_g = v; out_b = t; break;
	case 3:out_r = p; out_g = q; out_b = v; break;
	case 4:out_r = t; out_g = p; out_b = v; break;
	default: out_r = v; out_g = p; out_b = q;
		break;
	}

}
k4a::Pixel k4a::KinectAPI::ColorizeGreyScale(const DepthPixel& value,
	const DepthPixel& min,
	const DepthPixel& max)
{
	DepthPixel pixelValue = std::min(value, max);

	const uint8_t PixelMax = std::numeric_limits<uint8_t>::max();
	const auto normalizedValue = static_cast<uint8_t>((pixelValue - min) * (double(PixelMax) / (max - min)));

	return Pixel{ normalizedValue, normalizedValue, normalizedValue, PixelMax };
}
std::pair<uint16_t, uint16_t> k4a::KinectAPI::GetIrLevels(const k4a_depth_mode_t depthMode)
{
	switch (depthMode)
	{
	case K4A_DEPTH_MODE_PASSIVE_IR:
		printf("IrRange:%d,%d\n", 0, 100);
		return { (uint16_t)0, (uint16_t)100 };

	case K4A_DEPTH_MODE_OFF:
		throw "Invalid depth mode!";
	default:
		printf("IrRange:%d,%d\n", 0, 1000);
		return { (uint16_t)0, (uint16_t)1000 };
	}
}
void k4a::KinectAPI::ColorizeDepthImage(const k4a_image_t& depthImage,
	Pixel(KinectAPI::*visualizationFn)(const DepthPixel&, const DepthPixel&, const DepthPixel&),
	std::pair<uint16_t, uint16_t> expectedValueRange,
	std::vector<Pixel>* buffer)
{
	const k4a_image_format_t imageFormat = k4a_image_get_format(depthImage);
	if (imageFormat != K4A_IMAGE_FORMAT_DEPTH16 && imageFormat != K4A_IMAGE_FORMAT_IR16)
		throw "Attemped to colorize a non-depth image!";

	const int width = k4a_image_get_width_pixels(depthImage);
	const int height = k4a_image_get_height_pixels(depthImage);

	buffer->resize(static_cast<size_t>(width * height));

	const uint16_t* depthData = reinterpret_cast<const uint16_t*>(k4a_image_get_buffer(depthImage));

	for(int h = 0;h < height; h++)
		for (int w = 0; w < width; w++)
		{
			const size_t currentPixel = static_cast<size_t>(h * width + w);
			(*buffer)[currentPixel] = (this->*visualizationFn)(depthData[currentPixel], expectedValueRange.first, expectedValueRange.second);
		}
}
void k4a::KinectAPI::GetPointCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud)
{
		k4a_capture_t capture ;
		switch (k4a_device_get_capture(device, &capture, 10000))
		{
			case K4A_WAIT_RESULT_SUCCEEDED:
				printf("get capture success\n");
				break;
			case K4A_WAIT_RESULT_TIMEOUT:
				throw "Timed out waiting for a capture";
			case K4A_WAIT_RESULT_FAILED:
				throw "Failed to read a capture";
		}

		k4a_image_t colorImage = k4a_capture_get_color_image(capture);
		k4a_image_t depthImage= k4a_capture_get_depth_image(capture);

		int color_image_width_pixels = k4a_image_get_width_pixels(colorImage);
		int color_image_height_pixels = k4a_image_get_height_pixels(colorImage);

		k4a_image_t transformed_depth_image= NULL;
		k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16,
			color_image_width_pixels,
			color_image_height_pixels,
			color_image_width_pixels * (int)sizeof(uint16_t),
			&transformed_depth_image);

		k4a_image_t point_cloud_image= NULL;
		k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16,
			color_image_width_pixels,
			color_image_height_pixels,
			color_image_width_pixels * 3 * (int)sizeof(uint16_t),
			&point_cloud_image);

		k4a_calibration_t calibration;
		if (K4A_RESULT_SUCCEEDED !=
			k4a_device_get_calibration(device, config.depth_mode, config.color_resolution, &calibration))
		{
			throw ("Failed to get calibration\n");
		}

		k4a_transformation_t transformation = k4a_transformation_create(&calibration);

		k4a_transformation_depth_image_to_color_camera(transformation, depthImage, transformed_depth_image);
		k4a_transformation_depth_image_to_point_cloud(transformation, transformed_depth_image, K4A_CALIBRATION_TYPE_COLOR, point_cloud_image);

		int width =k4a_image_get_width_pixels(colorImage);
		int height =	k4a_image_get_height_pixels(colorImage);

		cloud->width = width;
		cloud->height = height;
		cloud->is_dense = false;
		cloud->points.resize(cloud->height * cloud->width);

	//uint8_t* colorData = k4a_image_get_buffer(colorImage);
		int16_t* point_cloud_image_data = (int16_t*)(void*)k4a_image_get_buffer(point_cloud_image);
		uint8_t *color_image_data = k4a_image_get_buffer(colorImage);

	#ifdef VTK_VISUALIZATION
		Eigen::Matrix3f mZ,mY;
		mZ = Eigen::AngleAxisf(-M_PI, Eigen::Vector3f::UnitZ());
		mY = Eigen::AngleAxisf(-M_PI, Eigen::Vector3f::UnitY());
	#endif

		for (int i = 0; i < width * height; ++i)
		{
			pcl::PointXYZRGBA point;

			point.x = point_cloud_image_data[3 * i + 0] / 1000.0f;
			point.y = point_cloud_image_data[3 * i + 1] / 1000.0f;
			point.z = point_cloud_image_data[3 * i + 2] / 1000.0f;
			//printf("%f ", point.z);

			if (point.z == 0)
			{
				continue;
			}

	#ifdef VTK_VISUALIZATION
			point.getVector3fMap() = mZ * point.getVector3fMap();
			point.getVector3fMap() = mY * point.getVector3fMap();
	#endif

			point.b = color_image_data[4 * i + 0];
			point.g = color_image_data[4 * i + 1];
			point.r = color_image_data[4 * i + 2];
			point.a = color_image_data[4 * i + 3];

			if (point.b == 0 && point.g == 0 && point.r == 0 && point.a == 0)
			{
				continue;
			}

			cloud->points[i] = point;
		}
		k4a_image_release(point_cloud_image);
		k4a_image_release(transformed_depth_image);
		k4a_image_release(depthImage);
		k4a_image_release(colorImage);
		//k4a_image_release(irImage);
		k4a_capture_release(capture);
}
void k4a::KinectAPI::GetXYZAtCameraView(const cv::Point2i point2D, float depth, cv::Point3f& point3D)
{
	//OpenCV类型的变量转化为kinect形式
	k4a_float2_t point2d;
	point2d.v[0] = point2D.x;
	point2d.v[1] = point2D.y;
	point2d.xy.x = point2D.x;
	point2d.xy.y = point2D.y;

	k4a_float3_t point3d;

	int valid = 0;
	//第一个K4A_CALIBRATION_TYPE_COLOR值输入的2D坐标是哪个相机图像的。第二个是指要转化到哪个相机视角下的3D坐标
	k4a_result_t result = k4a_calibration_2d_to_3d(
		&calibration, &point2d, depth, K4A_CALIBRATION_TYPE_COLOR, K4A_CALIBRATION_TYPE_COLOR, &point3d, &valid);
	if (K4A_RESULT_SUCCEEDED != result)
	{
		throw error("Calibration contained invalid transformation parameters!");
	}

	//将3D坐标从kinect格式转化为OpenCV格式
	point3D.x = point3d.xyz.x;
	point3D.y = point3d.xyz.y;
	point3D.z = point3d.xyz.z;
}
