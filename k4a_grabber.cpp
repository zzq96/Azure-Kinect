#include <Eigen/Dense>

#include "k4a_grabber.h"

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
}
void k4a::KinectAPI::ReleaseDevice() 
{

    k4a_device_stop_cameras(device);
    k4a_device_close(device);
}
void k4a::KinectAPI::ShowOpenCVImage(cv::Mat Img, std::string name)
{

	cv::namedWindow("name", CV_WINDOW_NORMAL);  
	cv::imshow("name", Img);
	cv::waitKey(0);
	cv::destroyAllWindows();
}
//depth已转到RGB相机视角
void k4a::KinectAPI::GetOpenCVImage(cv::Mat& colorMat, cv::Mat& depthMat, cv::Mat& depthcolorMat)
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
	int color_width = k4a_image_get_width_pixels(colorImage);
	int color_height = k4a_image_get_height_pixels(colorImage);

	k4a_image_t depthImageOld = k4a_capture_get_depth_image(capture);
	k4a_image_t depthImage = NULL;
	k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16,
		color_width,
		color_height,
		color_width * (int)sizeof(uint16_t),
		&depthImage);

	k4a_calibration_t calibration;
	if (K4A_RESULT_SUCCEEDED !=
		k4a_device_get_calibration(device, config.depth_mode, config.color_resolution, &calibration))
	{
		throw ("Failed to get calibration\n");
	}

	k4a_transformation_t transformation = k4a_transformation_create(&calibration);

	if (K4A_RESULT_SUCCEEDED == k4a_transformation_depth_image_to_color_camera(transformation, depthImageOld, depthImage))
	{
		printf(" | Depth16 res:%4dx%4d stride:%5d\n",
			k4a_image_get_height_pixels(depthImage),
			k4a_image_get_width_pixels(depthImage),
			k4a_image_get_stride_bytes(depthImage));
	}
	else throw "transform depth image failed!";
	//k4a_image_t irImage = k4a_capture_get_ir_image(capture);

	if (colorImage != NULL)
	{
		printf(" | Color16 res:%4dx%4d stride:%5d\n",
			k4a_image_get_height_pixels(colorImage),
			k4a_image_get_width_pixels(colorImage),
			k4a_image_get_stride_bytes(colorImage));

	}
	else throw "Capture colorImage failed!";

	//if (irImage!= NULL)
	//{
	//	printf(" | Ir16 res:%4dx%4d stride:%5d\n",
	//		k4a_image_get_height_pixels(irImage),
	//		k4a_image_get_width_pixels(irImage),
	//		k4a_image_get_stride_bytes(irImage));

	//}
	//else throw "Capture irImage failed!";

	uint8_t* colorData = k4a_image_get_buffer(colorImage);
	//uint8_t* depthData = k4a_image_get_buffer(depthImage);
	uint16_t* depthData = reinterpret_cast<uint16_t*>(k4a_image_get_buffer(depthImage));
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
	//cv::Mat irImg = cv::Mat(k4a_image_get_height_pixels(irImage),
	//	k4a_image_get_width_pixels(irImage), CV_8UC4, irTextureBuffer.data());
	depthcolorMat = depthcolorImg.clone();
	depthMat = depthImg.clone();
	colorMat = colorImg.clone();
	//irMat = irImg.clone();

	k4a_image_release(depthImage);
	k4a_image_release(colorImage);
	//k4a_image_release(irImage);
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
