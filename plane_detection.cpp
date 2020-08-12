#include "plane_detection.h"
#include <stdint.h>
#include <iomanip> // output double value precision

cv::Mat processImg(cv::Mat colorSrc, cv::Mat depthSrc, double*& center, 
	double*& normal, float& minAreaRectAngle, vector<VertexType>& highestPlanePoints_3D, cv::Point2f * vertices) {
	int start_x = 236;
	int start_y = 0;
	int roi_width = 280;
	int roi_height = 360;
	cv::Rect roi = cv::Rect(start_x, start_y, roi_width, roi_height);

	PlaneDetection plane_detection;
	plane_detection.readDepthImage(depthSrc, roi);
	plane_detection.readColorImage(colorSrc, roi);

	plane_detection.runPlaneDetection(vertices);

	center = new double[3];
	normal = new double[3];

	int highestPlaneId = plane_detection.plane_filter.highestPlane.first;

	center[0] = plane_detection.plane_filter.extractedPlanes[highestPlaneId]->center[0];
	center[1] = plane_detection.plane_filter.extractedPlanes[highestPlaneId]->center[1];
	center[2] = plane_detection.plane_filter.extractedPlanes[highestPlaneId]->center[2];

	normal[0] = plane_detection.plane_filter.extractedPlanes[highestPlaneId]->normal[0];
	normal[1] = plane_detection.plane_filter.extractedPlanes[highestPlaneId]->normal[1];
	normal[2] = plane_detection.plane_filter.extractedPlanes[highestPlaneId]->normal[2];

	minAreaRectAngle = plane_detection.plane_filter.rects[highestPlaneId].angle;

	vector<pair<int, int>> highestPlanePoints = plane_detection.plane_filter.highestPlanePoints;
	double x = 0, y = 0, z = 0;
	for (int i = 0; i < highestPlanePoints.size(); ++i) {
		plane_detection.cloud.get(highestPlanePoints[i].first, highestPlanePoints[i].second, x, y, z);
		if(((x - center[0])*(x - center[0])+(y - center[1])*(y - center[1])+(z - center[2])*(z - center[2]))<900)
		highestPlanePoints_3D.push_back(VertexType(x, y, z));
	}

	return plane_detection.color_img_;
}

PlaneDetection::PlaneDetection()
{
	cloud.vertices.resize(kDepthHeight * kDepthWidth);
	cloud.w = kDepthWidth;
	cloud.h = kDepthHeight;
}

PlaneDetection::~PlaneDetection()
{
	cloud.vertices.clear();
	seg_img_.release();
	color_img_.release();
	opt_membership_img_.release();
	pixel_boundary_flags_.clear();
	pixel_grayval_.clear();
	plane_colors_.clear();
	plane_pixel_nums_.clear();
	sum_stats_.clear();
}

// Temporarily don't need it since we set intrinsic parameters as constant values in the code.
//bool PlaneDetection::readIntrinsicParameterFile(string filename)
//{
//	ifstream readin(filename, ios::in);
//	if (readin.fail() || readin.eof())
//	{
//		cout << "WARNING: Cannot read intrinsics file " << filename << endl;
//		return false;
//	}
//	string target_str = "m_calibrationDepthIntrinsic";
//	string str_line, str, str_dummy;
//	double dummy;
//	bool read_success = false;
//	while (!readin.eof() && !readin.fail())
//	{
//		getline(readin, str_line);
//		if (readin.eof())
//			break;
//		istringstream iss(str_line);
//		iss >> str;
//		if (str == "m_depthWidth")
//			iss >> str_dummy >> width_;
//		else if (str == "m_depthHeight")
//			iss >> str_dummy >> height_;
//		else if (str == "m_calibrationDepthIntrinsic")
//		{
//			iss >> str_dummy >> fx_ >> dummy >> cx_ >> dummy >> dummy >> fy_ >> cy_;
//			read_success = true;
//			break;
//		}
//	}
//	readin.close();
//	if (read_success)
//	{
//		cloud.vertices.resize(height_ * width_);
//		cloud.w = width_;
//		cloud.h = height_;
//	}
//	return read_success;
//}

bool PlaneDetection::readColorImage(cv::Mat src, cv::Rect roi)
{
	src.copyTo(color_img_);
	if (color_img_.empty() || color_img_.depth() != CV_8U)
	{
		cout << color_img_.empty() << "\t" << color_img_.depth() << endl;
		cout << "ERROR: cannot read color image. No such a file, or the image format is not 8UC3" << endl;
		return false;
	}
	return true;
}

bool PlaneDetection::readDepthImage(cv::Mat src, cv::Rect roi)
{
	cv::Mat org_depth_img;
	src.copyTo(org_depth_img);
	if (org_depth_img.empty() || org_depth_img.depth() != CV_16U)
	{
		cout << org_depth_img.empty() << "\t" << org_depth_img.depth() << endl;
		cout << "WARNING: cannot read depth image. No such a file, or the image format is not 16UC1" << endl;
		return false;
	}
	cv::Mat mask = cv::Mat::zeros(kDepthHeight, kDepthWidth, CV_8UC1);
	mask(roi).setTo(255);

	cv::Mat depth_img;
	org_depth_img.copyTo(depth_img, mask);
	int rows = depth_img.rows, cols = depth_img.cols;
	int vertex_idx = 0;
	for (int i = 0; i < rows; ++i)
	{
		for (int j = 0; j < cols; ++j)
		{
			double z = (double)(depth_img.at<unsigned short>(i, j)) / kScaleFactor;
			if (_isnan(z))
			{
				cloud.vertices[vertex_idx++] = VertexType(0, 0, z);
				continue;
			}
			double x = ((double)j - kCx) * z / kFx;
			double y = ((double)i - kCy) * z / kFy;
			cloud.vertices[vertex_idx++] = VertexType(x, y, z);
		}
	}
	return true;
}

bool PlaneDetection::runPlaneDetection(cv::Point2f * vertices)
{
	seg_img_ = cv::Mat(kDepthHeight, kDepthWidth, CV_8UC3);
	plane_filter.run(&cloud, &plane_vertices_, &seg_img_);
	plane_num_ = (int)plane_vertices_.size();
	vector<cv::RotatedRect> rectangles= plane_filter.rects;
	
	for (int i = 0; i < rectangles.size(); ++i) {
		if (i == plane_filter.lowestPlane.first) continue;
		if (i == plane_filter.highestPlane.first) continue;
		if (rectangles[i].size.area() < 500) continue;
		/*string center = "";
		for (int j = 0; j < 3; ++j)
			center += to_string(plane_filter.extractedPlanes[i]->center[j]) + " ";*/

		//cv::Point2f vertices[4];
		rectangles[i].points(vertices);
		for (int j = 0; j < 4; ++j) {
			line(color_img_, vertices[j], vertices[(j + 1) % 4], cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
		}
		//cv::putText(color_img_, center, cv::Point(vertices[1].x, vertices[1].y - 10), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
	}

	//draw highest plane last
	/*string center = "(";
	for (int j = 0; j < 2; ++j)
		center += to_string((int)plane_filter.extractedPlanes[plane_filter.highestPlane.first]->center[j]) + ", ";
	center += to_string((int)plane_filter.extractedPlanes[plane_filter.highestPlane.first]->center[2]) + ")";*/
	//cv::Point2f vertices[4];
	rectangles[plane_filter.highestPlane.first].points(vertices);
	for (int j = 0; j < 4; ++j) {
		line(color_img_, vertices[j], vertices[(j + 1) % 4], cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
	}
	//cv::putText(color_img_, center, cv::Point(rectangles[plane_filter.highestPlane.first].center.x, rectangles[plane_filter.highestPlane.first].center.y), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
	

	// Here we set the plane index of a pixel which does NOT belong to any plane as #planes.
	// This is for using MRF optimization later.
	/*for (int row = 0; row < kDepthHeight; ++row)
		for (int col = 0; col < kDepthWidth; ++col)
			if (plane_filter.membershipImg.at<int>(row, col) < 0)
				plane_filter.membershipImg.at<int>(row, col) = plane_num_;*/
	return true;
}



// Note: input filename_prefix is like '/rgbd-image-folder-path/frame-XXXXXX'
void PlaneDetection::writeOutputFiles(string output_folder, string frame_name)
{

	if (output_folder.back() != '\\' && output_folder.back() != '/')
		output_folder += "/";	
	string filename_prefix = output_folder + frame_name + "-plane";
	cv::imwrite(filename_prefix + ".png", color_img_);
	//writePlaneLabelFile(filename_prefix + "-label.txt");
	//writePlaneDataFile(filename_prefix + "-data.txt");
	
}
void PlaneDetection::writePlaneLabelFile(string filename)
{
	ofstream out(filename, ios::out);
	out << plane_num_ << endl;
	if (plane_num_ == 0)
	{
		out.close();
		return;
	}
	for (int row = 0; row < kDepthHeight; ++row)
	{
		for (int col = 0; col < kDepthWidth; ++col)
		{
			int label = plane_filter.membershipImg.at<int>(row, col);
			out << label << " ";
		}
		out << endl;
	}
	out.close();
}

void PlaneDetection::writePlaneDataFile(string filename)
{
	computePlaneSumStats();
	ofstream out(filename, ios::out);
	out << "#plane_index number_of_points_on_the_plane plane_color_in_png_image(1x3) plane_normal(1x3) plane_center(1x3) "
		<< "sx sy sz sxx syy szz sxy syz sxz" << endl;

	for (int pidx = 0; pidx < plane_num_; ++pidx)
	{
		out << pidx << " ";
		out << plane_pixel_nums_[pidx] << " ";

		// Plane color in output image
		int vidx = plane_vertices_[pidx][0];
		cv::Vec3b c = seg_img_.at<cv::Vec3b>(vidx / kDepthWidth, vidx % kDepthWidth);
		out << int(c.val[2]) << " " << int(c.val[1]) << " "<< int(c.val[0]) << " "; // OpenCV uses BGR by default

		// Plane normal and center
		int new_pidx = pid_to_extractedpid[pidx];
		for (int i = 0; i < 3; ++i)
			out << plane_filter.extractedPlanes[new_pidx]->normal[i] << " ";
		for (int i = 0; i < 3; ++i)
			out << plane_filter.extractedPlanes[new_pidx]->center[i] << " ";

		// Sum of all points on the plane
		out << sum_stats_[pidx].sx << std::setprecision(8) << " " 
		<< sum_stats_[pidx].sy << std::setprecision(8) << " " 
				<< sum_stats_[pidx].sz << std::setprecision(8) << " " 
				<< sum_stats_[pidx].sxx << std::setprecision(8) << " "
				<< sum_stats_[pidx].syy << std::setprecision(8) << " "
				<< sum_stats_[pidx].szz << std::setprecision(8) << " "
				<< sum_stats_[pidx].sxy << std::setprecision(8) << " "
				<< sum_stats_[pidx].syz << std::setprecision(8) << " "
				<< sum_stats_[pidx].sxz << std::setprecision(8) << endl;

		// NOTE: the plane-sum parameters computed from AHC code seems different from that computed from points belonging to planes shown above.
		// Seems there is a plane refinement step in AHC code so points belonging to each plane are slightly changed.
		//ahc::PlaneSeg::Stats& stat = plane_filter.extractedPlanes[pidx]->stats;
		//cout << stat.sx << " " << stat.sy << " " << stat.sz << " " << stat.sxx << " "<< stat.syy << " "<< stat.szz << " "<< stat.sxy << " "<< stat.syz << " "<< stat.sxz << endl;
	}
	out.close();
}

void PlaneDetection::computePlaneSumStats()
{
	sum_stats_.resize(plane_num_);
	for (int pidx = 0; pidx < plane_num_; ++pidx)
	{
		for (int i = 0; i < plane_vertices_[pidx].size(); ++i)
		{
			int vidx = plane_vertices_[pidx][i];
			const VertexType& v = cloud.vertices[vidx];
			sum_stats_[pidx].sx += v[0];		 sum_stats_[pidx].sy += v[1];		  sum_stats_[pidx].sz += v[2];
			sum_stats_[pidx].sxx += v[0] * v[0]; sum_stats_[pidx].syy += v[1] * v[1]; sum_stats_[pidx].szz += v[2] * v[2];
			sum_stats_[pidx].sxy += v[0] * v[1]; sum_stats_[pidx].syz += v[1] * v[2]; sum_stats_[pidx].sxz += v[0] * v[2];
		}
		plane_pixel_nums_.push_back(int(plane_vertices_[pidx].size()));
	}
	for (int pidx = 0; pidx < plane_num_; ++pidx)
	{
		int num = plane_pixel_nums_[pidx];
		sum_stats_[pidx].sx /= num;		sum_stats_[pidx].sy /= num;		sum_stats_[pidx].sz /= num;
		sum_stats_[pidx].sxx /= num;	sum_stats_[pidx].syy /= num;	sum_stats_[pidx].szz /= num;
		sum_stats_[pidx].sxy /= num;	sum_stats_[pidx].syz /= num;	sum_stats_[pidx].sxz /= num;
	}
	// Note that the order of extracted planes in `plane_filter.extractedPlanes` is DIFFERENT from
	// the plane order in `plane_vertices_` after running plane detection function `plane_filter.run()`.
	// So here we compute a mapping between these two types of plane indices by comparing plane centers.
	vector<double> sx(plane_num_), sy(plane_num_), sz(plane_num_);
	for (int i = 0; i < plane_filter.extractedPlanes.size(); ++i)
	{
		sx[i] = plane_filter.extractedPlanes[i]->stats.sx / plane_filter.extractedPlanes[i]->stats.N;
		sy[i] = plane_filter.extractedPlanes[i]->stats.sy / plane_filter.extractedPlanes[i]->stats.N;
		sz[i] = plane_filter.extractedPlanes[i]->stats.sz / plane_filter.extractedPlanes[i]->stats.N;
	}
	extractedpid_to_pid.clear();
	pid_to_extractedpid.clear();
	// If two planes' centers are closest, then the two planes are corresponding to each other.
	for (int i = 0; i < plane_num_; ++i)
	{
		double min_dis = 1000000;
		int min_idx = -1;
		for (int j = 0; j < plane_num_; ++j)
		{
			double a = sum_stats_[i].sx - sx[j], b = sum_stats_[i].sy - sy[j], c = sum_stats_[i].sz - sz[j];
			double dis = a * a + b * b + c * c;
			if (dis < min_dis)
			{
				min_dis = dis;
				min_idx = j;
			}
		}
		if (extractedpid_to_pid.find(min_idx) != extractedpid_to_pid.end())
		{
			cout << "   WARNING: a mapping already exists for extracted plane " << min_idx << ":" << extractedpid_to_pid[min_idx] << " -> " << min_idx << endl;
		}
		pid_to_extractedpid[i] = min_idx;
		extractedpid_to_pid[min_idx] = i;
	}
	
	//--------------------------------------------------------------
	// Only for debug. It doesn't influence the plane detection.
	/*for (int pidx = 0; pidx < plane_num_; ++pidx)
	{
		double w = 0;
		//for (int j = 0; j < 3; ++j)
		//	w -= plane_filter.extractedPlanes[pidx]->normal[j] * plane_filter.extractedPlanes[pidx]->center[j];
		w -= plane_filter.extractedPlanes[pidx]->normal[0] * sum_stats_[pidx].sx;
		w -= plane_filter.extractedPlanes[pidx]->normal[1] * sum_stats_[pidx].sy;
		w -= plane_filter.extractedPlanes[pidx]->normal[2] * sum_stats_[pidx].sz;
		double sum = 0;
		for (int i = 0; i < plane_vertices_[pidx].size(); ++i)
		{
			int vidx = plane_vertices_[pidx][i];
			const VertexType& v = cloud.vertices[vidx];
			double dis = w;
			for (int j = 0; j < 3; ++j)
				dis += v[j] * plane_filter.extractedPlanes[pidx]->normal[j];
			sum += dis * dis;
		}
		sum /= plane_vertices_[pidx].size();
		cout << "Distance for plane " << pidx << ": " << sum << endl;
	}*/
}
