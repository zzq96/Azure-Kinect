#include "plane_detection.h"
#include "Robot.h"
#include <stdint.h>
#include <iomanip> // output double value precision

extern Robot rob;

void copyVec3(double* vec0, double* vec1) {
	vec0[0] = vec1[0];
	vec0[1] = vec1[1];
	vec0[2] = vec1[2];
}

cv::Mat processImg(cv::Mat colorSrc, cv::Mat depthSrc, vector<cv::Mat>& masks, double* center, 
	double* x_axis, double* y_axis, double* z_axis, vector<VertexType>& highestPlanePoints, cv::Point2f* vertices) {
	int start_x = 200;
	int start_y = 40;
	int roi_width = 215;
	int roi_height = 290;
	cv::Rect roi = cv::Rect(start_x, start_y, roi_width, roi_height);
	cv::Mat mask = cv::Mat::zeros(kDepthHeight, kDepthWidth, CV_8UC1);
	mask(roi).setTo(255);
	cv::Mat depth;
	depthSrc.copyTo(depth, mask);

	int size = masks.size();
	PlaneDetection* plane_detections = new PlaneDetection[size];
	int highest_index = -1, highest_index_index = -1, topRect = -1, k = 0;
	double highestPlaneHeight = -1;
	std::vector<cv::RotatedRect> boundingRects;
	cv::Mat point3d;
	for (int i = 0; i < size; ++i) {
		plane_detections[i].readDepthImage(depth, masks[i]);
		
		plane_detections[i].runPlaneDetection();
		for (int j = 0; j < plane_detections[i].plane_filter.extractedPlanes.size(); ++j, ++k) {
			//imshow(to_string(cv::getTickCount()), plane_detections[i].plane_filter.planes[j]);
			boundingRects.push_back(getBoundingRect(plane_detections[i].plane_filter.planes[j]));
			point3d = (cv::Mat_<double>(4,1)<<plane_detections[i].plane_filter.extractedPlanes[j]->center[0],
				plane_detections[i].plane_filter.extractedPlanes[j]->center[1],
				plane_detections[i].plane_filter.extractedPlanes[j]->center[2],1);
			//������������ϵ�µ�����ת��Ϊ��е������ϵ
			point3d = rob.depth_Homo_cam2base * point3d;

			//ѡ����ߵĿ��
			if (point3d.at<double>(2, 0) > highestPlaneHeight) {
				highestPlaneHeight = point3d.at<double>(2, 0);
				highest_index = i;
				highest_index_index = j;
				topRect = k;
			}
		}
	}
	
	if (highest_index != -1) {
		copyVec3(center, plane_detections[highest_index].plane_filter.extractedPlanes[highest_index_index]->center);
		copyVec3(x_axis, plane_detections[highest_index].plane_filter.extractedPlanes[highest_index_index]->x_axis);
		copyVec3(y_axis, plane_detections[highest_index].plane_filter.extractedPlanes[highest_index_index]->y_axis);
		copyVec3(z_axis, plane_detections[highest_index].plane_filter.extractedPlanes[highest_index_index]->z_axis);

		double x = -1, y = -1, z = -1;
		for (int i = 0; i < colorSrc.rows; ++i) {
			for (int j = 0; j < colorSrc.cols; ++j) {
				if (plane_detections[highest_index].plane_filter.planes[highest_index_index].at<uchar>(i, j) == 255) {
					plane_detections[highest_index].cloud.get(i, j, x, y, z);
					highestPlanePoints.push_back(VertexType(x, y, z));
				}
			}			
		}

		for (int i = 0; i < boundingRects.size(); ++i) {
			if (i == topRect) continue;
			boundingRects[i].points(vertices);
			for (int j = 0; j < 4; ++j) {
				line(colorSrc, vertices[j], vertices[(j + 1) % 4], cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
			}
		}

		boundingRects[topRect].points(vertices);
		for (int j = 0; j < 4; ++j) {
			line(colorSrc, vertices[j], vertices[(j + 1) % 4], cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
		}
	}
	else {
		center = new double[3];
	}

	delete[] plane_detections;

	return colorSrc;
}

cv::RotatedRect getBoundingRect(cv::Mat src) {
	cv::Mat element = getStructuringElement(0,
		cv::Size(5, 5),
		cv::Point(-1, -1));
	//erode(src, src, cv::Mat(), cv::Point(-1, -1));
	//dilate(src, src, element, cv::Point(-1, -1));
	threshold(src, src, 0, 255, cv::THRESH_BINARY);
	//imshow(to_string(cv::getTickCount()), src);

	std::vector<std::vector<cv::Point>> contours;
	findContours(src, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE, cv::Point());
	int max = 0, index = 0;
	for (int i = 0; i < contours.size(); ++i) {
		if (contours[i].size() > max) {
			max = contours[i].size();
			index = i;
		}
	}
	cv::RotatedRect minRect;
	minRect = cv::minAreaRect(cv::Mat(contours[index]));
	return minRect;
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
	opt_seg_img_.release();
	color_img_.release();
	opt_membership_img_.release();
	pixel_boundary_flags_.clear();
	pixel_grayval_.clear();
	plane_colors_.clear();
	plane_pixel_nums_.clear();
	opt_plane_pixel_nums_.clear();
	sum_stats_.clear();
	opt_sum_stats_.clear();
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

bool PlaneDetection::readColorImage(cv::Mat src)
{
	color_img_ = src.clone();

	if (color_img_.empty() || color_img_.depth() != CV_8U)
	{
		cout << color_img_.empty() << "\t" << color_img_.depth() << endl;
		cout << "ERROR: cannot read color image. No such a file, or the image format is not 8UC3" << endl;
		return false;
	}
	return true;
}

bool PlaneDetection::readDepthImage(cv::Mat src, cv::Mat mask)
{
	cv::Mat depth_img;
	src.copyTo(depth_img, mask);
	if (depth_img.empty() || depth_img.depth() != CV_16U)
	{
		cout << depth_img.empty() << "\t" << depth_img.depth() << endl;
		cout << "WARNING: cannot read depth image. No such a file, or the image format is not 16UC1" << endl;
		return false;
	}
	
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

bool PlaneDetection::runPlaneDetection()
{
	seg_img_ = cv::Mat(kDepthHeight, kDepthWidth, CV_8UC3);
	//ahc::utils::Timer timer(1000);
	//timer.tic();
	plane_filter.run(&cloud, &plane_vertices_, &seg_img_);
	//timer.toctic("Total time");
	plane_num_ = (int)plane_vertices_.size();

	// Here we set the plane index of a pixel which does NOT belong to any plane as #planes.
	// This is for using MRF optimization later.
	for (int row = 0; row < kDepthHeight; ++row)
		for (int col = 0; col < kDepthWidth; ++col)
			if (plane_filter.membershipImg.at<int>(row, col) < 0)
				plane_filter.membershipImg.at<int>(row, col) = plane_num_;
	return true;
}

void PlaneDetection::prepareForMRF()
{
	opt_seg_img_ = cv::Mat(kDepthHeight, kDepthWidth, CV_8UC3);
	opt_membership_img_ = cv::Mat(kDepthHeight, kDepthWidth, CV_32SC1);
	pixel_boundary_flags_.resize(kDepthWidth * kDepthHeight, false);
	pixel_grayval_.resize(kDepthWidth * kDepthHeight, 0);

	cv::Mat& mat_label = plane_filter.membershipImg;
	for (int row = 0; row < kDepthHeight; ++row)
	{
		for (int col = 0; col < kDepthWidth; ++col)
		{
			pixel_grayval_[row * kDepthWidth + col] = RGB2Gray(row, col);
			int label = mat_label.at<int>(row, col);
			if ((row - 1 >= 0 && mat_label.at<int>(row - 1, col) != label)
				|| (row + 1 < kDepthHeight && mat_label.at<int>(row + 1, col) != label)
				|| (col - 1 >= 0 && mat_label.at<int>(row, col - 1) != label)
				|| (col + 1 < kDepthWidth && mat_label.at<int>(row, col + 1) != label))
			{
				// Pixels in a fixed range near the boundary pixel are also regarded as boundary pixels
				for (int x = max(row - kNeighborRange, 0); x < min(kDepthHeight, row + kNeighborRange); ++x)
				{
					for (int y = max(col - kNeighborRange, 0); y < min(kDepthWidth, col + kNeighborRange); ++y)
					{
						// If a pixel is not on any plane, then it is not a boundary pixel.
						if (mat_label.at<int>(x, y) == plane_num_)
							continue;
						pixel_boundary_flags_[x * kDepthWidth + y] = true;
					}
				}
			}
		}
	}

	for (int pidx = 0; pidx < plane_num_; ++pidx)
	{
		int vidx = plane_vertices_[pidx][0];
		cv::Vec3b c = seg_img_.at<cv::Vec3b>(vidx / kDepthWidth, vidx % kDepthWidth);
		plane_colors_.push_back(c);
	}
	plane_colors_.push_back(cv::Vec3b(0,0,0)); // black for pixels not in any plane
}

// Note: input filename_prefix is like '/rgbd-image-folder-path/frame-XXXXXX'
void PlaneDetection::writeOutputFiles(string output_folder, string frame_name, bool run_mrf)
{
	computePlaneSumStats(run_mrf);

	if (output_folder.back() != '\\' && output_folder.back() != '/')
		output_folder += "/";	
	string filename_prefix = output_folder + frame_name + "-plane";

	//writePlaneLabelFile(filename_prefix + "-label.txt");
	//writePlaneDataFile(filename_prefix + "-data.txt");
	if (run_mrf)
	{
		cv::imwrite(filename_prefix + "-opt.png", opt_seg_img_);
		writePlaneLabelFile(filename_prefix + "-label-opt.txt", run_mrf);
		writePlaneDataFile(filename_prefix + "-data-opt.txt", run_mrf);
	}
	
}
void PlaneDetection::writePlaneLabelFile(string filename, bool run_mrf /* = false */)
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
			int label = run_mrf ? opt_membership_img_.at<int>(row, col) : plane_filter.membershipImg.at<int>(row, col);
			out << label << " ";
		}
		out << endl;
	}
	out.close();
}

void PlaneDetection::writePlaneDataFile(string filename, bool run_mrf /* = false */)
{
	ofstream out(filename, ios::out);
	out << "#plane_index number_of_points_on_the_plane plane_color_in_png_image(1x3) plane_normal(1x3) plane_center(1x3) "
		<< "sx sy sz sxx syy szz sxy syz sxz" << endl;

	for (int pidx = 0; pidx < plane_num_; ++pidx)
	{
		out << pidx << " ";
		if (!run_mrf)
			out << plane_pixel_nums_[pidx] << " ";
		else
			out << opt_plane_pixel_nums_[pidx] << " ";

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
		if (run_mrf)
		{
			out << opt_sum_stats_[pidx].sx << std::setprecision(8) << " " 
				<< opt_sum_stats_[pidx].sy << std::setprecision(8) << " " 
				<< opt_sum_stats_[pidx].sz << std::setprecision(8) << " " 
				<< opt_sum_stats_[pidx].sxx << std::setprecision(8) << " "
				<< opt_sum_stats_[pidx].syy << std::setprecision(8) << " "
				<< opt_sum_stats_[pidx].szz << std::setprecision(8) << " "
				<< opt_sum_stats_[pidx].sxy << std::setprecision(8) << " "
				<< opt_sum_stats_[pidx].syz << std::setprecision(8) << " "
				<< opt_sum_stats_[pidx].sxz << std::setprecision(8) << endl;
		}
		else
		{
			out << sum_stats_[pidx].sx << std::setprecision(8) << " " 
				<< sum_stats_[pidx].sy << std::setprecision(8) << " " 
				<< sum_stats_[pidx].sz << std::setprecision(8) << " " 
				<< sum_stats_[pidx].sxx << std::setprecision(8) << " "
				<< sum_stats_[pidx].syy << std::setprecision(8) << " "
				<< sum_stats_[pidx].szz << std::setprecision(8) << " "
				<< sum_stats_[pidx].sxy << std::setprecision(8) << " "
				<< sum_stats_[pidx].syz << std::setprecision(8) << " "
				<< sum_stats_[pidx].sxz << std::setprecision(8) << endl;
		}

		// NOTE: the plane-sum parameters computed from AHC code seems different from that computed from points belonging to planes shown above.
		// Seems there is a plane refinement step in AHC code so points belonging to each plane are slightly changed.
		//ahc::PlaneSeg::Stats& stat = plane_filter.extractedPlanes[pidx]->stats;
		//cout << stat.sx << " " << stat.sy << " " << stat.sz << " " << stat.sxx << " "<< stat.syy << " "<< stat.szz << " "<< stat.sxy << " "<< stat.syz << " "<< stat.sxz << endl;
	}
	out.close();
}

void PlaneDetection::computePlaneSumStats(bool run_mrf /* = false */)
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
	if (run_mrf)
	{
		opt_sum_stats_.resize(plane_num_);
		opt_plane_pixel_nums_.resize(plane_num_, 0);
		for (int row = 0; row < kDepthHeight; ++row)
		{
			for (int col = 0; col < kDepthWidth; ++col)
			{
				int label = opt_membership_img_.at<int>(row, col); // plane label each pixel belongs to
				if (label != plane_num_) // pixel belongs to some plane
				{
					opt_plane_pixel_nums_[label]++;
					int vidx = row * kDepthWidth + col;
					const VertexType& v = cloud.vertices[vidx];
					opt_sum_stats_[label].sx += v[0];		  opt_sum_stats_[label].sy += v[1];		    opt_sum_stats_[label].sz += v[2];
					opt_sum_stats_[label].sxx += v[0] * v[0]; opt_sum_stats_[label].syy += v[1] * v[1]; opt_sum_stats_[label].szz += v[2] * v[2];
					opt_sum_stats_[label].sxy += v[0] * v[1]; opt_sum_stats_[label].syz += v[1] * v[2]; opt_sum_stats_[label].sxz += v[0] * v[2];
				}
			}
		}
		for (int pidx = 0; pidx < plane_num_; ++pidx)
		{
			int num = opt_plane_pixel_nums_[pidx];
			opt_sum_stats_[pidx].sx /= num;		opt_sum_stats_[pidx].sy /= num;		opt_sum_stats_[pidx].sz /= num;
			opt_sum_stats_[pidx].sxx /= num;	opt_sum_stats_[pidx].syy /= num;	opt_sum_stats_[pidx].szz /= num;
			opt_sum_stats_[pidx].sxy /= num;	opt_sum_stats_[pidx].syz /= num;	opt_sum_stats_[pidx].sxz /= num;
		}
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
