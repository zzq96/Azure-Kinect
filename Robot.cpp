#include "Robot.h"

Robot::Robot(const std::string& Homo_cam2base_file, k4a::KinectAPI *kinect, bool verbose)
{
	this->kinect = kinect;
	cv::FileStorage fs2(Homo_cam2base_file, cv::FileStorage::READ); //��ȡ����������ת����ϵXML�ļ�  
	fs2["color_Homo_cam2base"] >> color_Homo_cam2base;
	fs2["depth_Homo_cam2base"] >> depth_Homo_cam2base;
	if (verbose)
	{
		std::cout << "color_Homo_cam2base" << color_Homo_cam2base << std::endl;
		std::cout << "depth_Homo_cam2base" << depth_Homo_cam2base << std::endl;
	}
	cv::Mat depth_Homo_base2cam = depth_Homo_cam2base.inv();
	fs2.release();
	/*����ξ���ת��Ϊ��ת�����ƽ�������������������*/
	HomogeneousMtr2RT(depth_Homo_base2cam, depth_R_base2cam, depth_t_base2cam);
}
bool Robot::isRotationMatrix(cv::Mat& R)
{
	cv::Mat Rt;
	transpose(R, Rt);
	cv::Mat shouldBeIdentity = Rt * R;
	cv::Mat I = cv::Mat::eye(3, 3, shouldBeIdentity.type());
	return  norm(I, shouldBeIdentity) < 1e-6;
}

// Calculates rotation matrix to euler angles
// The result is the same as MATLAB except the order
// of the euler angles ( x and z are swapped ).
cv::Vec3f Robot::rotationMatrixToEulerAngles(cv::Mat& R)
{
	//assert(isRotationMatrix(R));

	double sy = sqrt(R.at<double>(0, 0) * R.at<double>(0, 0) + R.at<double>(1, 0) * R.at<double>(1, 0));

	bool singular = sy < 1e-6; // If

	double x, y, z;
	if (!singular) {
		x = atan2(R.at<double>(2, 1), R.at<double>(2, 2));
		y = atan2(-R.at<double>(2, 0), sy);
		z = atan2(R.at<double>(1, 0), R.at<double>(0, 0));
	}
	else {
		x = atan2(-R.at<double>(1, 2), R.at<double>(1, 1));
		y = atan2(-R.at<double>(2, 0), sy);
		z = 0;
	}
	return cv::Vec3f(z, y, x) / CV_PI * 180;
}
//�����(row,col������ȣ�Ϊ�˽�������Ӱ�죬����Χ2*size��С������������Ϊ�����㡣
double Robot::getDepthValue(cv::Mat depthMat, int row, int col, int size)
{
	int rows = depthMat.rows, cols = depthMat.cols;
	double sum = 0;
	int cnt = 0;
	for (int r = std::max(0, row - size); r <= std::min(rows - 1, row + size); r++)
	{
		for (int c = std::max(0, col - size); c <= std::min(cols - 1, col + size); c++)
			if (depthMat.at<cv::uint16_t>(r, c) != 0)
			{
				cnt++;
				sum += depthMat.at<cv::uint16_t>(r, c);
			}
	}
	assert(cnt);
	return sum / cnt;
}
//�����point2D����ȣ�Ϊ�˽�������Ӱ�죬����Χ2*size��С������������Ϊ�����㡣
cv::uint16_t Robot::getDepth(const cv::Mat & depthMat, cv::Mat &point2D)
{
	int x = (int)point2D.at<double>(0, 0);
	int y = (int)point2D.at<double>(1, 0);
	cv::uint16_t Zc = getDepthValue(depthMat, y, x, 6);
	//TODO:����߽��ж�
	return Zc;
}
double Robot::getLength(cv::Mat a)
{
	double dx = a.at<double>(0, 0);
	double dy = a.at<double>(1, 0);
	double d = sqrt(dx * dx + dy * dy);
	return d;
}
double Robot::getDistance(cv::Point3f a, cv::Point3f b)
{
	double  d = sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
	return d;
}
//��ξ���ת��Ϊ��ת�����ƽ������
void Robot::HomogeneousMtr2RT(cv::Mat& HomoMtr, cv::Mat& R, cv::Mat& T)
{
	//Mat R_HomoMtr = HomoMtr(Rect(0, 0, 3, 3)); //ע��Rectȡֵ
	//Mat T_HomoMtr = HomoMtr(Rect(3, 0, 1, 3));
	//R_HomoMtr.copyTo(R);
	//T_HomoMtr.copyTo(T);
	/*HomoMtr(Rect(0, 0, 3, 3)).copyTo(R);
	HomoMtr(Rect(3, 0, 1, 3)).copyTo(T);*/
	cv::Rect R_rect(0, 0, 3, 3);
	cv::Rect T_rect(3, 0, 1, 3);
	R = HomoMtr(R_rect);
	T = HomoMtr(T_rect);

}

//��ת�����ƽ������ת��Ϊ��ξ���
cv::Mat Robot::RT2HomogeneousMatrix(const cv::Mat& R, const cv::Mat& T)
{
	cv::Mat HomoMtr;
	cv::Mat_<double> R1 = (cv::Mat_<double>(4, 3) <<
		R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
		R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
		R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2),
		0, 0, 0);
	cv::Mat_<double> T1 = (cv::Mat_<double>(4, 1) <<
		T.at<double>(0, 0),
		T.at<double>(1, 0),
		T.at<double>(2, 0),
		1);
	cv::hconcat(R1, T1, HomoMtr);		//����ƴ��
	return HomoMtr;
}
//������������ϵ�ĵ㣬���ػ�е������ϵ�µ�����
void Robot::calPoint3D(cv::Mat point2D, cv::Point3f& real, cv::uint16_t Zc)
{
	assert(Zc != 0);
	cv::Mat point3D = depth_R_base2cam.t() * (kinect->depthCameraMatrix.inv() * Zc * point2D - depth_t_base2cam);
	real.x = point3D.at<double>(0, 0);
	real.y = point3D.at<double>(1, 0);
	real.z = point3D.at<double>(2, 0);
}
cv::Mat Robot::calPoint3D(cv::Mat & depthMat, cv::Mat point2D)
{
	double Zc = getDepthValue(depthMat, point2D.at<double>(1, 0), point2D.at<double>(0, 0), 6);
	return depth_R_base2cam.t() * (kinect->depthCameraMatrix.inv() * Zc * point2D - depth_t_base2cam);
}
//������Ӿ��μ����ݵ���ת����
cv::Mat Robot::calRotationMatrix(cv::Mat& depthMat,  cv::Point2f* R, double scale)
{
	cv::Mat a = (cv::Mat_<double>(3, 1) << R[0].x, R[0].y, 1);
	cv::Mat b = (cv::Mat_<double>(3, 1) << R[1].x, R[1].y, 1);
	cv::Mat c = (cv::Mat_<double>(3, 1) << R[2].x, R[2].y, 1);
	cv::Mat d = (cv::Mat_<double>(3, 1) << R[3].x, R[3].y, 1);
	//cout << "abcd" << a << b << c << d << endl;
	cv::Mat center = (a + b + c + d) / 4;
	//cout << "center" << center << endl;

	cv::Mat aa = center + (a - center) * scale;
	cv::Mat bb = center + (b - center) * scale;
	cv::Mat cc = center + (c - center) * scale;
	cv::Mat dd = center + (d - center) * scale;
	//cout << aa << bb << cc << dd << endl;

	cv::Point3f raa, rbb, rcc, rdd, rcenter;
	calPoint3D(center, rcenter, getDepth(depthMat, center));
	calPoint3D(aa, raa, getDepth(depthMat, aa));
	calPoint3D(bb, rbb, getDepth(depthMat, bb));
	calPoint3D(cc, rcc, getDepth(depthMat, cc));
	calPoint3D(dd, rdd, getDepth(depthMat, dd));
	//����4���ߵ�����
	cv::Point3f ab = raa - rbb;
	cv::Point3f dc = rdd - rcc;
	cv::Point3f ad = raa - rdd;
	cv::Point3f bc = rbb - rcc;
	//���ߵķ��򵱳�x���y��ƽ��һ��
	cv::Point3f v1 = (ab + dc) / 2;
	cv::Point3f v2 = (ad + bc) / 2;
	//�ѽϳ��ıߵ���x�ᣬ��Ϊ2��������x�᷽���
	cv::Point3f X, Y;
	if (v1.dot(v1) > v2.dot(v2))
	{
		X = v1 / sqrt(v1.dot(v1));//��λ��
		Y = v2 / sqrt(v2.dot(v2));
	}
	else
	{
		X = v2 / sqrt(v2.dot(v2));//��λ��
		Y = v1 / sqrt(v1.dot(v1));
	}

	if (X.x < 0)
		X *= -1;

	//��������������Ȼ��ƽ��һ�£�����
	cv::Point3f normal1 = (raa - rbb).cross(raa - rdd);
	cv::Point3f normal2 = (rcc - rbb).cross(rcc - rdd);
	cv::Point3f normal3 = (rbb - raa).cross(rbb - rcc);
	cv::Point3f normal4 = (rdd - raa).cross(rdd - rcc);
	normal1 = normal1 / sqrt(normal1.dot(normal1));
	normal2 = normal2 / sqrt(normal2.dot(normal2));
	normal3 = normal3 / sqrt(normal3.dot(normal3));
	normal4 = normal4 / sqrt(normal4.dot(normal4));

	if (normal1.z < 0)
		normal1 *= -1;
	if (normal2.z < 0)
		normal2 *= -1;
	if (normal3.z < 0)
		normal3 *= -1;
	if (normal4.z < 0)
		normal4 *= -1;

	cv::Point3f normal = (normal1 + normal2 + normal3 + normal4) / 4;
	normal = normal / sqrt(normal.dot(normal));

	//��֪x���z�ᣬ���ݲ�����y��,��֮ǰ�õ�y��ͳһ����
	if (normal.cross(X).x * Y.x < 0)
		Y *= -1;
	//cout << "����" << endl;
	//cout << X.dot(X) << " " << Y.dot(Y) << " " << normal.dot(normal) << endl;;

	cv::Mat rotation = cv::Mat_<double>(3, 3);
	rotation.at<double>(0, 0) = X.x;
	rotation.at<double>(1, 0) = X.y;
	rotation.at<double>(2, 0) = X.z;
	rotation.at<double>(0, 1) = Y.x;
	rotation.at<double>(1, 1) = Y.y;
	rotation.at<double>(2, 1) = Y.z;
	rotation.at<double>(0, 2) = normal.x;
	rotation.at<double>(1, 2) = normal.y;
	rotation.at<double>(2, 2) = normal.z;

	//����֮ǰ�����x,y,z��3���᲻һ��������������������������ֵ�ֽ�ķ������һ�����������ľ���
	cv::Mat w, u, vt;
	cv::SVDecomp(rotation, w, u, vt);
	cv::Mat I = cv::Mat::eye(3, 3, rotation.type());
	cv::Mat new_rotation = u * I * vt;

	return new_rotation;
}
//����
/*������z��*/
double Robot::calAngle(cv::Point2f* R, int h, int w, cv::uint16_t depth)
{
	int size = 4;
	cv::Mat point2D(3, 1, CV_64F, cv::Scalar(0));
	point2D.at<double>(2, 0) = 1;
	cv::Mat point3D(3, 1, CV_64F, cv::Scalar(0));
	cv::Point3f a, b, c;//�����������������ʵ����
	/*���ĳһ���㳬���߽磬��˳�Ƶ㡣ë����Ӧ�����ֻ��һ���㳬���߽�*/
	int k;
	for (int i = 0; i < size; i++)
	{
		k = i;
		if (R[k].x < 0 || R[k].x >= w || R[k].y < 0 || R[k].y >= h)
			continue;
		point2D.at<double>(0, 0) = R[k].x;
		point2D.at<double>(1, 0) = R[k].y;
		calPoint3D(point2D, a, depth);

		k = (i + 1) % size;
		if (R[k].x < 0 || R[k].x >= w || R[k].y < 0 || R[k].y >= h)
			continue;
		point2D.at<double>(0, 0) = R[k].x;
		point2D.at<double>(1, 0) = R[k].y;
		calPoint3D(point2D, b, depth);

		k = (i + 2) % size;
		if (R[k].x < 0 || R[k].x >= w || R[k].y < 0 || R[k].y >= h)
			continue;
		point2D.at<double>(0, 0) = R[k].x;
		point2D.at<double>(1, 0) = R[k].y;
		calPoint3D(point2D, c, depth);
		break;
	}
	double angle;
	//cout << "�߳�1" << getDistance(a, b) << ":�߳�2" << getDistance(b, c) << endl;
	//cout << endl;
	if (getDistance(a, b) > getDistance(b, c))
	{
		if (abs(a.x - b.x) < 30)
		{
			angle = 90;
		}
		else
		{
			double k = (a.y - b.y) / (a.x - b.x);
			angle = (tanh(k) / PI * 180);
		}
	}
	else
	{
		if (abs(b.x - c.x) < 30)
		{
			angle = 90;
		}
		else
		{
			double k = (b.y - c.y) / (b.x - c.x);
			angle = (tanh(k) / PI * 180);
		}
	}
	//cout << "oldangle" << endl;
	//cout << angle << endl;
	if (angle > 90)
		angle -= 180;
	if (angle < -90)
		angle += 180;
	return angle;
}
