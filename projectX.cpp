#include "plane_detection.h"

int main1()
{
    string depthFileName = "C:\\Users\\tota1Noob\\source\\repos\\projectX\\imgs\\img11_depth.png";
    string colorFileName = "C:\\Users\\tota1Noob\\source\\repos\\projectX\\imgs\\img11_color.png";
    
    cv::Mat colorSrc = imread(colorFileName, cv::IMREAD_ANYCOLOR);
    cv::Mat depthSrc = imread(depthFileName, cv::IMREAD_ANYDEPTH);

    double* center;
    double* normal;
    float angle;

    cv::imshow("result", processImg(colorSrc, depthSrc, center, normal, angle));

    std::cout << center[0] << " " << center[1] << " " << center[2] << std::endl;
    std::cout << normal[0] << " " << normal[1] << " " << normal[2] << std::endl;
    std::cout << angle << std::endl;

    cv::waitKey(0);
    /*PlaneDetection plane_detection;
    plane_detection.readDepthImage(depthSrc, roi);
    plane_detection.readColorImage(colorSrc, roi);

    int pos = depthFileName.find_last_of("/\\");
    string fileName = depthFileName.substr(pos + 1);
    pos = fileName.find_last_of("_");
    fileName = fileName.substr(0, pos);
      
    ahc::Timer timer(1000);
    timer.tic();
    plane_detection.runPlaneDetection();
    timer.toctic("Total");
    
    plane_detection.writeOutputFiles("D:\\Desktop\\test", fileName);*/
}

