#pragma once
#include <stdio.h>
#include <string.h>
#include <curl/curl.h>
#include <fstream>
#include <iostream>

#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"

#include <opencv2/highgui/highgui_c.h>
#include <opencv2/opencv.hpp>
#include <io.h>     
#include <string>
#include <direct.h>    
#include <vector>    
//#include <axxbsolver.h>
//#include<conventionalaxxbsvdsolver.h>
 
#include<opencv2/imgproc/types_c.h>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace rapidjson;

void getMasks(cv::Mat& mat, vector<cv::Mat> & masks);
