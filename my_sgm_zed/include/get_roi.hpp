#ifndef __GET_ROI_H__
#define __GET_ROI_H__


#include <vector> 
#include <opencv2/imgproc.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>
#include <iostream>
int relativeDis(cv::Vec4f line_para, std::vector<cv::Point2f> point);
void get_roi(cv::Mat& image, cv::Mat& mask, bool& has_roi, std::vector<int>& rect_roi) ;

#endif