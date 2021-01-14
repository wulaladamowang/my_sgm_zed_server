#ifndef __GET_POINT_CLOUD_H__
#define __GET_POINT_CLOUD_H__

#include <sl/Camera.hpp>
#include <opencv2/opencv.hpp>
#include <string>

void get_image_transform_mat(char* intrinsics, char* extrinsics, std::vector<cv::Mat>& mats);
void get_disparity(const cv::Mat& img_left, const cv::Mat& img_right, long long& ts, bool& run, cv::Mat& img_disparity);
void get_tranform_mat(const std::vector<cv::Mat>& mats, const cv::Size& img_size, std::vector<cv::Mat>& maps);

#endif