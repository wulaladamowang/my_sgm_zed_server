#ifndef __GET_DISPARITY_H__
#define __GET_DISPARITY_H__

#include <libsgm.h>
#include <opencv2/opencv.hpp>
#include <cuda.h>
#include <cuda_runtime.h>
#include <cuda_runtime_api.h>
#include "little_tips.hpp"

struct device_buffer
{
	device_buffer() : data(nullptr) {}
	device_buffer(size_t count) { allocate(count); }
	void allocate(size_t count) { cudaMalloc(&data, count); }
	~device_buffer() { cudaFree(data); }
	void* data;
};

int get_disparity(sgm::StereoSGM& sgm, const cv::Mat& img_left, const cv::Mat& img_right, cv::Mat& disparity, const int disp_size, const bool subpixel);
#endif