
#include "get_disparity.hpp"


int get_disparity(cv::Mat& img_left, cv::Mat& img_right, cv::Mat& disparity, const int disp_size, const bool subpixel){
    static const int width = img_left.cols;
    static const int height = img_left.rows;
    
    static const int input_depth = img_left.type() == CV_8U ? 8 : 16;
    static const int input_bytes = input_depth * width * height / 8;
    static const int output_depth = 16;
    static const int output_bytes = output_depth * width * height / 8;

    const sgm::StereoSGM::Parameters params{10, 120, 0.95f, subpixel};
    static sgm::StereoSGM sgm(width, height, disp_size, input_depth, output_depth, sgm::EXECUTE_INOUT_CUDA2CUDA, params);

    static device_buffer d_I1(input_bytes), d_I2(input_bytes), d_disparity(output_bytes);

    
    cudaMemcpy(d_I1.data, img_left.data, input_bytes, cudaMemcpyHostToDevice);//input_bytes : nBytes
	cudaMemcpy(d_I2.data, img_right.data, input_bytes, cudaMemcpyHostToDevice);
    sgm.execute(d_I1.data, d_I2.data, d_disparity.data);
    cudaDeviceSynchronize();
    cudaMemcpy(disparity.data, d_disparity.data, output_bytes, cudaMemcpyDeviceToHost);
    
    return 0;
}