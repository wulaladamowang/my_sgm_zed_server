/*
 * @Author: your name
 * @Date: 2020-12-12 10:31:18
 * @LastEditTime: 2020-12-21 09:58:59
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /sgm_code/src/init_camera.cpp
 */

#include "my_camera.hpp"
/*
 * 初始化相机参数
 * nb_zeds: 连接的硬件相机数
 * zeds: 将连接的相机赋予指定的打开模式
 */
int init_camera_parameters(int nb_zeds, std::vector<sl::Camera>& zeds){
    std::vector<sl::DeviceProperties> devList = sl::Camera::getDeviceList();
    int nb_detected_zed = devList.size();
    for (int z = 0; z < nb_detected_zed; z++){
        std::cout << "ID : " << devList[z].id << " ,model : " << devList[z].camera_model << " , S/N : " << devList[z].serial_number << " , state : "<<devList[z].camera_state<<std::endl;
    }
    if (nb_detected_zed == 0) {
        std::cout << "No ZED Detected, exit program" << std::endl;
        return EXIT_FAILURE;
    }
    std::cout << nb_detected_zed << " ZED Detected" << std::endl;
    if (nb_detected_zed != nb_zeds){
        std::cout << "未能检测全部ZED相机 " << std::endl;
        return EXIT_FAILURE;
    }

	sl::InitParameters init_parameters;
    init_parameters.depth_mode = sl::DEPTH_MODE::PERFORMANCE;
    init_parameters.camera_resolution = sl::RESOLUTION::HD1080;

    zeds = std::move(std::vector<sl::Camera>(nb_detected_zed));
    // try to open every detected cameras
    for (int z = 0; z < nb_detected_zed; z++) {
        init_parameters.input.setFromCameraID(z);
        sl::ERROR_CODE err = zeds[z].open(init_parameters);
        if (err == sl::ERROR_CODE::SUCCESS) {
            auto cam_info = zeds[z].getCameraInformation();
            std::cout << cam_info.camera_model << ", ID: " << z << ", SN: " << cam_info.serial_number << " Opened" << std::endl;
        } else {
            std::cout << "ZED ID:" << z << " Error: " << err << std::endl;
            zeds[z].close();
        }
    }
    return EXIT_SUCCESS;
};

/*
传入打开的相机， 获得相机图像
zed: 打开的相机类
img_left，img_right: 左右相机照片
ts: 获得图片的时间
*/
void zed_acquisition(sl::Camera& zed, cv::Mat& img_left, cv::Mat& img_right, std::vector<cv::Mat>& map, bool& run, long long& ts) {
    sl::Mat zed_image;
    sl::Resolution res = zed.getCameraInformation().camera_configuration.resolution;
    const int w_low_res = res.width;
    const int h_low_res = res.height;
    while (run) {
        // grab current images and compute depth
        if (zed.grab() == sl::ERROR_CODE::SUCCESS) {
            zed.retrieveImage(zed_image, sl::VIEW::LEFT_UNRECTIFIED_GRAY, sl::MEM::CPU, res);
            // copy Left image to the left part of the side by side image
            cv::cvtColor(cv::Mat(h_low_res, w_low_res, CV_8UC4, zed_image.getPtr<sl::uchar1>(sl::MEM::CPU)),img_left, cv::COLOR_RGBA2RGB);
            zed.retrieveImage(zed_image, sl::VIEW::RIGHT_UNRECTIFIED_GRAY, sl::MEM::CPU, res);
            // copy Dpeth image to the right part of the side by side image
            cv::cvtColor(cv::Mat(h_low_res, w_low_res, CV_8UC4, zed_image.getPtr<sl::uchar1>(sl::MEM::CPU)), img_right, cv::COLOR_RGBA2RGB);
            cv::remap(img_left, img_left, map[0], map[1], cv::INTER_LINEAR);
            cv::remap(img_right, img_right, map[2], map[3], cv::INTER_LINEAR);
            ts = getCurrentTime();
        }
        sl::sleep_ms(1);
    }
}

/*
图像转换函数
*/
cv::Mat slMat2cvMat(sl::Mat& input) {
    int cv_type = -1;
    switch (input.getDataType()) {
        case sl::MAT_TYPE::F32_C1: cv_type = CV_32FC1; break;
        case sl::MAT_TYPE::F32_C2: cv_type = CV_32FC2; break;
        case sl::MAT_TYPE::F32_C3: cv_type = CV_32FC3; break;
        case sl::MAT_TYPE::F32_C4: cv_type = CV_32FC4; break;
        case sl::MAT_TYPE::U8_C1: cv_type = CV_8UC1; break;
        case sl::MAT_TYPE::U8_C2: cv_type = CV_8UC2; break;
        case sl::MAT_TYPE::U8_C3: cv_type = CV_8UC3; break;
        case sl::MAT_TYPE::U8_C4: cv_type = CV_8UC4; break;
        default: break;
    }
    return cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(sl::MEM::CPU));
}