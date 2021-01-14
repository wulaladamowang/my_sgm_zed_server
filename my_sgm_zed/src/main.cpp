#include "my_camera.hpp"
#include "get_point_cloud.hpp"
#include "get_roi.hpp"
#include "get_disparity.hpp"
#include "little_tips.hpp"
#include <thread>
#include <string>
#include <iostream>

int main(int argc, char** argv){

// zed相机的初始化
	sl::Camera zed;
	sl::InitParameters initParameters;
	initParameters.camera_resolution = sl::RESOLUTION::HD1080;
    if (argc >= 5) initParameters.input.setFromSVOFile(argv[4]);
	sl::ERROR_CODE err = zed.open(initParameters);
	if (err != sl::ERROR_CODE::SUCCESS) {
		std::cout << toString(err) << std::endl;
		zed.close();
		return 1;
	}
    sl::RuntimeParameters runtime_parameters;
    runtime_parameters.sensing_mode = sl::SENSING_MODE::LAST;

//图像变量的获取及初始化
    const int width = static_cast<int>(zed.getCameraInformation().camera_resolution.width);
	const int height = static_cast<int>(zed.getCameraInformation().camera_resolution.height);
    sl::Mat zed_image_l(zed.getCameraInformation().camera_resolution, sl::MAT_TYPE::U8_C1);
	sl::Mat zed_image_r(zed.getCameraInformation().camera_resolution, sl::MAT_TYPE::U8_C1);//相机原格式获得图像
    cv::Mat img_zed_left = slMat2cvMat(zed_image_l);
    cv::Mat img_zed_right = slMat2cvMat(zed_image_r);//sl::Mat 到opencv 格式图像的转换
    cv::Mat img_zed_left_remap;// 存储校正畸变后的图像
    cv::Mat img_zed_right_remap;


    const float scale = argc >= 2 ? atof(argv[1]) : 0.5;//图像缩放比例,默认为变为原来的0.5倍 
    const int disp_size = (argc >= 3) ? std::stoi(argv[2]) : 128;//默认的disparity size, 可选择64,128,256
    const bool subpixel = (argc >= 4) ? std::stoi(argv[3]) != 0 : true;//是否使用subpixel

// 相机内外参数的读取, 注意相机内外参数要与使用的相机型号或拍摄视频的相机型号相一致
    std::string in = "/home/wang/code/c++Code/my_sgm_zed_server/canshu/intrinsics.yml";
    cv::FileStorage fs(in, cv::FileStorage::READ);
    if(!fs.isOpened())
    {
        printf("Failed to open file %s\n", argv[3]);
        return -1;
    }
    cv::Mat M1, D1, M2, D2;
    fs["M1"] >> M1;
    fs["D1"] >> D1;
    fs["M2"] >> M2;
    fs["D2"] >> D2;
    std::string out = "/home/wang/code/c++Code/my_sgm_zed_server/canshu/extrinsics.yml";
    fs.open(out, cv::FileStorage::READ);
    if(!fs.isOpened())
    {
        printf("Failed to open file %s\n", argv[4]);
        return -1;
    }
    cv::Mat R, T, R1, P1, R2, P2;
    fs["R"] >> R;
    fs["T"] >> T;
    cv::Rect roi1, roi2;
    cv::Mat Q;
    cv::Size img_size = img_zed_right.size();
    cv::stereoRectify( M1, D1, M2, D2, img_size, R, T, R1, R2, P1, P2, Q, cv::CALIB_ZERO_DISPARITY, -1, img_size, &roi1, &roi2 );
    cv::Mat map11, map12, map21, map22;
    cv::initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, map11, map12);
    cv::initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_16SC2, map21, map22);

    // cv::Mat img_left = cv::imread(argv[1], -1);
    // cv::Mat img_right = cv::imread(argv[2], -1);
    cv::Size siz = cv::Size(width*scale, height*scale);// 对于原图进行缩放尺寸
    cv::Mat img_zed_left_scale, img_zed_right_scale;// 缩放之后的图像, 在校正之后的图像基础上进行缩放
    cv::Mat disparity(siz, CV_16S);//存储sgm算法获得的视差图
    cv::Mat disparity_8u, disparity_32f, disparity_mask;// 8u转化便于显示的灰度图, 32f实际视差图,带小数点, disparity_mask保留mask区域的视差
    char key = ' ';
    long long ts0 =getCurrentTime();
    long long ts1 =getCurrentTime();
    long long ts2 = getCurrentTime();
    long long ts3 =getCurrentTime();
    long long ts4 = getCurrentTime();

    long long ts5 =getCurrentTime();
    long long ts6 = getCurrentTime();
    bool has_roi = false;// 是否检测到roi
    cv::Mat mask(img_size, CV_8U);// 检测到的roi, 用mask 标记出来
    cv::Mat mask_scale(siz, CV_8U);
    std::vector<int> rect_roi;// 存储包围目标区域的最小正矩形的四个像素坐标

    while('q' != key){
        if (zed.grab(runtime_parameters) == sl::ERROR_CODE::SUCCESS) {
            ts0 =getCurrentTime();
            // Retrieve the left image, depth image in half-resolution
            zed.retrieveImage(zed_image_l, sl::VIEW::LEFT_UNRECTIFIED_GRAY, sl::MEM::CPU);
            zed.retrieveImage(zed_image_r, sl::VIEW::RIGHT_UNRECTIFIED_GRAY, sl::MEM::CPU);
            ts1 = getCurrentTime();
            std::cout << "get image: " << (ts1-ts0) << "毫秒" << std::endl;

            cv::remap(img_zed_left, img_zed_left_remap, map11, map12, cv::INTER_LINEAR);
            cv::remap(img_zed_right, img_zed_right_remap, map21, map22, cv::INTER_LINEAR);
            cv::resize(img_zed_left_remap, img_zed_left_scale, siz, cv::INTER_LINEAR);
            cv::resize(img_zed_right_remap, img_zed_right_scale, siz, cv::INTER_LINEAR);
            ts2 = getCurrentTime();
            std::cout << "Process image: " << (ts2-ts1) << "毫秒" << std::endl;
            
            get_roi(std::ref(img_zed_left_remap), std::ref(mask), std::ref(has_roi), std::ref(rect_roi));
            cv::resize(mask, mask_scale, siz, cv::INTER_LINEAR);
            ts3 = getCurrentTime();
            std::cout << "roi image: " << (ts3-ts2) << "毫秒" << std::endl;

            get_disparity(std::ref(img_zed_left_scale), std::ref(img_zed_right_scale), std::ref(disparity), disp_size, subpixel);
            disparity.convertTo(disparity_32f, CV_32F, subpixel ? 1. / sgm::StereoSGM::SUBPIXEL_SCALE : 1);
            ts4 = getCurrentTime();
            std::cout << "disparity image: " << (ts4-ts3) << "毫秒" << std::endl;

            ts5 = getCurrentTime();
            std::cout << "total: " << (ts5-ts6) << "毫秒" << std::endl;
            std::cout << "-------------" << std::endl;
            disparity_32f.convertTo(disparity_8u, CV_8U, 255. / 64);
            disparity_mask = disparity_8u.mul(mask_scale);
            cv::imshow("left image", img_zed_left_scale);
            cv::imshow("disparity 1", disparity_8u);
            cv::imshow("mask", disparity_mask);
            key = cv::waitKey(1);
            ts6 = getCurrentTime();
        }
    }
    zed.close();
    cv::destroyAllWindows();

    return 0;
}


