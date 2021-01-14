#include "get_point_cloud.hpp"
void get_image_transform_mat(char* intrinsics, char* extrinsics, std::vector<cv::Mat>& mats){
    std::string in = intrinsics;
    cv::FileStorage fs(in, cv::FileStorage::READ);
    if(!fs.isOpened())
    {
        printf("Failed to open file %s\n", intrinsics);
        return;
    }

    cv::Mat M1, D1, M2, D2;
    fs["M1"] >> M1;
    fs["D1"] >> D1;
    fs["M2"] >> M2;
    fs["D2"] >> D2;
    std::cout << "M1-------------------" << std::endl;
    std::cout << M1 << std::endl;
    std::cout << "D1-------------------" << std::endl;
    std::cout << D1 << std::endl;
    std::cout << "M2-------------------" << std::endl;
    std::cout << M2 << std::endl;
    std::cout << "D2-------------------" << std::endl;
    std::cout << D2 << std::endl;
    std::cout << "-------------------" << std::endl;

    std::string out = extrinsics;
    fs.open(out, cv::FileStorage::READ);
    if(!fs.isOpened())
    {
        printf("Failed to open file %s\n", extrinsics);
        return;
    }
    cv::Mat R, T;
    fs["R"] >> R;
    fs["T"] >> T;

    std::cout << "R-------------------" << std::endl;
    std::cout << R << std::endl;
    std::cout << "T-------------------" << std::endl;
    std::cout << T << std::endl;
    std::cout << "-------------------" << std::endl;
    mats.push_back(M1);
    mats.push_back(D1);
    mats.push_back(M2);
    mats.push_back(D2);
    mats.push_back(R);
    mats.push_back(T);
}
void get_tranform_mat(const std::vector<cv::Mat>& mats, const cv::Size& img_size, std::vector<cv::Mat>& maps){
     cv::Mat Q;
     cv::Mat R1, P1, R2, P2;
     cv::stereoRectify( mats[0], mats[1], mats[2], mats[3], img_size, mats[4], mats[5], R1, R2, P1, P2, Q, cv::CALIB_ZERO_DISPARITY, -1, img_size, nullptr, nullptr );
    cv::Mat map11, map12, map21, map22;
    cv::initUndistortRectifyMap(mats[0], mats[1], R1, P1, img_size, CV_16SC2, map11, map12);
    cv::initUndistortRectifyMap(mats[2], mats[3], R2, P2, img_size, CV_16SC2, map21, map22);
    maps.push_back(map11);
    maps.push_back(map12);
    maps.push_back(map21);
    maps.push_back(map22);
}

void get_disparity(const cv::Mat& img_left, const cv::Mat& img_right, long long& ts, bool& run, cv::Mat& img_disparity){
    long long last_ts = 0;
    while(run){
        if(ts > last_ts){
            //获得深度函数 img_left + img_right = img_disparity
            last_ts = ts;
        }
    }
};