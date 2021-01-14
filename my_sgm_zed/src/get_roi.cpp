#include "get_roi.hpp"

int relativeDis(cv::Vec4f line_para, std::vector<cv::Point2f> point) {
    double A = line_para[1]/line_para[0];
    double B = -1;
    double C = line_para[3]*(1-line_para[1]/line_para[0]);
    double min = 0.0;
    int index = -1;
    for(int i=0;i<point.size();i++){
        double dis = A*point[i].x+B*point[i].y+C;
        if(dis<0)
            dis = -dis;
        if(-1==index || dis<min)
        {
            min = dis;
            index = i;
        }
    }
    return index;
};

void get_roi(cv::Mat& image, cv::Mat& mask, bool& has_roi, std::vector<int>& rect_roi) {
    
    static cv::Size mask_size = image.size();
    static const cv::Ptr<cv::aruco::Dictionary> c_dictionary = cv::aruco::getPredefinedDictionary(
        cv::aruco::DICT_4X4_50);//DICT_6X6_1000
    static const double pi = acos(-1.0);
    static std::vector<std::vector<cv::Point2f>> marker_corners;
    static std::vector<int> marker_ids;
    int min_x, min_y, max_x, max_y;
    cv::aruco::detectMarkers(image, c_dictionary, marker_corners, marker_ids);
    ///获得检测的面积最大的aruco marker序号,每个ID都有一个（若检测到）
    int marker_number = marker_ids.size();
    if(0 != marker_number)
    {
        int buff_id[6] = {-1, -1, -1, -1, -1, -1};//若存在相应的aruco marker id, 则记录其序号
        double buff_id_length[6] = {0.0, 0, 0, 0, 0, 0};//若存在相应的aruco marker， 则记录其在当前时刻周长的最大值
        for(int i=0;i<marker_number;i++){
            double cur_marker_len = cv::arcLength(marker_corners[i], true);
            if(1 == marker_ids[i]){
                if(cur_marker_len > buff_id_length[1]){
                    buff_id_length[1] = cur_marker_len;
                    buff_id[1] = i;
                }
            }else if(2 == marker_ids[i]){
                if(cur_marker_len > buff_id_length[2]){
                    buff_id_length[2] = cur_marker_len;
                    buff_id[2] = i;
                }
            }else if(3 == marker_ids[i]){
                if(cur_marker_len > buff_id_length[3]){
                    buff_id_length[3] = cur_marker_len;
                    buff_id[3] = i;
                }
            }else if(4 == marker_ids[i]){
                if(cur_marker_len > buff_id_length[4]){
                    buff_id_length[4] = cur_marker_len;
                    buff_id[4] = i;
                }
           }else if(5 == marker_ids[i]){
               if(cur_marker_len > buff_id_length[5]){
                   buff_id_length[5] = cur_marker_len;
                   buff_id[5] = i;
               }
            } else
                continue;
        }

        std::vector<cv::Point2f> compute_line;///用于选中ID的线，每个存储的为选中的ID的中点坐标
        std::vector<int> no_id;///存储被选中的ID的id号
        for(int m=5;m>0;m--){
            if(-1!=buff_id[m]){
                compute_line.emplace_back(cv::Point2f((marker_corners[buff_id[m]][1].x + marker_corners[buff_id[m]][2].x + marker_corners[buff_id[m]][3].x + marker_corners[buff_id[m]][0].x)/4,
                                                      (marker_corners[buff_id[m]][1].y + marker_corners[buff_id[m]][2].y + marker_corners[buff_id[m]][3].y + marker_corners[buff_id[m]][0].y)/4));
                no_id.push_back(m);
            }
        }
        int center_x ;
        int center_y ;
        int index = 0;///记录compute_line中距离拟合线最近的点的序号
        if(no_id.size()<3){
        }else{
            cv::Vec4f line_para;
            cv::fitLine(compute_line, line_para, cv::DIST_L2, 0, 1e-2, 1e-2);
            index = relativeDis(line_para, compute_line);
        }
        center_x = compute_line[index].x;
        center_y = compute_line[index].y;

        std::vector<cv::Point > roi_position;//用于记录roi四个角点的位置
        roi_position.reserve(4);
        cv::Point roi_p0, roi_p1, roi_p2, roi_p3;//用来包裹整个目标圆柱
        ///轴向比例扩大，该参数通过过分支choose coefficient 获得，与目标检测物与贴码位置有关
        float axial_coefficient = 0;//目标长度为marker边长的倍数
        float axial_position_coefficient = 0.0;//marker的ID不同，则则其位于目标的位置不同，距离ID1的上端的位置分数比例
        float radial_coefficient = 0;//目标横向（径向）为marker边长的倍数
        switch (no_id[index]) {
            case 5 : axial_coefficient = 12 ; axial_position_coefficient = 0.13 ; radial_coefficient = 1.4  ;break;
            case 4 : axial_coefficient = 13 ; axial_position_coefficient = 0.32 ; radial_coefficient = 1.9  ;break;
            case 3 : axial_coefficient = 14 ; axial_position_coefficient = 0.50 ; radial_coefficient = 2.2 ;break;
            case 2 : axial_coefficient = 16 ; axial_position_coefficient = 0.67 ; radial_coefficient = 2.5 ;break;
            case 1 : axial_coefficient = 17 ; axial_position_coefficient = 0.84 ; radial_coefficient = 3.1 ;break;
        }

        int i = no_id[index];
        ///纵向比例扩大

        roi_p0.x = axial_position_coefficient*axial_coefficient*(marker_corners[buff_id[i]][1].x - marker_corners[buff_id[i]][0].x)+marker_corners[buff_id[i]][0].x;
        roi_p0.y = axial_position_coefficient*axial_coefficient*(marker_corners[buff_id[i]][1].y - marker_corners[buff_id[i]][0].y)+marker_corners[buff_id[i]][0].y;

        roi_p1.x = (1-axial_position_coefficient)*axial_coefficient*(marker_corners[buff_id[i]][0].x - marker_corners[buff_id[i]][1].x)+marker_corners[buff_id[i]][1].x;
        roi_p1.y = (1-axial_position_coefficient)*axial_coefficient*(marker_corners[buff_id[i]][0].y - marker_corners[buff_id[i]][1].y)+marker_corners[buff_id[i]][1].y;

        roi_p2.x = (1-axial_position_coefficient)*axial_coefficient*(marker_corners[buff_id[i]][3].x - marker_corners[buff_id[i]][2].x)+marker_corners[buff_id[i]][2].x;
        roi_p2.y = (1-axial_position_coefficient)*axial_coefficient*(marker_corners[buff_id[i]][3].y - marker_corners[buff_id[i]][2].y)+marker_corners[buff_id[i]][2].y;

        roi_p3.x = axial_position_coefficient*axial_coefficient*(marker_corners[buff_id[i]][2].x - marker_corners[buff_id[i]][3].x)+marker_corners[buff_id[i]][3].x;
        roi_p3.y = axial_position_coefficient*axial_coefficient*(marker_corners[buff_id[i]][2].y - marker_corners[buff_id[i]][3].y)+marker_corners[buff_id[i]][3].y;
        ///径向比例扩大

        roi_p0.x = radial_coefficient*(marker_corners[buff_id[i]][0].x - marker_corners[buff_id[i]][3].x)+roi_p0.x;
        roi_p0.y = radial_coefficient*(marker_corners[buff_id[i]][0].y - marker_corners[buff_id[i]][3].y)+roi_p0.y;

        roi_p1.x = radial_coefficient*(marker_corners[buff_id[i]][1].x - marker_corners[buff_id[i]][2].x)+roi_p1.x;
        roi_p1.y = radial_coefficient*(marker_corners[buff_id[i]][1].y - marker_corners[buff_id[i]][2].y)+roi_p1.y;

        roi_p2.x = radial_coefficient*(marker_corners[buff_id[i]][2].x - marker_corners[buff_id[i]][1].x)+roi_p2.x;
        roi_p2.y = radial_coefficient*(marker_corners[buff_id[i]][2].y - marker_corners[buff_id[i]][1].y)+roi_p2.y;

        roi_p3.x = radial_coefficient*(marker_corners[buff_id[i]][3].x - marker_corners[buff_id[i]][0].x)+roi_p3.x;
        roi_p3.y = radial_coefficient*(marker_corners[buff_id[i]][3].y - marker_corners[buff_id[i]][0].y)+roi_p3.y;

        center_x = (roi_p0.x+roi_p1.x+roi_p2.x+roi_p3.x)/4;
        center_y = (roi_p0.y+roi_p1.y+roi_p2.y+roi_p3.y)/4;


        mask.setTo(0);
        roi_position.push_back(roi_p0);
        roi_position.push_back(roi_p1);
        roi_position.push_back(roi_p2);
        roi_position.push_back(roi_p3);


        min_x = roi_p0.x>0?roi_p0.x:0;
        min_y = roi_p0.y>0?roi_p0.y:0;
        max_x = roi_p0.x>mask_size.width-1?mask_size.width-1:roi_p0.x;
        max_y = roi_p0.y>mask_size.height-1?mask_size.height-1:roi_p0.y;

        for(int m = 1;m<4;m++){
            if(roi_position[m].x<min_x)
                min_x = roi_position[m].x>0?roi_position[m].x:0;
            if(roi_position[m].y<min_y)
                min_y = roi_position[m].y>0?roi_position[m].y:0;
            if(roi_position[m].x>max_x)
                max_x = roi_position[m].x>mask_size.width-1?mask_size.width-1:roi_position[m].x;
            if(roi_position[m].y>max_y)
                max_y = roi_position[m].y>mask_size.height-1?mask_size.height-1:roi_position[m].y;
        };
        std::vector<std::vector<cv::Point>> contours;
        contours.push_back(roi_position);
        cv::fillPoly(mask, contours, 1);
        has_roi = true;
    }else{
        mask.setTo(0);
        std::cout << "No aruco marker detected " << "\n";
        has_roi = false;
    }
}
