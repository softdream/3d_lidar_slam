#ifndef __RANGE_IMAGE_H
#define __RANGE_IMAGE_H

#include "point_cloud.h"
#include "config.h"
#include <opencv2/opencv.hpp>
namespace slam
{
/***
 * 基于这样的假设：
 * 雷达的坐标系 X轴正方向为车辆前进方向，Y轴正方向指向左侧， Z轴正方向指向天空
 * 
*/


template<typename PointType>
void generateRangeImage(const PointCloud<PointType> &point_cloud_in, cv::Mat &range_mat) {
    Config config;
    range_mat = cv::Mat(config.N_SCANS, config.N_HORIZON, CV_8UC3, cv::Scalar(FLT_MAX, FLT_MAX, FLT_MAX));
    float vertical_angle, horizon_angle, range;
    size_t row_ind, column_ind, index, cloud_size;
    PointType this_point;
    cloud_size = point_cloud_in.points.size();
    
    for (size_t i = 0; i < cloud_size; ++i) {
        this_point.x = point_cloud_in.points[i].x;
        this_point.y = point_cloud_in.points[i].y;
        this_point.z = point_cloud_in.points[i].z;

        vertical_angle = std::atan2(this_point.z, std::sqrt(this_point.x * this_point.x + this_point.y * this_point.y)) * 180/M_PI;
        row_ind = -(vertical_angle + config.Angle_Bottom) / config.Angular_Resolution_Vertical;

        if (row_ind < 0 || row_ind >= config.N_SCANS)
            continue;
        horizon_angle = std::atan2(this_point.y, this_point.x) * 180 / M_PI;
        column_ind = -std::round((horizon_angle - 0.0)/config.Angular_Resolution_Horizon) + config.N_HORIZON / 2;

        if (column_ind >= config.N_HORIZON)
            column_ind -= config.N_HORIZON;
        if (column_ind < 0 || column_ind >= config.N_HORIZON) 
            continue;
        range = std::sqrt(this_point.x * this_point.x + this_point.y * this_point.y + this_point.z * this_point.z);
        if (range < 3) 
            continue;
        cv::Vec3b pix((range / config.Max_Range) * UCHAR_MAX, 0, (range / config.Max_Range) * UCHAR_MAX);
        // if(range > 80) std::cout << range << std::endl;
        range_mat.at<cv::Vec3b>(row_ind, column_ind) = pix;
    }
    // return range_mat;
}


}
#endif // __RANGE_IMAGE_H