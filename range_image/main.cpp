#include "range_image.h"
#include "point_cloud.h"
#include "file_record.h"
#include "opencv2/opencv.hpp"

int main(int argc, char** argv) {
	slam::FileRecord record("/home/khz/code/test_data/3d_lidar_record_file");
	slam::PointCloud<slam::Point3f> point_cloud;

	// 1. source point cloud
    record.readOneFrame( point_cloud );
	record.readOneFrame( point_cloud );
	record.readOneFrame( point_cloud );	

	std::cout<<"source point cloud : "<<std::endl;
       	std::cout<<"time_stamp = "<<point_cloud.time_stamp<<std::endl;
        std::cout<<"cloud width = "<<point_cloud.width<<std::endl;
        std::cout<<"cloud height = "<<point_cloud.height<<std::endl;
	cv::Mat range_image;
	slam::generateRangeImage(point_cloud, range_image);
	cv::imshow("range image", range_image);
	return 0;
}