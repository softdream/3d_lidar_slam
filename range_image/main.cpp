#include "range_image.h"
#include "point_cloud.h"
#include "file_record.h"
#include "opencv2/opencv.hpp"

void print(slam::PointCloud<slam::Point3f> &point_cloud) {
	std::cout<<"source point cloud : ";
       	std::cout<<"time_stamp = "<<point_cloud.time_stamp<< " ";
        std::cout<<"cloud width = "<<point_cloud.width<< " ";
        std::cout<<"cloud height = "<<point_cloud.height<<std::endl;
}


int main(int argc, char** argv) {
	slam::FileRecord record("/root/share/test_data/3d_lidar_record_file");
	slam::PointCloud<slam::Point3f> point_cloud;

	// 1. source point cloud
    record.readOneFrame( point_cloud );
	record.readOneFrame( point_cloud );
	record.readOneFrame( point_cloud );	
	cv::Mat range_image;
	slam::generateRangeImage(point_cloud, range_image);
	cv::imshow("range image", range_image);
	cv::waitKey(100);
	cv::imwrite("../range_image1.jpg", range_image);

	while(record.readOneFrame(point_cloud) && false) {
		slam::generateRangeImage(point_cloud, range_image);
		cv::imshow("range image", range_image);
		cv::waitKey(100);
		cv::imwrite("./range_image.jpg", range_image);
	}
	
	
	return 0;
}