#include "file_record.h"
#include <unistd.h>
#include "visualize.h"
#include <mutex>
#include <thread>
#include "feature_extract.h"

#include "scan_match.h"

void loadLidarDataThread()
{
	std::cout<<"----------- frame -----------"<<std::endl;

	slam::FileRecord record( "/home/riki/Test/3d_lidar_slam/3d_lidar_slam/scan_match_test2/rabbit" );
	slam::PointCloud<slam::Point3F> first_point_cloud, second_point_cloud;
	slam::Visualize visual;

	// 1. first point cloud
        record.readOneFrame( first_point_cloud );
	std::cout<<"first point cloud size = "<<first_point_cloud.points.size()<<std::endl;

	visual.initWindow( "window" );
	visual.displayOnePointCloud( first_point_cloud );
	visual.spinWindow();



	// 5. scan match test
	std::cout<<"------------------------------- SCAN MATCH TEST --------------------------------"<<std::endl;
	std::cout<<"end !"<<std::endl;
	record.closeFile();
	visual.destroyWindow();
}


int main()
{
	std::cout<<"------------------ LIDAR DATA READ ------------------"<<std::endl;
	
	loadLidarDataThread();

	return 0;
}
