#include "file_record.h"
#include <unistd.h>
#include "visualize.h"
#include <mutex>
#include <thread>

#include <chrono>

#include "feature_extract.h"

#include "scan_match.h"

#include <fstream>

#include "range_image.h"

void loadLidarDataThread()
{
	std::cout<<"----------- frame -----------"<<std::endl;


	slam::FileRecord record( "/home/arm/Test/3d_lidar_slam/data/kitti_loop_loam" );
	slam::PointCloud<slam::Point3F> point_cloud;
	slam::Visualize visual;
	visual.initWindow( "window" );

	int count = 0;
        while( !record.endOfFile() ){
		count ++;

		record.readOneFrame( point_cloud );

		std::cout<<"point_cloud size = "<<point_cloud.points.size()<<std::endl;
	
		

	}

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
