#include "file_record.h"
#include <unistd.h>
#include "visualize.h"
#include <mutex>
#include <thread>
#include "feature_extract.h"

// 
slam::FileRecord record( "../data/3d_lidar_record_file" );
slam::PointCloud<slam::Point3F> point_cloud;
slam::Visualize visual;

void loadLidarDataThread()
{
	std::cout<<"----------- frame -----------"<<std::endl;

        record.readOneFrame( point_cloud );

       	std::cout<<"time_stamp = "<<point_cloud.time_stamp<<std::endl;
        std::cout<<"cloud width = "<<point_cloud.width<<std::endl;
        std::cout<<"cloud height = "<<point_cloud.height<<std::endl;

       	for( int i = 0; i < 10; i ++ ){
       		std::cout<<"( "<<point_cloud.points[i].x <<", "<<
                point_cloud.points[i].y<<", "<<
                point_cloud.points[i].z <<" )"<<" ";
        }
        std::cout<<std::endl;

	visual.initWindow( "window" );
	visual.displayOnePointCloud( point_cloud );
	visual.spinWindow();

	std::cout<<"file end !"<<std::endl;
	record.closeFile();
	visual.destroyWindow();
}


int main()
{
	std::cout<<"------------------ LIDAR DATA READ ------------------"<<std::endl;
	
	loadLidarDataThread();

	return 0;
}
