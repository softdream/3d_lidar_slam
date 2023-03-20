#include "file_record.h"
#include <unistd.h>

int main()
{
	std::cout<<"------------------ LIDAR DATA READ ------------------"<<std::endl;
	slam::FileRecord record( "/home/riki/Test/3d_lidar_slam/data/3d_lidar_record_file" ); 

	int count = 0;
	while( !record.endOfFile() ){
	//while( count < 1 ){
		std::cout<<"----------- frame -----------"<<std::endl;

		slam::PointCloud<slam::Point3F> point_cloud;
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

		count ++;
		usleep( 50000 );
	}

	record.closeFile();

	return 0;
}
