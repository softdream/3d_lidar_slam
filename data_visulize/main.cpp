#include "file_record.h"
#include <unistd.h>
#include "visualize.h"
#include <mutex>
#include <thread>

// 
slam::FileRecord record( "../data/3d_lidar_record_file" );
slam::PointCloud<slam::Point3f> point_cloud;
std::mutex lidar_mux;
slam::Visualize visual;

void loadLidarDataThread()
{
	int count = 0;
        while( !record.endOfFile() ){
        //while( count < 1 ){
                std::cout<<"----------- frame -----------"<<std::endl;

		lidar_mux.lock();
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


		lidar_mux.unlock();

                count ++;
                usleep( 50000 );
        }

	std::cout<<"file end !"<<std::endl;
	record.closeFile();
}

void liarDataVisualizeThread()
{
	visual.initWindow( "window" );

	while(!visual.isStopped()){
		lidar_mux.lock();
		visual.displayOnePointCloud( point_cloud );
		lidar_mux.unlock();
		//usleep( 100000 );
		visual.spinWindowOnce( 100, false);
	}

	visual.destroyWindow();
}

int main()
{
	std::cout<<"------------------ LIDAR DATA READ ------------------"<<std::endl;
	
	std::thread t1( liarDataVisualizeThread );
	std::thread t2( loadLidarDataThread );

	t1.join();
	t2.join();

	while(1){

	}

	return 0;
}
