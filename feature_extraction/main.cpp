#include "file_record.h"
#include <unistd.h>
#include "visualize.h"
#include <mutex>
#include <thread>
#include "feature_extract.h"


void loadLidarDataThread()
{
	std::cout<<"----------- frame -----------"<<std::endl;

	slam::FileRecord record( "/home/riki/Test/3d_lidar_slam/data/3d_lidar_record_file" );
	slam::PointCloud<slam::Point3F> point_cloud;
	slam::Visualize visual;

	// 1. source point cloud
        record.readOneFrame( point_cloud );
	record.readOneFrame( point_cloud );
	record.readOneFrame( point_cloud );	

	std::cout<<"source point cloud : "<<std::endl;
       	std::cout<<"time_stamp = "<<point_cloud.time_stamp<<std::endl;
        std::cout<<"cloud width = "<<point_cloud.width<<std::endl;
        std::cout<<"cloud height = "<<point_cloud.height<<std::endl;


	visual.initWindow( "window" );
	visual.displayOnePointCloud( point_cloud );
	visual.spinWindow();


	// 2. features extraction
	slam::PointCloud<slam::Point3F> point_cloud_plane, point_cloud_corner;
	std::vector<int> plane_cloud_scan_idx;
	std::cout<<"addr point_cloud_plane = "<<&point_cloud_plane<<std::endl;
	std::cout<<"addr point_cloud_corner = "<<&point_cloud_corner<<std::endl;	
	std::cout<<"addr plane cloud scan idx = "<<&plane_cloud_scan_idx<<std::endl;
	
	slam::CornerPlannerFeature corner_planner_feature;
	
	slam::extractFeaturesFromCloud( corner_planner_feature, point_cloud, point_cloud_plane, point_cloud_corner, plane_cloud_scan_idx ); 

	std::cout<<"plane feature point cloud size = "<<point_cloud_plane.points.size()<<std::endl;
	std::cout<<"corner feature point cloud size = "<<point_cloud_corner.width<<std::endl;
	std::cout<<"plane cloud scan idx vec size = "<<plane_cloud_scan_idx.size()<<std::endl;

	visual.displayOnePointCloud( point_cloud_plane, slam::PointColor::Yellow );
        visual.spinWindow();

	visual.displayOnePointCloud( point_cloud_corner, slam::PointColor::Green );
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
