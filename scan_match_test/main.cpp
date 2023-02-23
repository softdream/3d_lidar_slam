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

	slam::FileRecord record( "/home/arm/Test/3d_lidar_slam/data/3d_lidar_record_file" );
	slam::PointCloud<slam::Point3F> first_point_cloud, second_point_cloud;
	slam::Visualize visual;

	slam::CornerPlannerFeature corner_planner_feature;


	// 1. first point cloud
        record.readOneFrame( first_point_cloud );

	visual.initWindow( "window" );
	visual.displayOnePointCloud( first_point_cloud );
	visual.spinWindow();


	// 2. features extraction
	slam::PointCloud<slam::Point3F> first_point_cloud_plane, first_point_cloud_corner;
	
	slam::extractFeaturesFromCloud( corner_planner_feature, first_point_cloud, first_point_cloud_plane, first_point_cloud_corner ); 

	std::cout<<"first point cloud : "<<std::endl;
	std::cout<<"plane feature point cloud size = "<<first_point_cloud_plane.points.size()<<std::endl;
	std::cout<<"corner feature point cloud size = "<<first_point_cloud_corner.width<<std::endl;
	std::cout<<std::endl;

	// 3. second point cloud
	record.readOneFrame( second_point_cloud );
        visual.displayOnePointCloud( second_point_cloud );
        visual.spinWindow();

	// 4. features extraction
        slam::PointCloud<slam::Point3F> second_point_cloud_plane, second_point_cloud_corner;

        slam::extractFeaturesFromCloud( corner_planner_feature, second_point_cloud, second_point_cloud_plane, second_point_cloud_corner );

	std::cout<<"second point cloud : "<<std::endl;
        std::cout<<"plane feature point cloud size = "<<second_point_cloud_plane.points.size()<<std::endl;
        std::cout<<"corner feature point cloud size = "<<second_point_cloud_corner.width<<std::endl;
	std::cout<<std::endl;

	// 5. scan match test
	std::cout<<"------------------------------- SCAN MATCH TEST --------------------------------"<<std::endl;

	slam::Point2PointICP<float> p2l_icp;
	Eigen::Matrix<float, 4, 4> transform;
	transform << 1, 0, 0, 0,
		     0, 1, 0, 0,
		     0, 0, 1, 0,
		     0, 0, 0, 1;

	slam::scanMatch( p2l_icp, first_point_cloud_corner, second_point_cloud_corner, transform );

	std::cout<<"estimated transformation : "<<std::endl<<transform<<std::endl;

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
