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

	slam::FileRecord record( "/home/arm/Test/3d_lidar_slam/scan_match_test2/rabbit2" );
	slam::PointCloud<slam::Point3F> first_point_cloud, second_point_cloud;
	slam::Visualize visual;

	// 1. first point cloud
        record.readOneFrame( first_point_cloud );
	std::cout<<"first point cloud size = "<<first_point_cloud.points.size()<<std::endl;

	visual.initWindow( "window" );
	visual.displayOnePointCloud( first_point_cloud );
	visual.spinWindow();


	// 2. second point cloud
	record.readOneFrame( second_point_cloud );
        visual.displayOnePointCloud( second_point_cloud );
        visual.spinWindow();


	// 5. scan match test
	std::cout<<"------------------------------- SCAN MATCH TEST --------------------------------"<<std::endl;
	/*slam::Point2PointICP<float> p2l_icp;
	Eigen::Matrix<float, 4, 4> transform = Eigen::Matrix<float, 4, 4>::Identity();

	slam::scanMatch( p2l_icp, first_point_cloud, second_point_cloud, transform, 50 );

        std::cout<<"estimated transformation by p2l icp: "<<std::endl<<transform<<std::endl;
	*/

	slam::Point2PlaneICP<float, slam::ThirdNormalPolicy> p2p_icp;
	Eigen::Matrix<float, 4, 4> transform = Eigen::Matrix<float, 4, 4>::Identity();

	slam::scanMatch( p2p_icp, first_point_cloud, second_point_cloud, transform, 4 );

        std::cout<<"estimated transformation by p2p icp: "<<std::endl<<transform<<std::endl;
	

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
