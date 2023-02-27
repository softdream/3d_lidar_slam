#include "file_record.h"
#include <unistd.h>
#include "visualize.h"
#include <mutex>
#include <thread>

#include <chrono>

#include "feature_extract.h"

#include "scan_match.h"

#include <fstream>

std::ofstream outfile;

void loadLidarDataThread()
{
	std::cout<<"----------- frame -----------"<<std::endl;

	outfile.open( "./3d_slam_pose3.txt", std::ios::out );
	if( !outfile.is_open() ) {
		std::cerr<<"Can not open the file !"<<std::endl;
		exit( -1 );
	}

	slam::FileRecord record( "/home/riki/Test/3d_lidar_slam/data/3d_lidar_record_file4" );
	slam::PointCloud<slam::Point3F> point_cloud;
	slam::Visualize visual;

	slam::CornerPlannerFeature corner_planner_feature;

	slam::PointCloud<slam::Point3F> pre_point_cloud_plane, pre_point_cloud_corner;

	slam::PointCloud<slam::Point3F> pre_point_cloud;

	Eigen::Matrix<float, 4, 4> pose = Eigen::Matrix<float, 4, 4>::Identity();

	int count = 0;
        while( !record.endOfFile() ){
		count ++;

		record.readOneFrame( point_cloud );

		visual.initWindow( "window" );
		visual.displayOnePointCloud( point_cloud );
		visual.spinWindowOnce(10);

		// 2. features extraction
		slam::PointCloud<slam::Point3F> point_cloud_plane, point_cloud_corner;
	
		slam::extractFeaturesFromCloud( corner_planner_feature, point_cloud, point_cloud_plane, point_cloud_corner ); 

		std::cout<<"plane feature point cloud size = "<<point_cloud_plane.points.size()<<std::endl;
		std::cout<<"corner feature point cloud size = "<<point_cloud_corner.width<<std::endl;
		std::cout<<std::endl;
			

		if( count == 1 ){
                        pre_point_cloud_plane = point_cloud_plane;
                        pre_point_cloud_corner = point_cloud_corner;

			//pre_point_cloud = point_cloud;
                        continue;
                }


		// scan match
		/*slam::Point2PlaneICP<float> p2p_icp;
		Eigen::Matrix<float, 4, 4> transform = Eigen::Matrix<float, 4, 4>::Identity();
		slam::scanMatch( p2p_icp, pre_point_cloud_plane, point_cloud_plane, transform, 7);
		std::cout<<"estimated transformation : "<<std::endl<<transform<<std::endl;
		*/
		slam::Point2PointICP<float> p2l_icp;
		Eigen::Matrix<float, 4, 4> transform = Eigen::Matrix<float, 4, 4>::Identity();
		//slam::scanMatch( p2l_icp, pre_point_cloud, point_cloud, transform, 3 );
		slam::scanMatch( p2l_icp, pre_point_cloud_plane, point_cloud_plane, transform, 3 );
		std::cout<<"estimated transformation : "<<std::endl<<transform<<std::endl;
		
		// update the pose
		pose = pose * transform;
		outfile <<"pose "<< pose(0, 3)<<" "<<pose(1, 3)<<std::endl;

		// update 
		pre_point_cloud_plane = point_cloud_plane;
		pre_point_cloud_corner = point_cloud_corner;
	
		//pre_point_cloud = point_cloud;
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
