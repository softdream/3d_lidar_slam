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

void pointCloudTransform( slam::PointCloud<slam::Point3F>& cloud, const Eigen::Matrix4f& transform )
{
	Eigen::Matrix3f R = transform.block<3, 3>(0, 0);
        Eigen::Vector3f T = transform.block<3, 1>(0, 3);

	for ( size_t i = 0; i < cloud.points.size(); i ++ ) {
                Eigen::Vector3f pt( cloud.points[i].x, cloud.points[i].y, cloud.points[i].z );

                pt = R * pt + T;

                cloud.points[i].x = pt[0];
                cloud.points[i].y = pt[1];
                cloud.points[i].z = pt[2];
        }
}

void loadLidarDataThread()
{
	std::cout<<"----------- frame -----------"<<std::endl;

	outfile.open( "./3d_slam_pose3.txt", std::ios::out );
	if( !outfile.is_open() ) {
		std::cerr<<"Can not open the file !"<<std::endl;
		exit( -1 );
	}

	slam::FileRecord record( "/home/arm/Test/3d_lidar_slam/data/kitti_loop_loam" );
	slam::PointCloud<slam::Point3F> point_cloud;
	slam::Visualize visual;
	visual.initWindow( "window" );

	slam::CornerPlannerFeature corner_planner_feature;

	slam::PointCloud<slam::Point3F> pre_point_cloud_plane, pre_point_cloud_corner;

	//slam::PointCloud<slam::Point3F> pre_point_cloud;

	slam::PointCloud<slam::Point3F> mapping_cloud;

	Eigen::Matrix<float, 4, 4> pose = Eigen::Matrix<float, 4, 4>::Identity();

	int count = 0;
        while( !record.endOfFile() ){
		count ++;

		record.readOneFrame( point_cloud );

		std::cout<<"point_cloud size = "<<point_cloud.points.size()<<std::endl;


		// 2. features extraction
		slam::PointCloud<slam::Point3F> point_cloud_plane, point_cloud_corner;
	
		slam::extractFeaturesFromCloud( corner_planner_feature, point_cloud, point_cloud_plane, point_cloud_corner ); 

		std::cout<<"plane feature point cloud size = "<<point_cloud_plane.points.size()<<std::endl;
		std::cout<<"corner feature point cloud size = "<<point_cloud_corner.width<<std::endl;
		std::cout<<std::endl;
			
		//visual.displayOnePointCloud( point_cloud_corner );
                //visual.spinWindowOnce(100);

		if( count == 1 ){
                        pre_point_cloud_plane = point_cloud_plane;
                        pre_point_cloud_corner = point_cloud_corner;

			//pre_point_cloud = point_cloud;
                        continue;
                }


		// scan match

		auto t1 = std::chrono::steady_clock::now();

		slam::Point2PointICP<float> p2l_icp;
		Eigen::Matrix<float, 4, 4> transform = Eigen::Matrix<float, 4, 4>::Identity();
		//slam::scanMatch( p2l_icp, pre_point_cloud, point_cloud, transform, 20 );
		//slam::scanMatch( p2l_icp, pre_point_cloud_plane, point_cloud_plane, transform, 15 );
		slam::scanMatch( p2l_icp, pre_point_cloud_corner, point_cloud_corner, transform, 10 );
		std::cout<<"estimated transformation : "<<std::endl<<transform<<std::endl;
		
		slam::Point2PlaneICP<float> p2p_icp;
                //Eigen::Matrix<float, 4, 4> transform = Eigen::Matrix<float, 4, 4>::Identity();
                slam::scanMatch( p2p_icp, pre_point_cloud_plane, point_cloud_plane, transform, 5);
                std::cout<<"point to plane estimated transformation : "<<std::endl<<transform<<std::endl;


		auto t2 = std::chrono::steady_clock::now();
		double dr_ms = std::chrono::duration<double,std::milli>(t2-t1).count();
		std::cout<<"duration : "<<dr_ms<<std::endl;
		

		// update the pose
		pose = pose * transform;
		outfile <<"pose "<< pose(0, 3)<<" "<<pose(1, 3)<<std::endl;

		// update 
		pre_point_cloud_plane = point_cloud_plane;
		pre_point_cloud_corner = point_cloud_corner;
	
		//pre_point_cloud = point_cloud;
		

		// mapping
		if( count % 10 == 0 ) {
			pointCloudTransform( point_cloud_corner, pose );

			mapping_cloud.points.insert( mapping_cloud.points.end(), point_cloud_corner.points.begin(), point_cloud_corner.points.end() );
			visual.displayOnePointCloud( mapping_cloud );
                	visual.spinWindowOnce(100);
		}
	}

	visual.spinWindow();

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
