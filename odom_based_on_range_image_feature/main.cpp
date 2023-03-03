#include "file_record.h"
#include <unistd.h>
#include "visualize.h"
#include <mutex>
#include <thread>

#include <chrono>

#include "feature_extract.h"

#include "scan_match.h"

#include <fstream>

#include "range_image_feature.h"

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

	slam::RangeImageFeature range_image_feature;

	slam::PointCloud<slam::Point3F> pre_point_cloud_corner;

	slam::PointCloud<slam::Point3F> mapping_cloud;

        Eigen::Matrix<float, 4, 4> pose = Eigen::Matrix<float, 4, 4>::Identity();

	int count = 0;
        while( !record.endOfFile() ){
		count ++;
	
		// 1. data read
		record.readOneFrame( point_cloud );

		std::cout<<"point_cloud size = "<<point_cloud.points.size()<<std::endl;

		// 2. feature extraction
		slam::PointCloud<slam::Point3F> point_cloud_plane, point_cloud_corner;
		slam::extractFeaturesFromCloud( range_image_feature, point_cloud, point_cloud_corner );
	
        	std::cout<<"corner feature point cloud size = "<<point_cloud_corner.points.size()<<std::endl;	
		
		// 3. scan match
		if ( count == 1 ) {
                        pre_point_cloud_corner = point_cloud_corner;

			continue;
		} 

		auto t1 = std::chrono::steady_clock::now();
		slam::Point2PointICP<float> point2point_icp;
                Eigen::Matrix<float, 4, 4> transform = Eigen::Matrix<float, 4, 4>::Identity();
		slam::scanMatch( point2point_icp, pre_point_cloud_corner, point_cloud_corner, transform, 20 );
		std::cout<<"estimated transformation : "<<std::endl<<transform<<std::endl;


		auto t2 = std::chrono::steady_clock::now();
                double dr_ms = std::chrono::duration<double,std::milli>(t2-t1).count();
                std::cout<<"duration : "<<dr_ms<<std::endl;

		// 4. update the pose
                pose = pose * transform;
                outfile <<"pose "<< pose(0, 3)<<" "<<pose(1, 3)<<std::endl;

                // 5. update
                pre_point_cloud_corner = point_cloud_corner;
	
		// 6. mapping
		if( count % 10 == 0 ) {
                        pointCloudTransform( point_cloud_corner, pose );

                        mapping_cloud.points.insert( mapping_cloud.points.end(), point_cloud_corner.points.begin(), point_cloud_corner.points.end() );
                        visual.displayOnePointCloud( mapping_cloud );
                        visual.spinWindowOnce(100);
                }

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
