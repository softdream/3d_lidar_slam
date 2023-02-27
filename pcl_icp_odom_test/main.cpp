#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>

#include <Eigen/Dense>

#include <chrono>


#include "point_cloud.h"
#include "file_record.h"

std::ofstream outfile;

void convert2PclCloud( const slam::PointCloud<slam::Point3F>& point_cloud, 
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud )
{
	cloud->header.stamp = point_cloud.time_stamp;
	cloud->width = point_cloud.width;
	cloud->height = point_cloud.height;

	for ( int i = 0; i < point_cloud.points.size(); i ++ ) {
		cloud->points.push_back( { point_cloud.points[i].x, point_cloud.points[i].y, point_cloud.points[i].z } );
	}
}

void pointCloudTransform( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const Eigen::Matrix4f& transform )
{
	Eigen::Matrix3f R = transform.block<3, 3>(0, 0);
	Eigen::Vector3f T = transform.block<3, 1>(0, 3);

	for ( size_t i = 0; i < cloud->points.size(); i ++ ) {
		Eigen::Vector3f pt( cloud->points[i].x, cloud->points[i].y, cloud->points[i].z );

		pt = R * pt + T;

		cloud->points[i].x = pt[0];
		cloud->points[i].y = pt[1];
		cloud->points[i].z = pt[2];
	}
}

void pointCloudTransform( const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, 
			  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out,
			  const Eigen::Matrix4f& transform )
{
        Eigen::Matrix3f R = transform.block<3, 3>(0, 0);
        Eigen::Vector3f T = transform.block<3, 1>(0, 3);

        for ( size_t i = 0; i < cloud_in->points.size(); i ++ ) {
                Eigen::Vector3f pt( cloud_in->points[i].x, cloud_in->points[i].y, cloud_in->points[i].z );

                pt = R * pt + T;
		cloud_out->push_back( { pt(0), pt(1), pt(2) } );
        }
}


int main (int argc, char** argv)
{
	outfile.open( "./3d_slam_pose2.txt", std::ios::out );
        if( !outfile.is_open() ) {
                std::cerr<<"Can not open the file !"<<std::endl;
                exit( -1 );
        }

	slam::FileRecord record( "/home/arm/Test/3d_lidar_slam/data/kitti_loop_loam" );

	pcl::visualization::CloudViewer viewer("Viewer");

	slam::PointCloud<slam::Point3F> point_cloud;
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr pre_cloud( new pcl::PointCloud<pcl::PointXYZ> );

	Eigen::Matrix<float, 4, 4> pose = Eigen::Matrix<float, 4, 4>::Identity();
	Eigen::Matrix4f last_pose = pose;


	pcl::PointCloud<pcl::PointXYZ>::Ptr mapping_cloud( new pcl::PointCloud<pcl::PointXYZ> );

	int count = 0;
        while( !record.endOfFile() ){
                count ++;

                record.readOneFrame( point_cloud );
	
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
		// 1. 
		convert2PclCloud( point_cloud, cloud );

		// 2. down sampling
		pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::VoxelGrid<pcl::PointXYZ> sor;
		sor.setInputCloud( cloud );

		sor.setLeafSize(0.9, 0.9, 0.9);
    
        	sor.filter( *filtered_cloud );
		std::cout<<"filtered point cloud size = "<<filtered_cloud->points.size()<<std::endl;

		//viewer.showCloud( filtered_cloud );

		// 3. icp scan match
		if( count == 1 ) {
			*pre_cloud = *filtered_cloud;
			continue;
		}

		pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
		icp.setMaxCorrespondenceDistance(100);
        	icp.setMaximumIterations(100);//迭代次数
        	icp.setTransformationEpsilon(1e-6);
       	 	icp.setEuclideanFitnessEpsilon(1e-6);//欧几里得平方误差的总和
        	icp.setRANSACIterations(0);// 设置RANSAC运行次数    

		icp.setInputCloud(filtered_cloud);
                icp.setInputTarget(pre_cloud);

        	pcl::PointCloud<pcl::PointXYZ> cloud_in_after_icp ;
        	icp.align(cloud_in_after_icp);

		std::cout << "has converged:" << icp.hasConverged() << " score: " <<    icp.getFitnessScore() << std::endl;
		std::cout << icp.getFinalTransformation() << std::endl;

		Eigen::Matrix4f transform = icp.getFinalTransformation();

		// update the pose
		pose = last_pose * transform;
		last_pose = pose;

		outfile <<"pose "<< pose(0, 3)<<" "<<pose(1, 3)<<std::endl;
	

		*pre_cloud = *filtered_cloud;

		if( count % 10 == 0 ) {
			//pointCloudTransform( filtered_cloud, pose );
			//*mapping_cloud += *filtered_cloud;
			pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ>);
			//pcl::transformPointCloud( *filtered_cloud, *transformed_cloud, pose );
			pointCloudTransform( filtered_cloud, pose );

			//*mapping_cloud += *transformed_cloud;
			*mapping_cloud += *filtered_cloud;

			viewer.showCloud( mapping_cloud );
		}

		usleep(100000);
	}
 
  	return (0);
}
