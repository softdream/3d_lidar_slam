#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>

#include <Eigen/Dense>

#include "point_cloud.h"


// ------------------------------------------------- //
std::ofstream outfile;
// ------------------------------------------------- //


int main (int argc, char** argv)
{
 	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
 
  	if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/arm/Test/3d_lidar_slam/convert_pcd/rabbit.pcd", *cloud) == -1) {
    		PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    		return (-1);
  	}
  	
	std::cout << "Loaded "
            	<< cloud->width * cloud->height
            	<< " data points from test_pcd.pcd with the following fields: "
            	<< std::endl;
  
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_pointcloud(new pcl::PointCloud<pcl::PointXYZ>);
 	pcl::VoxelGrid<pcl::PointXYZ> sor;

    	sor.setInputCloud(cloud);

    	sor.setLeafSize(0.5, 0.5, 0.5);

    	sor.filter(*filtered_pointcloud);
	std::cout<<"filtered point cloud size = "<<filtered_pointcloud->width * filtered_pointcloud->height<<std::endl;


	// storage
	outfile.open( "./rabbit", std::ios::app );
        if( !outfile.is_open() ){
                std::cerr<<"open file failed ..."<<std::endl;
                exit(-1);
        }

	std::cout<<"filtered_pointcloud->header.stamp = "<<filtered_pointcloud->header.stamp<<std::endl;

        slam::PointCloud<slam::Point3F> point_cloud( filtered_pointcloud->header.stamp, filtered_pointcloud->width, filtered_pointcloud->height );

        outfile.write( reinterpret_cast<char *>( &point_cloud.time_stamp ), sizeof( point_cloud.time_stamp ) );
        outfile.write( reinterpret_cast<char *>( &point_cloud.width ), sizeof( point_cloud.width ) );
        outfile.write( reinterpret_cast<char *>( &point_cloud.height ), sizeof( point_cloud.height ) );

        for ( size_t i = 0; i < filtered_pointcloud->points.size(); i ++ ) {
                point_cloud.points[i].x = filtered_pointcloud->points[i].x;
                point_cloud.points[i].y = filtered_pointcloud->points[i].y;
                point_cloud.points[i].z = filtered_pointcloud->points[i].z;
        }

	outfile.write( reinterpret_cast<char *>( point_cloud.points.data() ), point_cloud.points.size() * sizeof( slam::Point3F ) );


	// transform
	Eigen::Matrix3f R;
	R << 0.98106,  -0.168918,  0.0948021,
  	     0.172987,    0.98425, -0.0364272,
	    -0.0871557,  0.0521368,   0.994829;
	Eigen::Vector3f T( 0.2, 0.2, 0.2 );

	slam::PointCloud<slam::Point3F> point_cloud_transed( point_cloud.time_stamp, point_cloud.width, point_cloud.height );
	outfile.write( reinterpret_cast<char *>( &point_cloud_transed.time_stamp ), sizeof( point_cloud_transed.time_stamp ) );
        outfile.write( reinterpret_cast<char *>( &point_cloud_transed.width ), sizeof( point_cloud_transed.width ) );
        outfile.write( reinterpret_cast<char *>( &point_cloud_transed.height ), sizeof( point_cloud_transed.height ) );

	for ( size_t i = 0; i < point_cloud.points.size(); i ++ ) {
		Eigen::Vector3f pt( point_cloud.points[i].x, point_cloud.points[i].y, point_cloud.points[i].z );

		Eigen::Vector3f pt_transed = R * pt + T;

		point_cloud_transed.points[i].x = pt_transed(0);
		point_cloud_transed.points[i].y = pt_transed(1);
		point_cloud_transed.points[i].z = pt_transed(2);
        }

        outfile.write( reinterpret_cast<char *>( point_cloud_transed.points.data() ), point_cloud_transed.points.size() * sizeof( slam::Point3F ) );


	// end storage

	pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");//创建一个显示窗口
  
	viewer.showCloud(filtered_pointcloud);					//显示点云
  
	while(!viewer.wasStopped()) {  
  	
	}
 
  	return (0);
}
