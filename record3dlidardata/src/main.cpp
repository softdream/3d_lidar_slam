#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_types.h>                 //pcl点云格式头文件
#include <pcl_conversions/pcl_conversions.h> //转换
#include <pcl/point_cloud.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <iomanip>

#include "point_cloud.h"

// ------------------------------------------------- //
std::ofstream outfile;
// ------------------------------------------------- //

void laserCallback( const sensor_msgs::PointCloud2::ConstPtr &scan )
{
	std::cout<<"--------------------------------"<<std::endl;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pcl_xyz( new pcl::PointCloud<pcl::PointXYZ> );
	pcl::fromROSMsg(*scan, *cloud_pcl_xyz);

	//std::cout<<"cloud pcl size = "<<cloud_pcl_xyz.points.size()<<std::endl;
	//std::cout<<"cloud pcl width = "<<cloud_pcl_xyz.width<<std::endl;
	//std::cout<<"cloud pcl height = "<<cloud_pcl_xyz.height<<std::endl;
	//std::cout<<"cloud pcl timestamp = "<<cloud_pcl_xyz.header.stamp<<std::endl;

	for(int i=0; i < 10; i++){
		std::cout << cloud_pcl_xyz->points[i] << std::endl;
	}

	/*pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> sor;
        sor.setInputCloud( cloud_pcl_xyz );

        sor.setLeafSize(0.9, 0.9, 0.9);

        sor.filter( *filtered_cloud );
        std::cout<<"filtered point cloud size = "<<filtered_cloud->points.size()<<std::endl;
	*/
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud = cloud_pcl_xyz;

	
	// storage
	slam::PointCloud<slam::Point3f> point_cloud( filtered_cloud->header.stamp, filtered_cloud->width, filtered_cloud->height );
	
	outfile.write( reinterpret_cast<char *>( &point_cloud.time_stamp ), sizeof( point_cloud.time_stamp ) );
        outfile.write( reinterpret_cast<char *>( &point_cloud.width ), sizeof( point_cloud.width ) );
	outfile.write( reinterpret_cast<char *>( &point_cloud.height ), sizeof( point_cloud.height ) );
	

	for( int i = 0; i < filtered_cloud->points.size(); i ++ ){
		point_cloud.points[i].x = filtered_cloud->points[i].x;
		point_cloud.points[i].y = filtered_cloud->points[i].y;
		point_cloud.points[i].z = filtered_cloud->points[i].z;
	}	

	outfile.write( reinterpret_cast<char *>( point_cloud.points.data() ), point_cloud.points.size() * sizeof( slam::Point3f ) );
}



int main( int argc, char **argv )
{
	std::cout<<"Program Begins ..."<<std::endl;
	ros::init( argc, argv, "ReadData" );

	ros::NodeHandle n;


	//ros::Subscriber scan_sub = n.subscribe<sensor_msgs::PointCloud2>("/points_raw", 1, laserCallback);
	ros::Subscriber scan_sub = n.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 1, laserCallback);
	

	// ---------- open the file --------------//
	outfile.open( "./kitti_small_loam", std::ios::app );
	if( !outfile.is_open() ){
		std::cerr<<"open file failed ..."<<std::endl;
		exit(-1);
	}


	ros::spin();

	//outfile.close();

	return 0;
}
