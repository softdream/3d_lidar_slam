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
#include "icp_manual.h"


int main (int argc, char** argv)
{
	// 1. first point cloud
 	pcl::PointCloud<pcl::PointXYZ>::Ptr first_cloud (new pcl::PointCloud<pcl::PointXYZ>);
 
  	if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/arm/Test/pcl_icp_test/rabbit3.pcd", *first_cloud) == -1) {
    		PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    		return (-1);
  	}
  	
	std::cout << "Loaded "
            	<< first_cloud->width * first_cloud->height
            	<< " data points from test_pcd.pcd with the following fields: "
            	<< std::endl;
  	
	// 2. second point cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr second_cloud (new pcl::PointCloud<pcl::PointXYZ>);

        if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/arm/Test/pcl_icp_test/rabbit4.pcd", *second_cloud) == -1) {
                PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
                return (-1);
        }

        std::cout << "Loaded "
                << second_cloud->width * second_cloud->height
                << " data points from test_pcd.pcd with the following fields: "
                << std::endl;
	
	// 3. scan match test
	std::cout<<"--------------------- SCAN MATCH TEST ------------------------"<<std::endl;
	/*pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
 
	auto t1 = std::chrono::steady_clock::now();

	icp.setMaxCorrespondenceDistance(100);
    	icp.setMaximumIterations(100);//迭代次数
    	icp.setTransformationEpsilon(1e-6);//先前转换和当前估计转换（即两次位姿转换）之间的 epsilon（差异）
    	icp.setEuclideanFitnessEpsilon(1e-6);//欧几里得平方误差的总和
    	icp.setRANSACIterations(0);// 设置RANSAC运行次数    
    	icp.setInputCloud(second_cloud);
    	icp.setInputTarget(first_cloud);
	
	pcl::PointCloud<pcl::PointXYZ> cloud_in_after_icp ;
    	icp.align(cloud_in_after_icp);

	auto t2 = std::chrono::steady_clock::now();
	double dr_ms = std::chrono::duration<double,std::milli>(t2-t1).count();
	std::cout<<dr_ms<<" "<<std::endl;

	std::cout << "has converged:" << icp.hasConverged() << " score: " <<    icp.getFitnessScore() << std::endl;
    	std::cout << icp.getFinalTransformation() << std::endl;//获得最后的变换矩阵
	*/

	Eigen::Matrix<float, 4, 4> transform = Eigen::Matrix<float, 4, 4>::Identity();

	test::Point2PointICP icp_m;
	icp_m.scanMatch( first_cloud, *second_cloud, transform, 50 );
	//icp_m.scanMatchSVD( first_cloud, *second_cloud, transform, 20 );

	std::cout<<"estimated transformation by p2p icp: "<<std::endl<<transform<<std::endl;


	pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
  
	viewer.showCloud(second_cloud);
  
	while(!viewer.wasStopped()) {  
  	
	}
 
  	return (0);
}
