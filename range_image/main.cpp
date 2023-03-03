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


void loadLidarDataThread()
{
	std::cout<<"----------- frame -----------"<<std::endl;


	slam::FileRecord record( "/home/arm/Test/3d_lidar_slam/data/kitti_loop_loam" );
	slam::PointCloud<slam::Point3F> point_cloud;
	slam::Visualize visual;
	visual.initWindow( "window" );

	int count = 0;
        while( !record.endOfFile() ){
		count ++;

	
		record.readOneFrame( point_cloud );

		std::cout<<"point_cloud size = "<<point_cloud.points.size()<<std::endl;
		//visual.displayOnePointCloud( point_cloud );
		//visual.spinWindow();	

	//	slam::RangeImage<float>::RangeImageType range_image;
	//	slam::RangeImage<float>::generateRangeImage( point_cloud, range_image );	
	//	visual.displayOneRangeImage( range_image, 0 );

		slam::PointCloud<slam::Point3F> point_cloud_plane, point_cloud_corner;
		slam::RangeImageFeature range_image_feature;	
		slam::extractFeaturesFromCloud( range_image_feature, point_cloud, point_cloud_plane, point_cloud_corner );
	
		std::cout<<"plane feature point cloud size = "<<point_cloud_plane.points.size()<<std::endl;
        	std::cout<<"corner feature point cloud size = "<<point_cloud_corner.points.size()<<std::endl;	
	
		visual.displayOnePointCloud( point_cloud_plane, slam::PointColor::Yellow );
		visual.spinWindow();
	
		visual.displayOnePointCloud( point_cloud_corner, slam::PointColor::Green );
                visual.spinWindow();
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
