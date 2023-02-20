#include "file_record.h"
#include <unistd.h>
#include "visualize.h"
#include <mutex>
#include <thread>
#include "feature_extract.h"

#include "kdtree_point_cloud_adapter.h"

void loadLidarDataThread()
{
	std::cout<<"----------- frame -----------"<<std::endl;

	slam::FileRecord record( "/home/arm/Test/3d_lidar_slam/data/3d_lidar_record_file" );
	slam::PointCloud<slam::Point3F> point_cloud;
	slam::Visualize visual;

	// 1. source point cloud
        record.readOneFrame( point_cloud );
	record.readOneFrame( point_cloud );
	record.readOneFrame( point_cloud );	

	std::cout<<"source point cloud : "<<std::endl;
       	std::cout<<"time_stamp = "<<point_cloud.time_stamp<<std::endl;
        std::cout<<"cloud width = "<<point_cloud.width<<std::endl;
        std::cout<<"cloud height = "<<point_cloud.height<<std::endl;


	visual.initWindow( "window" );
	visual.displayOnePointCloud( point_cloud );
	visual.spinWindow();


	// 2. features extraction
	slam::PointCloud<slam::Point3F> point_cloud_plane, point_cloud_corner;
	std::cout<<"addr point_cloud_plane = "<<&point_cloud_plane<<std::endl;
	std::cout<<"addr point_cloud_corner = "<<&point_cloud_corner<<std::endl;	

	slam::CornerPlannerFeature corner_planner_feature;
	
	slam::extractFeaturesFromCloud( corner_planner_feature, point_cloud, point_cloud_plane, point_cloud_corner ); 

	std::cout<<"plane feature point cloud size = "<<point_cloud_plane.points.size()<<std::endl;
	std::cout<<"corner feature point cloud size = "<<point_cloud_corner.width<<std::endl;

	// knn
	slam::kdtree::KdTreePointCloudType<float> kd_point_cloud( point_cloud_plane );
	slam::kdtree::KdTreeType<float> index( 3, kd_point_cloud, {10} );

	std::vector<size_t> out_ids(5);
        std::vector<float> out_dists_sqr(5);

        nanoflann::KNNResultSet<float> result_set( 5 );
        result_set.init( &out_ids[0], &out_dists_sqr[0] );


        index.findNeighbors( result_set, kd_point_cloud.point2vec(0).data() );

        for( int i = 0; i < out_ids.size(); i ++ ){
                std::cout<<"out_ids = "<<out_ids[i]<<", out dists sqr = "<<out_dists_sqr[i]<<std::endl;
        }



	visual.displayOnePointCloud( point_cloud_plane, slam::PointColor::Yellow );
        visual.spinWindow();

	visual.displayOnePointCloud( point_cloud_corner, slam::PointColor::Green );
        visual.spinWindow();


	std::cout<<"file end !"<<std::endl;
	record.closeFile();
	visual.destroyWindow();
}


int main()
{
	std::cout<<"------------------ LIDAR DATA READ ------------------"<<std::endl;
	
	loadLidarDataThread();

	return 0;
}
