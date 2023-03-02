#ifndef __VISUALiZE_H
#define __VISUALIZE_H

#include <opencv2/opencv.hpp>
#include <opencv2/viz.hpp>

#include "point_cloud.h"
#include "range_image.h"

namespace slam
{

typedef enum PointColor_
{
	Red,
	Yellow,
	Green,
	Black,
	Gray, 
	Blue,	
}PointColor;

class Visualize
{
public:
	Visualize()
	{

	}

	~Visualize()
	{

	}

	void initWindow( const std::string& window_name )
	{
		window_ = new cv::viz::Viz3d( window_name );
		window_->setBackgroundColor();
	}

	template<typename T>
	void displayOnePointCloud( const slam::PointCloud<T>& point_cloud, const PointColor& color = Red )
	{
		std::cout<<"----------visualize the point cloud ---------"<<std::endl;
	
		int point_num = point_cloud.points.size();
		cv::Mat point_cloud_mat = cv::Mat::zeros(3, point_num, CV_32FC3);
	
		for(int row = 0; row < point_num; row ++) {
 
    			point_cloud_mat.ptr<cv::Vec3f>(0)[row][0] = point_cloud.points[row].x;
 
    			point_cloud_mat.ptr<cv::Vec3f>(0)[row][1] = point_cloud.points[row].y;
 
    			point_cloud_mat.ptr<cv::Vec3f>(0)[row][2] = point_cloud.points[row].z;
 
		}
	
		switch ( color ) {
			case Red : { cv::viz::WCloud cloud( point_cloud_mat, cv::viz::Viz3d::Color::red() ); window_->showWidget("cloud", cloud); break; }
			case Yellow : { cv::viz::WCloud cloud( point_cloud_mat, cv::viz::Viz3d::Color::yellow() ); window_->showWidget("cloud", cloud); break; }
			case Green : { cv::viz::WCloud cloud( point_cloud_mat, cv::viz::Viz3d::Color::green() ); window_->showWidget("cloud", cloud); break; }
			case Black : { cv::viz::WCloud cloud( point_cloud_mat, cv::viz::Viz3d::Color::black() ); window_->showWidget("cloud", cloud); break; }
			case Gray : { cv::viz::WCloud cloud( point_cloud_mat, cv::viz::Viz3d::Color::gray() ); window_->showWidget("cloud", cloud); break; }
			case Blue : { cv::viz::WCloud cloud( point_cloud_mat, cv::viz::Viz3d::Color::blue() ); window_->showWidget("cloud", cloud); break; }
			default : break;			
		}
		
		
	}

	void destroyWindow()
	{
		delete window_;
	}	

	void spinWindow()
	{
		return window_->spin();
	}

	void spinWindowOnce( const int ms, const bool flag = false )
	{
		return window_->spinOnce( ms, flag );
	}

	bool isStopped()
	{
		return window_->wasStopped();
	}


	// for range image 
	template<typename RangeImageType>
	void displayOneRangeImage( const RangeImageType& range_image, const int ms = 0 )
	{	
		int width = range_image.mat.cols();
		int height = range_image.mat.rows();

		cv::Mat image = cv::Mat( height, width, CV_8UC3 );

		for ( size_t i = 0; i < height; i ++ ) {
			for ( size_t j = 0; j < width; j ++ ) {
				int range_int = static_cast<int>( range_image.mat( i, j ) * 2000 );
				char data[3] = {0};
				data[0] |= range_int;
				data[1] |= ( range_int >> 8 );
				data[2] |= ( range_int >> 16 );
				image.at<cv::Vec3b>(i, j)[0] = data[2]; 
				image.at<cv::Vec3b>(i, j)[1] = data[1]; 
				image.at<cv::Vec3b>(i, j)[2] = data[0];  

				if ( image.at<cv::Vec3b>(i, j)[0] == 255 && image.at<cv::Vec3b>(i, j)[1] == 255 && image.at<cv::Vec3b>(i, j)[2] == 255 ) {
					std::cout<<"( "<<i <<", "<<j<<" ) : "<<range_image.mat( i, j )<<", "<<range_int<<std::endl;
				
				}
				
			}
		}

		cv::imshow("range image", image);
		cv::waitKey( ms );
	}

private:
	
	cv::viz::Viz3d* window_;
};

}

#endif
