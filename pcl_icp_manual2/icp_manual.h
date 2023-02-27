#ifndef __ICP_MANUAL_H
#define __ICP_MANUAL_H

#include <pcl/registration/icp.h>    
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>

#include "so3.h"

#include <chrono>

namespace test
{

class Point2PointICP
{
public:
	using CloudTypePtr = typename pcl::PointCloud<pcl::PointXYZ>::Ptr;
	using CloudType = typename pcl::PointCloud<pcl::PointXYZ>;
	
	using TransformationType = typename Eigen::Matrix4f;

	void scanMatch( const CloudTypePtr& first_cloud, 
			const CloudType& second_cloud,
		        TransformationType& transform,
			const int max_iterations )
	{
		int iter = 0;

		kdtree_ptr_.setInputCloud( first_cloud );

		while( iter < max_iterations ) {
			mse_ = 0;
			auto t1 = std::chrono::steady_clock::now();

			CloudType transformed_cloud;
			pcl::transformPointCloud( second_cloud, transformed_cloud, transform );

			for ( size_t i = 0; i < transformed_cloud.points.size(); i ++ ) {
				auto transformed_point = transformed_cloud.points[i];

				std::vector<int> indices(1);
            			std::vector<float> distances(1);
				
				kdtree_ptr_.nearestKSearch( transformed_point, 1, indices, distances );

				auto original_point = second_cloud.points[i];
				auto nearest_point = kdtree_ptr_.getInputCloud()->points[indices[0]];
				mse_ += distances[0];

				if( distances[0] > 1 ) continue;

				Eigen::Vector3f source_pt(original_point.x, original_point.y, original_point.z);
				Eigen::Vector3f transformed_pt(transformed_point.x, transformed_point.y, transformed_point.z);
				Eigen::Vector3f target_pt(nearest_point.x, nearest_point.y, nearest_point.z);
				Eigen::Matrix<float, 3, 6> jacobian = Eigen::Matrix<float, 3, 6>::Zero();
				jacobian.block<3, 3>(0, 0) = -transform.block<3, 3>(0, 0) * slam::SO3::hat(source_pt);
            			jacobian.block<3, 3>(0, 3) = Eigen::Matrix3f::Identity();
				Eigen::Vector3f resiual = transformed_pt - target_pt;

				Hessian_ +=  jacobian.transpose() * jacobian;
			       	B_ +=  -jacobian.transpose() * resiual;	
			}

			mse_ /= static_cast<float>( transformed_cloud.points.size() );

			if (Hessian_.determinant() == 0) { 
				continue;
			}

			Eigen::Matrix<float, 6, 1> delta_x = Hessian_.householderQr().solve(B_);

			Eigen::Vector3f tmp = delta_x.block<3, 1>(0, 0);
			transform.block<3, 3>(0, 0) *= slam::SO3::exp( tmp );
        		transform.block<3, 1>(0, 3) += delta_x.block<3, 1>(3, 0);

			iter ++;

			auto t2 = std::chrono::steady_clock::now();
                        auto duration = std::chrono::duration<double,std::milli>(t2-t1).count();
			std::cout<<"duration : "<<duration<<" ms, mse = "<<mse_<<std::endl;
		}
	}

	void scanMatchSVD( const CloudTypePtr& first_cloud,
                        const CloudType& second_cloud,
                        TransformationType& transform,
                        const int max_iterations )
        {
                int iter = 0;

                kdtree_ptr_.setInputCloud( first_cloud );

                while( iter < max_iterations ) {
			auto t1 = std::chrono::steady_clock::now();

			CloudType transformed_cloud;
                        pcl::transformPointCloud( second_cloud, transformed_cloud, transform );

			std::vector<float> source_points_xyz;
        		std::vector<float> target_points_xyz;

			source_points_xyz.reserve(transformed_cloud.points.size() * 3);  //
        		target_points_xyz.reserve(transformed_cloud.points.size() * 3);

			for( size_t i = 0; i < transformed_cloud.points.size(); i ++ ) {
				auto transformed_point = transformed_cloud.points[i];
				std::vector<int> indices(1);
            			std::vector<float> distances(1);
				
				kdtree_ptr_.nearestKSearch(transformed_point, 1, indices, distances);

				mse_ += distances[0];
				if (distances[0] > 1) continue;
			
				source_points_xyz.push_back(second_cloud.points[i].x);
            			source_points_xyz.push_back(second_cloud.points[i].y);
            			source_points_xyz.push_back(second_cloud.points[i].z);

            			target_points_xyz.push_back(kdtree_ptr_.getInputCloud()->points[indices[0]].x);
            			target_points_xyz.push_back(kdtree_ptr_.getInputCloud()->points[indices[0]].y);
            			target_points_xyz.push_back(kdtree_ptr_.getInputCloud()->points[indices[0]].z);
			}
		
			mse_ /= static_cast<float>( transformed_cloud.points.size() );

			Eigen::Map<Eigen::Matrix3Xf> source_points(source_points_xyz.data(), 3, source_points_xyz.size() / 3);
        		Eigen::Map<Eigen::Matrix3Xf> target_points(target_points_xyz.data(), 3, target_points_xyz.size() / 3);

			Eigen::Vector3f source_mean = source_points.rowwise().mean();
        		Eigen::Vector3f target_mean = target_points.rowwise().mean();
		
			source_points.colwise() -= source_mean;
        		target_points.colwise() -= target_mean;

			Eigen::Matrix3f H = source_points * target_points.transpose();
        		Eigen::JacobiSVD<Eigen::Matrix3f> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);

			Eigen::Vector3f prev_translation = transform.block<3, 1>(0, 3);

			Eigen::Matrix3f updated_rotation = svd.matrixV() * svd.matrixU().transpose();
        		Eigen::Vector3f updated_translation = target_mean - updated_rotation * source_mean;

        		transform.block<3, 3>(0, 0) = updated_rotation;
        		transform.block<3, 1>(0, 3) = updated_translation;

			iter ++;

			auto t2 = std::chrono::steady_clock::now();
                        auto duration = std::chrono::duration<double,std::milli>(t2-t1).count();
                        std::cout<<"duration : "<<duration<<" ms, mse = "<<mse_<<std::endl;
		}
	}

private:
	pcl::KdTreeFLANN<pcl::PointXYZ>  kdtree_ptr_;

	Eigen::Matrix<float, 6, 6> Hessian_ = Eigen::Matrix<float, 6, 6>::Zero();
        Eigen::Matrix<float, 6, 1> B_ = Eigen::Matrix<float, 6, 1>::Zero();


	float mse_ = 0;
};

}

#endif


