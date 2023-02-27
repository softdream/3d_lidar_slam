#ifndef __ICP_MANUAL_H
#define __ICP_MANUAL_H

#include <pcl/registration/icp.h>    
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>

#include "so3.h"

namespace test
{

template<typename T>
class Point2PointICP
{
public:
	using ValueType = T;
	using CloudTypePtr = typename pcl::PointCloud<pcl::PointXYZ>::Ptr;
	using CloudType = typename pcl::PointCloud<pcl::PointXYZ>;

	using TransformationType = typename Eigen::Matrix<ValueType, 4, 4>;
        using RotationType = typename Eigen::Matrix<ValueType, 3, 3>;
        using TranslationType = typename Eigen::Matrix<ValueType, 3, 1>;

	void scanMatch( const CloudTypePtr& first_cloud, 
			const CloudType& second_cloud,
		        TransformationType& transform,
			const int max_iterations )
	{
	//	kdtree_ptr_ = new  pcl::KdTreeFLANN<pcl::PointXYZ>();

		rotation_matrix_ = transform.template block<3,3>(0, 0);
                translation_  = transform.template block<3, 1>( 0, 3 );

		int iter = 0;
		while ( iter < max_iterations ) {
                        auto t1 = std::chrono::steady_clock::now();
	
			mse_ = 0;
			kdtree_ptr_.setInputCloud( first_cloud );
			CloudType transformed_cloud;
			pcl::transformPointCloud(second_cloud, transformed_cloud, transform);

                        estimateOnce( *first_cloud, second_cloud, transformed_cloud, transform );
                        auto t2 = std::chrono::steady_clock::now();
                        double dr_ms = std::chrono::duration<double,std::milli>(t2-t1).count();
                        std::cout<<dr_ms<<" "<<mse_<<std::endl;

                        iter ++;
                }

	}

private:
	void estimateOnce( const CloudType& first_cloud, 
				const CloudType& second_cloud, 
				const CloudType& transformed_cloud,
				TransformationType& transform )
	{
		Hessian_.setZero();
                B_.setZero();

		for ( size_t i = 0; i < transformed_cloud.size(); i ++ ) {
			auto ori_point = second_cloud.points.at(i);

			auto transformed_point = transformed_cloud.at(i);

			std::vector<float> distances;
            		std::vector<int>indexs;
            		kdtree_ptr_.nearestKSearch(transformed_point, 1, indexs, distances);      // knn搜索
			if( distances[0] > 0.5 ) { 
				continue;       
			}

			Eigen::Matrix<ValueType, 3, 1> closet_point = Eigen::Matrix<ValueType, 3, 1>( first_cloud.at(indexs[0]).x, first_cloud.at(indexs[0]).y, first_cloud.at(indexs[0]).z );

			Eigen::Matrix<ValueType, 3, 1> err_dist = Eigen::Matrix<ValueType, 3, 1>( transformed_point.x, transformed_point.y, transformed_point.z ) - closet_point;

			mse_ += err_dist.norm();

			auto t1 = std::chrono::steady_clock::now();
			Eigen::Matrix<ValueType,3,6> Jacobian(Eigen::Matrix<ValueType,3,6>::Zero());
			 
            		Jacobian.template block<3, 3>( 0, 0 ) = Eigen::Matrix<ValueType, 3, 3>::Identity();
			Jacobian.template block<3, 3>( 0, 3 ) = -rotation_matrix_ * slam::SO3::hat( Eigen::Matrix<ValueType, 3, 1>( ori_point.x, ori_point.y, ori_point.z ) );


                    	Hessian_  +=  Jacobian.transpose()* Jacobian; 
                    	B_ += -Jacobian.transpose() * err_dist;
	
			auto t2 = std::chrono::steady_clock::now();
                        double dr_ms = std::chrono::duration<double,std::milli>(t2-t1).count();
                        //std::cout<<"duration ["<<i<<"] = "<<dr_ms<<std::endl;
		}

		if (Hessian_.determinant() == 0) {
                	return;
        	}

		Eigen::Matrix<ValueType, 6, 1> delta_x = Hessian_.inverse() * B_;

        	translation_ += delta_x.template head<3>();
		Eigen::Matrix<ValueType, 3, 1> delta_rotation_tmp = delta_x.template block<3, 1>(3, 0);
		rotation_matrix_ *= slam::SO3::exp( delta_rotation_tmp );

        	//auto  delta_rotation = Sophus::SO3f::exp(delta_x.tail<3>());
        	//rotation_matrix_ *= delta_rotation.matrix();

        	transform.template block<3,3>(0,0) = rotation_matrix_;
        	transform.template block<3,1>(0,3) = translation_;
	}

private:
	pcl::KdTreeFLANN<pcl::PointXYZ>  kdtree_ptr_;

	Eigen::Matrix<ValueType, 6, 6> Hessian_ = Eigen::Matrix<ValueType, 6, 6>::Zero();
        Eigen::Matrix<ValueType, 6, 1> B_ = Eigen::Matrix<ValueType, 6, 1>::Zero();

        RotationType rotation_matrix_ = RotationType::Zero();
        TranslationType translation_ = TranslationType::Zero();

	ValueType mse_ = 0;
};

}

#endif


