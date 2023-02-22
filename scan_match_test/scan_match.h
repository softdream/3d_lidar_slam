#ifndef __SCAN_MATCH_H
#define __SCAN_MATCH_H

#include "kdtree_point_cloud_adapter.h"
#include "so3.h"

namespace slam
{

template<typename DerivedType>
class ScanMatchBase
{
public:
	ScanMatchBase()
	{
	
	}

	virtual ~ScanMatchBase()
	{
	
	}
	
/*	void scanMatch()
	{
		if ( auto ptr = static_cast<DerivedType*>( this ) ) {
                        ptr->scanMatch();
                }

	}*/
};

template<typename T>
class Point2PointICP : public ScanMatchBase<Point2PointICP<T>>
{
public:
	using ValueType = T;
	using PointType = Point3<ValueType>;
	using PointCloudType = PointCloud<Point3<ValueType>>;

	using TransformationType = typename Eigen::Matrix<ValueType, 4, 4>;
	using RotationType = typename Eigen::Matrix<ValueType, 3, 3>;
	using TranslationType = typename Eigen::Matrix<ValueType, 3, 1>;

	Point2PointICP()
	{
	
	}


	~Point2PointICP()
	{
	
	}	
	

	void scanMatch( const PointCloudType& first_point_cloud, 
			const PointCloudType& second_point_cloud,
			TransformationType& transform,
			const int max_iterations = 10 )
	{
		// 1. error process
		if ( first_point_cloud.points.size() == 0 || first_point_cloud.points.size() == 0 ) {
			return;
		}		

		// 2. construct the kd tree of the first point cloud
		kdtree::KdTreePointCloudType<ValueType> kd_first_point_cloud( first_point_cloud );
		kdtree_ptr_ = std::make_unique<kdtree::KdTreeType<ValueType>>( 3, kd_first_point_cloud, {10} );

		// 3. get the initial transformation
		rotation_matrix_ = transform.block<3,3>(0, 0);
                translation_vector_  = transform.block<3, 1>( 0, 3 );
		
		// 4. start transform
		int iter = 0;
		while ( iter <= max_iterations ) {
			estimateOnce( first_point_cloud, second_point_cloud, transform );
			iter ++;
		}
	}

private:
	void estimateOnce( const PointCloudType& first_point_cloud,
                           const PointCloudType& second_point_cloud,
                           TransformationType& transform )
	{
		Hessian_.setZero();
		B_.setZero();

		// 4.1 for every point in the second point cloud
		for ( size_t i = 0; i < second_point_cloud.poits.size(); i ++ ) {
			auto pt_in_second = second_point_cloud.poits[i];
			
			// 4.1.1 transform the second frame point to the first frame coordinate system
			auto pt_in_transformed = rotation_matrix_ * pt_in_second + translation_vector_;

			// 3.2.1 for point transformed, find the closed point in the first point cloud 
			ValueType min_dist = 0;
			size_t closed_point_idx = -1;
			nanoflann::KNNResultSet<ValueType> ret_set( 1 );

			ValueType query_pt[3] = { pt_in_transformed.x, pt_in_transformed.y, pt_in_transformed.z };
			ret_set.init( &closed_point_idx, &min_dist );

			kdtree_ptr_->findNeighbors( ret_set, query_pt );

			// 3.2.2 the invalid points need to be removed according to the distance
			//if( min_dist > min_dist_thresh ) {
			//	continue;	
			//}
			//
			//TODO... add some other policies for removing invalid points
			//

			Eigen::Matrix<ValueType, 3, 1> closed_pt_in_first( first_point_cloud.points[closed_point_idx].x,
				       				      	   first_point_cloud.points[closed_point_idx].y,
								      	   first_point_cloud.points[closed_point_idx].z);
		
			// 3.2.3 caculate the error vector
			Eigen::Matrix<ValueType, 3, 1> error = Eigen::Matrix<ValueType, 3, 1>( pt_in_transformed.x, pt_in_transformed.y, pt_in_transformed.z ) - closed_pt_in_first;

			// 
			//if ( error.norm() > threshold ) {
			//	continue;
			//}

			// 3.2.4 caculate the Jacobian 
			Eigen::Matrix<ValueType, 3, 6> Jacobian = Eigen::Matrix<ValueType, 3, 6>::Zero();
			Jacobian.block<3, 3>( 0, 0 ) = Eigen::Matrix<ValueType, 3, 3>::Identity();

			Jacobian.block<3, 3>( 0, 3 ) = -rotation_matrix_ * SO3::hat( Eigen::Matrix<ValueType, 3, 1>( pt_in_second.x, pt_in_second.y, pt_in_second.z ) ); 
			Hessian_ += Jacobian.transpose() * Jacobian;
			B_ += -Jacobian.transpose() * error;
		}

		if ( Hessian_.determinant() == 0 ) {
			return;
		}

		Eigen::Matrix<ValueType, 6, 1> delta = Hessian_.inverse() * B_;

		translation_vector_ += delta.block<0, 3>(0, 0);
		auto delta_rotation = SO3::exp( delta.block<0, 3>(0, 3) );
		rotation_matrix_ *= delta_rotation;
	
		transform.block<3,3>(0, 0) = rotation_matrix_;
		transform.block<3, 1>( 0, 3 ) = translation_vector_;
	}
	
private:

	std::unique_ptr<kdtree::KdTreeType<ValueType>> kdtree_ptr_;

	Eigen::Matrix<ValueType, 6, 6> Hessian_ = Eigen::Matrix<ValueType, 6, 6>::Zero();
	Eigen::Matrix<ValueType, 6, 1> B_ = Eigen::Matrix<ValueType, 6, 1>::Zero();

	RotationType rotation_matrix_;
	RotationType translation_vector_;
};

template<typename T>
class Point2PlaneICP : public ScanMatchBase<Point2PlaneICP<T>>
{
public:
        using ValueType = T;
        using PointType = Point3<ValueType>;
        using PointCloudType = PointCloud<Point3<ValueType>>;

        using TransformationType = typename Eigen::Matrix<ValueType, 4, 4>;
        using RotationType = typename Eigen::Matrix<ValueType, 3, 3>;
        using TranslationType = typename Eigen::Matrix<ValueType, 3, 1>;

	Point2PlaneICP() 
	{

	}

	~Point2PlaneICP() 
	{

	}
	
	void scanMatch( const PointCloudType& first_point_cloud,
                        const PointCloudType& second_point_cloud,
                        TransformationType& transform,
                        const int max_iterations = 10 )
        {
                // 1. error process
                if ( first_point_cloud.points.size() == 0 || first_point_cloud.points.size() == 0 ) {
                        return;
                }

                // 2. construct the kd tree of the first point cloud
                kdtree::KdTreePointCloudType<ValueType> kd_first_point_cloud( first_point_cloud );
                kdtree_ptr_ = std::make_unique<kdtree::KdTreeType<ValueType>>( 3, kd_first_point_cloud, {10} );

                // 3. get the initial transformation
                rotation_matrix_ = transform.block<3,3>(0, 0);
                translation_vector_  = transform.block<3, 1>( 0, 3 );

                // 4. start transform
                int iter = 0;
                while ( iter <= max_iterations ) {
                        estimateOnce( first_point_cloud, second_point_cloud, transform );
                        iter ++;
                }
        }

private:
	void estimateOnce( const PointCloudType& first_point_cloud,
                           const PointCloudType& second_point_cloud,
                           TransformationType& transform )
        {
               	Hessian.setZero();
               	B.setZero();

                // 4.1 for every point in the second point cloud
                for ( size_t i = 0; i < second_point_cloud.poits.size(); i ++ ) {
                        auto pt_in_second = second_point_cloud.poits[i];

                        // 4.1.1 transform the second frame point to the first frame coordinate system
                        auto pt_in_transformed = rotation_matrix_ * pt_in_second + translation_vector_;
		}
	}
private:
	std::unique_ptr<kdtree::KdTreeType<ValueType>> kdtree_ptr_;

	Eigen::Matrix<ValueType, 6, 6> Hessian_ = Eigen::Matrix<ValueType, 6, 6>::Zero();
	Eigen::Matrix<ValueType, 6, 1> B_ = Eigen::Matrix<ValueType, 6, 1>::Zero();
	
	RotationType rotation_matrix_;
        RotationType translation_vector_;
};

}

#endif
