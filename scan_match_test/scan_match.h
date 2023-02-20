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
	using PointCloudType = typename PointCloud<ValueType>;


	Point2PointICP()
	{
	
	}


	~Point2PointICP()
	{
	
	}	
	

	void scanMatch( const PointCloudType& first_point_cloud, 
			PointCloudType& second_point_cloud,
			TransformationType& pose,
			const int max_iterations = 10 )
	{
		// 1. error process
		
		// 2. construct the kd tree of the first point cloud
		kdtree::KdTreePointCloudType<ValueType> kd_first_point_cloud( first_point_cloud );
		kdtree_ptr_ = std::make_unique<kdtree::KdTreeType<ValueType>>( 3, kd_first_point_cloud, {10} );
		// 3. start optimization
		
	}

private:
	void estimateOnce( const PointCloudType& first_point_cloud,
                           PointCloudType& second_point_cloud,
                           State::TransformationType<ValueType>& pose )
	{
		// 3.1 for all the point of the second point cloud, do pose transformation
		pointCloudTransform( second_point_cloud, pose );
		
		Hessian.setZero();
		B.setZero();

		// 3.2 for every point in the second point cloud
		for ( size_t i = 0; i < second_point_cloud.poits.size(); i ++ ) {
			auto pt_in_second = second_point_cloud.poits[i];

			// 3.2.1 for point in second point cloud, find the closed point in the first point cloud 
			ValueType min_dist = 0;
			size_t closed_point_idx = -1;
			nanoflann::KNNResultSet<ValueType> ret_set( 1 );

			ValueType query_pt[3] = { pt_in_second.x, pt_in_second.y, pt_in_second.z };
			ret_set.init( &closed_point_idx, &min_dist );

			kdtree_ptr_->findNeighbors( ret_set, query_pt );

			// 3.2.2 the invalid points need to be removed according to the distance
			//if( min_dist > min_dist_thresh ) {
			//	continue;	
			//}
			//
			//TODO... add some other policies for removing invalid points
			//

			State::Vector3<ValueType> closed_pt_in_first( first_point_cloud.points[closed_point_idx].x,
				       				      first_point_cloud.points[closed_point_idx].y,
								      first_point_cloud.points[closed_point_idx].z);
		
			// 3.2.3 caculate the error vector
			State::Vector3<ValueType> error = State::Vector3<ValueType>( pt_in_second.x, pt_in_second.y, pt_in_second.z ) - closed_pt_in_first;

			// 3.2.4 caculate the Jacobian 
			Eigen::Matrix<ValueType, 3, 6> Jacobian = Eigen::Matrix<ValueType, 3, 6>::Zero();
			Jacobian.leftCols<3>() = Eigen::Matrix<ValueType, 3, 3>::Identity();

			State::RotationType<ValueType> rotation = pose.block<3, 3>(0, 0);
			State::Translation<ValueType> translation = pose.block<3, 1>( 0, 3 );

			//Jacobian.rightCols<3>() = -rotation * SO3::hat()
		}
	}

	void pointCloudTransform( PointCloudType& point_cloud, State::TransformationType<ValueType>& transform ) 
	{
		
	}

	void pointCloudTransform( const PointCloudType& point_cloud_befor,
		       		  PointCloudType& point_cloud_after,
		       		  State::TransformationType<ValueType>& transform )
	{
		
	}

private:

	std::unique_ptr<kdtree::KdTreeType<ValueType>> kdtree_ptr_;

	Eigen::Matrix<ValueType, 6, 6> Hessian = Eigen::Matrix<ValueType, 6, 6>::Zero();
	Eigen::Matrix<ValueType, 6, 1> B = Eigen::Matrix<ValueType, 6, 1>::Zero();
};

}

#endif
