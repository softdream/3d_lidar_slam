#ifndef __SCAN_MATCH_H
#define __SCAN_MATCH_H

#include "config.h"
#include "kdtree_point_cloud_adapter.h"
#include "so3.h"

namespace slam
{

template<typename DerivedType, 
	 typename PointCloudType,
	 typename TransformationType>
class ScanMatchBase
{
public:
	ScanMatchBase()
	{
	
	}

	virtual ~ScanMatchBase()
	{
	
	}
	
	void scanMatch( const PointCloudType& first_point_cloud,
		     	const PointCloudType& second_point_cloud,
		     	TransformationType& transform,
		     	const int max_iterations )
	{
		if ( auto ptr = static_cast<DerivedType*>( this ) ) {
                        ptr->scanMatch( first_point_cloud, second_point_cloud, transform, max_iterations );
                }

	}
};

template<typename T>
class Point2PointICP : public ScanMatchBase<Point2PointICP<T>, PointCloud<Point3<T>>, Eigen::Matrix<T, 4, 4> >
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

		translation_vector_ += delta.block<3, 1>(0, 0);
		auto delta_rotation = SO3::exp( delta.block<3, 1>(3, 0) );
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
class Point2PlaneICP : public ScanMatchBase<Point2PlaneICP<T>, PointCloud<Point3<T>>, Eigen::Matrix<T, 4, 4> >
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
               	Hessian_.setZero();
               	B_.setZero();

                // 4.1 for every point in the second point cloud
                for ( size_t i = 0; i < second_point_cloud.poits.size(); i ++ ) {
                        auto pt_in_second = second_point_cloud.poits[i];

                        // 4.1.1 transform the second frame point to the first frame coordinate system
                        auto pt_in_transformed = rotation_matrix_ * pt_in_second + translation_vector_;
		
			Eigen::Matrix<ValueType, 3, 1> seed_pt_vec = Eigen::Matrix<ValueType, 3, 1>::Zero();
			Eigen::Matrix<ValueType, 3, 1> normal_vec;
			if( !getPlaneNormalVector( first_point_cloud, pt_in_transformed, seed_pt_vec, normal_vec ) ) {
				continue;
			}

			Eigen::Matrix<ValueType, 3, 1> pt_in_transformed_vec( pt_in_transformed.x, pt_in_transformed.y, pt_in_transformed.z );

			auto error = ( pt_in_transformed_vec - seed_pt_vec ).dot( normal_vec );

			Eigen::Matrix<ValueType, 6, 1> Jacobian = Eigen::Matrix<ValueType, 6, 1>::Zero();

			auto tmp = ( pt_in_transformed_vec.transpose() * normal_vec ) / ( pt_in_transformed_vec.transpose() * normal_vec ).norm();

			Jacobian.block<3, 1>(0, 0) = Eigen::Matrix<ValueType, 3, 3>::Identity() * normal_vec * tmp;
			Jacobian.block<3, 1>(3, 0) = ( -SO3::hat( rotation_matrix_ * Eigen::Matrix<ValueType, 3, 1>( pt_in_second.x, pt_in_second.y, pt_in_second.z ) ) ).transpose() * tmp;
		
			Hessian_ += Jacobian * Jacobian.transpose();
			B_ += -Jacobian * error;
		}

		if ( Hessian_.determinant() == 0 ) {
                        return;
                }

                Eigen::Matrix<ValueType, 6, 1> delta = Hessian_.inverse() * B_;

                translation_vector_ += delta.block<3, 1>(0, 0);
                auto delta_rotation = SO3::exp( delta.block<3, 1>(3, 0) );
                rotation_matrix_ *= delta_rotation;

                transform.block<3, 3>(0, 0) = rotation_matrix_;
                transform.block<3, 1>( 0, 3 ) = translation_vector_;

	}

private:
	bool getPlaneNormalVector( const PointCloudType& first_point_cloud, 
				   const PointType& pt_in_transformed,
				   Eigen::Matrix<ValueType, 3, 1>& seed_pt_vec,
		       		   Eigen::Matrix<ValueType, 3, 1>& normal_vec )
	{
		ValueType min_dist = 0;
                size_t seed_point_idx = -1;
                nanoflann::KNNResultSet<ValueType> ret_set( 1 );

                ValueType query_pt[3] = { pt_in_transformed.x, pt_in_transformed.y, pt_in_transformed.z };
                ret_set.init( &seed_point_idx, &min_dist );

                kdtree_ptr_->findNeighbors( ret_set, query_pt );

                auto seed_point = first_point_cloud.points[seed_point_idx];
		seed_pt_vec = Eigen::Matrix<ValueType, 3, 1>( seed_point.x, seed_point.y, seed_point.z );

                std::vector<size_t> plane_closed_idx( 20 );
                std::vector<ValueType> plane_min_dists( 20 );
                nanoflann::KNNResultSet<ValueType> plane_ret_set( 20 );
                ValueType plane_query_pt = { seed_point.x, seed_point.y, seed_point.z };
		
		plane_ret_set.init( &plane_closed_idx[0], &plane_min_dists[0] );

		kdtree_ptr_->findNeighbors( plane_ret_set, plane_query_pt );

		std::vector<size_t> five_points_idx_vec;
		std::vector<int> diff_scan_id_point_idx_vec;
		size_t pre_scan_id = -1;
		size_t n = 4;

		for ( size_t i = 0; i < plane_closed_idx.size(); i ++ ) {
			auto candidate_pt = first_point_cloud.points[ plane_closed_idx[i] ];

			int scan_id = -1;
			ValueType angle = ::atan2( candidate_pt.z, ::sqrt( candidate_pt.x * candidate_pt.x + candidate_pt.y * candidate_pt.y ) ) * 180 / M_PI;

			if constexpr ( Config::N_SCANS == 16 ) {
                                if ( angle >= -15 && angle <= 15 ) {
                                        scan_id = static_cast<int>( ( angle + 15 ) / 2 + 0.5 );
                                }
                        }
			
			if ( i == 0 ) {
				pre_scan_id = scan_id;
			}

			if ( i < 5 ) {
				five_points_idx_vec.push_back( plane_closed_idx[i] );
			}
			else {
				if ( scan_id != pre_scan_id && scan_id >= 0 && scan_id < 16 ) {
					diff_scan_id_point_idx_vec.push_back( plane_closed_idx[i] );
					
					n = i;

					if ( diff_scan_id_point_idx_vec.size() >= 2 ) { break; };
				}
			}
		}

		if ( diff_scan_id_point_idx_vec.size() == 1 ) {
			five_points_idx_vec[4] = diff_scan_id_point_idx_vec[0];
		}
		else if ( diff_scan_id_point_idx_vec.size() == 2 ) {
			five_points_idx_vec[3] = diff_scan_id_point_idx_vec[0];
			five_points_idx_vec[4] = diff_scan_id_point_idx_vec[1];
		}

		if( plane_min_dists[n] >= 1 ) return false;

		Eigen::Matrix<ValueType, 5, 3> Y = Eigen::Matrix<ValueType, 5, 3>::Zero();
		Eigen::Matrix<ValueType, 5, 1> b;
		b.fill(-1);
		normal_vec.setZero();

		for ( size_t i = 0; i < 5; i ++ ) {
			Y( i, 0 ) = first_point_cloud.points[ five_points_idx_vec[i] ].x;
			Y( i, 1 ) = first_point_cloud.points[ five_points_idx_vec[i] ].y;
			Y( i, 2 ) = first_point_cloud.points[ five_points_idx_vec[i] ].z;
		}

		normal_vec = Y.colPivHouseholderQr().solve( b );
		normal_vec.normalize(); // normal vector of the plane

		return true;

	}

private:
	std::unique_ptr<kdtree::KdTreeType<ValueType>> kdtree_ptr_;

	Eigen::Matrix<ValueType, 6, 6> Hessian_ = Eigen::Matrix<ValueType, 6, 6>::Zero();
	Eigen::Matrix<ValueType, 6, 1> B_ = Eigen::Matrix<ValueType, 6, 1>::Zero();
	
	RotationType rotation_matrix_;
        RotationType translation_vector_;
};

template<typename DerivedType, typename PointCloudType, typename TransformationType>
void scanMatch( ScanMatchBase<DerivedType, PointCloudType, TransformationType>&& instance, 
	     	const PointCloudType& first_point_cloud,
                const PointCloudType& second_point_cloud,
                TransformationType& transform,
                const int max_iterations )
{
	instance.scanMatch( first_point_cloud, second_point_cloud, transform, max_iterations );
}


}

#endif
