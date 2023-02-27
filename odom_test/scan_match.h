#ifndef __SCAN_MATCH_H
#define __SCAN_MATCH_H

#include "config.h"
#include "kdtree_point_cloud_adapter.h"
#include "so3.h"

namespace slam
{

// CRTP Base Class
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
	
	// interface
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


// Point to Point ICP method
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
			const int max_iterations )
	{
		// 1. error process
		if ( first_point_cloud.points.size() == 0 || first_point_cloud.points.size() == 0 ) {
			return;
		}		

		// 2. construct the kd tree of the first point cloud
		kdtree::KdTreePointCloudType<ValueType> kd_first_point_cloud( first_point_cloud );
		kdtree_ptr_ = std::make_unique<kdtree::KdTreeType<ValueType>>( 3, kd_first_point_cloud, 10 );

		// 3. get the initial transformation
		rotation_matrix_ = transform.template block<3,3>(0, 0);
                translation_vector_  = transform.template block<3, 1>( 0, 3 );

		// 4. start transform
		int iter = 0;
		ValueType pre_mse = 100;;
		//while ( iter <= max_iterations && std::abs( mse_ - pre_mse ) > 0.1 ) {
		while ( iter < max_iterations ) {
			mse_ = 0;
			estimateOnce( first_point_cloud, second_point_cloud, transform );
			std::cout<<"mse = "<<mse_<<std::endl;
			pre_mse = mse_;
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
		for ( size_t i = 0; i < second_point_cloud.points.size(); i ++ ) {
			auto pt_in_second = second_point_cloud.points[i]; // get the point
			Eigen::Matrix<ValueType, 3, 1> pt_in_second_vec( pt_in_second.x, pt_in_second.y, pt_in_second.z ); // conver to eigen type

			// 4.1.1 transform the second frame point to the first frame coordinate system
			auto pt_in_transformed = rotation_matrix_ * pt_in_second_vec + translation_vector_; 

			// 4.1.2 for point transformed, find the closed point in the first point cloud 
			ValueType min_dist = 0;
			size_t closed_point_idx = -1;
			nanoflann::KNNResultSet<ValueType> ret_set( 1 );

			ValueType query_pt[3] = { pt_in_transformed(0), pt_in_transformed(1), pt_in_transformed(2) };
			ret_set.init( &closed_point_idx, &min_dist );

			kdtree_ptr_->findNeighbors( ret_set, query_pt );

			//std::cout<<"min_dist["<<i<<"] = "<<min_dist<<std::endl;

			// 4.1.3 the invalid points need to be removed according to the distance
			if ( min_dist > Config::icp_pt_min_dist_thresh<ValueType> ) {
				continue;	
			}
			
			//TODO... add some other policies for removing invalid points
			//

			Eigen::Matrix<ValueType, 3, 1> closed_pt_in_first( first_point_cloud.points[closed_point_idx].x,
				       				      	   first_point_cloud.points[closed_point_idx].y,
								      	   first_point_cloud.points[closed_point_idx].z);
		
			// 4.1.4 caculate the error vector
			Eigen::Matrix<ValueType, 3, 1> error = pt_in_transformed - closed_pt_in_first;
			//std::cout<<"error["<<i<<"] = "<<error.transpose()<<std::endl;

			// remove the invalid matched points
			if ( error.norm() > Config::icp_pt_min_dist_thresh<ValueType> ) {
				continue;
			}

			//std::cout<<"error["<<i<<"] = "<<error.transpose()<<std::endl;
	
			mse_ += error.norm(); // caculate mse 

			// 4.1.5 caculate the Jacobian 
			Eigen::Matrix<ValueType, 3, 6> Jacobian = Eigen::Matrix<ValueType, 3, 6>::Zero();
			Jacobian.template block<3, 3>( 0, 0 ) = Eigen::Matrix<ValueType, 3, 3>::Identity();

			Jacobian.template block<3, 3>( 0, 3 ) = -rotation_matrix_ * SO3::hat( pt_in_second_vec ); 


			Hessian_ += Jacobian.transpose() * Jacobian;
			B_ += -Jacobian.transpose() * error;
		}

		if ( Hessian_.determinant() == 0 ) {
			return;
		}

		// 4.2 caculate the increments of the transformation
		Eigen::Matrix<ValueType, 6, 1> delta = Hessian_.inverse() * B_;

		// 4.3 update the translation & rotation matrix
                translation_vector_ += delta.template block<3, 1>(0, 0);

		Eigen::Matrix<ValueType, 3, 1> delta_rotation_tmp = delta.template block<3, 1>(3, 0);
                rotation_matrix_ *= SO3::exp( delta_rotation_tmp );

		// 4.4 update the transformation matrix
                transform.template block<3, 3>(0, 0) = rotation_matrix_;
                transform.template block<3, 1>( 0, 3 ) = translation_vector_;
	}
	
private:

	std::unique_ptr<kdtree::KdTreeType<ValueType>> kdtree_ptr_;

	Eigen::Matrix<ValueType, 6, 6> Hessian_ = Eigen::Matrix<ValueType, 6, 6>::Zero();
	Eigen::Matrix<ValueType, 6, 1> B_ = Eigen::Matrix<ValueType, 6, 1>::Zero();

	RotationType rotation_matrix_ = RotationType::Zero();
	TranslationType translation_vector_ = TranslationType::Zero();

	ValueType mse_ = 0;
};

// Policy for caculating the plane normal vectors
class FirstNormalPolicy
{
public:
	template<typename PointCloudType, typename ValueType>
	static bool getPlaneNormalVector( const std::unique_ptr<kdtree::KdTreeType<ValueType>>& kdtree_ptr, 
					  const PointCloudType& first_point_cloud, 
                                   	  const Eigen::Matrix<ValueType, 3, 1>& pt_in_transformed,
                                   	  Eigen::Matrix<ValueType, 3, 1>& seed_pt_vec,
                                   	  Eigen::Matrix<ValueType, 3, 1>& normal_vec )
        {
                ValueType min_dist = 0;
                size_t seed_point_idx = -1;
                nanoflann::KNNResultSet<ValueType> ret_set( 1 );

                ValueType query_pt[3] = { pt_in_transformed(0), pt_in_transformed(1), pt_in_transformed(2) };
                ret_set.init( &seed_point_idx, &min_dist );

                kdtree_ptr->findNeighbors( ret_set, query_pt );

                auto seed_point = first_point_cloud.points[seed_point_idx];
                seed_pt_vec = Eigen::Matrix<ValueType, 3, 1>( seed_point.x, seed_point.y, seed_point.z );

                std::vector<size_t> plane_closed_idx( 20 );
                std::vector<ValueType> plane_min_dists( 20 );
                nanoflann::KNNResultSet<ValueType> plane_ret_set( 20 );
                ValueType plane_query_pt[3] = { seed_point.x, seed_point.y, seed_point.z };
                
                plane_ret_set.init( &plane_closed_idx[0], &plane_min_dists[0] );

                kdtree_ptr->findNeighbors( plane_ret_set, plane_query_pt );

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
	
};

// Policy for caculating the plane normal vectors
class SecondNormalPolicy
{
public:
        template<typename PointCloudType, typename ValueType>
        static bool getPlaneNormalVector( const std::unique_ptr<kdtree::KdTreeType<ValueType>>& kdtree_ptr,
                                          const PointCloudType& first_point_cloud,
                                          const Eigen::Matrix<ValueType, 3, 1>& pt_in_transformed,
                                          Eigen::Matrix<ValueType, 3, 1>& seed_pt_vec,
                                          Eigen::Matrix<ValueType, 3, 1>& normal_vec )
        {
                ValueType min_dist = 0;
                size_t seed_point_idx = -1;
                nanoflann::KNNResultSet<ValueType> ret_set( 1 );

                ValueType query_pt[3] = { pt_in_transformed(0), pt_in_transformed(1), pt_in_transformed(2) };
                ret_set.init( &seed_point_idx, &min_dist );

                kdtree_ptr->findNeighbors( ret_set, query_pt );

                auto seed_point = first_point_cloud.points[seed_point_idx];
                seed_pt_vec = Eigen::Matrix<ValueType, 3, 1>( seed_point.x, seed_point.y, seed_point.z );

                std::vector<size_t> plane_closed_idx( 5 );
                std::vector<ValueType> plane_min_dists( 5 );
                nanoflann::KNNResultSet<ValueType> plane_ret_set( 5 );
                ValueType plane_query_pt[3] = { seed_point.x, seed_point.y, seed_point.z };

                plane_ret_set.init( &plane_closed_idx[0], &plane_min_dists[0] );

                kdtree_ptr->findNeighbors( plane_ret_set, plane_query_pt );

                if( plane_min_dists[4] >= 1 ) return false;

		Eigen::Matrix<ValueType, 5, 3> Y = Eigen::Matrix<ValueType, 5, 3>::Zero();
                Eigen::Matrix<ValueType, 5, 1> b;
                b.fill(-1);
                normal_vec.setZero();

                for ( size_t i = 0; i < 5; i ++ ) {
                        Y( i, 0 ) = first_point_cloud.points[ plane_closed_idx[i] ].x;
                        Y( i, 1 ) = first_point_cloud.points[ plane_closed_idx[i] ].y;
                        Y( i, 2 ) = first_point_cloud.points[ plane_closed_idx[i] ].z;
                }

                normal_vec = Y.colPivHouseholderQr().solve( b );
                normal_vec.normalize(); // normal vector of the plane

                return true;

        }

};

// Point to Plane ICP method
template<typename T, typename CacuNormalPolicy = FirstNormalPolicy>
class Point2PlaneICP : public ScanMatchBase<Point2PlaneICP<T, CacuNormalPolicy>, PointCloud<Point3<T>>, Eigen::Matrix<T, 4, 4> >
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
                        const int max_iterations )
        {
                // 1. error process
                if ( first_point_cloud.points.size() == 0 || first_point_cloud.points.size() == 0 ) {
                        return;
                }

                // 2. construct the kd tree of the first point cloud
                kdtree::KdTreePointCloudType<ValueType> kd_first_point_cloud( first_point_cloud );
                kdtree_ptr_ = std::make_unique<kdtree::KdTreeType<ValueType>>( 3, kd_first_point_cloud, 10 );

                // 3. get the initial transformation
                rotation_matrix_ = transform.template block<3, 3>(0, 0);
                translation_vector_  = transform.template block<3, 1>( 0, 3 );

                // 4. start transform
                int iter = 0;
                while ( iter < max_iterations ) {
			mse_ = 0;
                        estimateOnce( first_point_cloud, second_point_cloud, transform );
	
			std::cout<<"mse = "<<mse_<<std::endl;
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
                for ( size_t i = 0; i < second_point_cloud.points.size(); i ++ ) {
                        auto pt_in_second = second_point_cloud.points[i];
			Eigen::Matrix<ValueType, 3, 1> pt_in_second_vec( pt_in_second.x, pt_in_second.y, pt_in_second.z ); // conver to eigen type

                        // 4.1.1 transform the second frame point to the first frame coordinate system
                        Eigen::Matrix<ValueType, 3, 1> pt_in_transformed = rotation_matrix_ * pt_in_second_vec + translation_vector_;

			// 4.1.2 caculate the normal vector
			Eigen::Matrix<ValueType, 3, 1> seed_pt_vec = Eigen::Matrix<ValueType, 3, 1>::Zero();
			Eigen::Matrix<ValueType, 3, 1> normal_vec;
			
			if( !CacuNormalPolicy::getPlaneNormalVector( kdtree_ptr_, first_point_cloud, pt_in_transformed, seed_pt_vec, normal_vec ) ) {
				continue;
			}

			// 4.1.3 caculate the error
			auto error = ( pt_in_transformed - seed_pt_vec ).dot( normal_vec );
			if ( error > Config::icp_pt_min_dist_thresh<ValueType> ) {
                                continue;
                        }
		
			mse_ += error;

			// 4.1.4 caculate the Jacobian
			Eigen::Matrix<ValueType, 6, 1> Jacobian = Eigen::Matrix<ValueType, 6, 1>::Zero();

			//auto tmp = error / error.norm();
				
			Jacobian.template block<3, 1>(0, 0) = Eigen::Matrix<ValueType, 3, 3>::Identity() * normal_vec /* tmp*/;
		
			Eigen::Matrix<ValueType, 3, 1> rotated_pt = rotation_matrix_ * pt_in_second_vec;
			Jacobian.template block<3, 1>(3, 0) = ( -SO3::hat( rotated_pt ) ).transpose() * normal_vec;
		
			// 4.1.5 caculate the Hessian matrix and B matrix of the Gaussian-Newton Method
			Hessian_ += Jacobian * Jacobian.transpose();
			B_ += -Jacobian * error;
		}

		if ( Hessian_.determinant() == 0 ) {
                        return;
                }

		// 4.2 caculate the increments of the transformation
                Eigen::Matrix<ValueType, 6, 1> delta = Hessian_.inverse() * B_;

		// 4.3 update the translation & rotation matrix
                translation_vector_ += delta.template block<3, 1>(0, 0);
		Eigen::Matrix<ValueType, 3, 1> delta_rotation_tmp = delta.template block<3, 1>(3, 0);
                rotation_matrix_ *= SO3::exp( delta_rotation_tmp );

		// 4.4 update the transformation matrix
                transform.template block<3, 3>(0, 0) = rotation_matrix_;
                transform.template block<3, 1>( 0, 3 ) = translation_vector_;

	}

private:
	std::unique_ptr<kdtree::KdTreeType<ValueType>> kdtree_ptr_;

	Eigen::Matrix<ValueType, 6, 6> Hessian_ = Eigen::Matrix<ValueType, 6, 6>::Zero();
	Eigen::Matrix<ValueType, 6, 1> B_ = Eigen::Matrix<ValueType, 6, 1>::Zero();
	
	RotationType rotation_matrix_;
        TranslationType translation_vector_;

	ValueType mse_ = 0;
};


// CRTP Helper function
template<typename DerivedType, typename PointCloudType, typename TransformationType>
void scanMatch( ScanMatchBase<DerivedType, PointCloudType, TransformationType>& instance, 
	     	const PointCloudType& first_point_cloud,
                const PointCloudType& second_point_cloud,
                TransformationType& transform,
                const int max_iterations = 10 )
{
	instance.scanMatch( first_point_cloud, second_point_cloud, transform, max_iterations );
}


}

#endif
