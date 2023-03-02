#ifndef __RANGE_IMAGE_H
#define __RANGE_IMAGE_H

#include "config.h"
#include <Eigen/Dense>
#include <map>
#include <type_traits>


namespace slam
{

template<int SCANS>
struct lidar_type_trait : public std::false_type
{

};

template<>
struct lidar_type_trait<64> : public std::true_type
{
	static constexpr int rows = 64;
	static constexpr int cols = 4000;
};

template<>
struct lidar_type_trait<16> : public std::true_type
{
        static constexpr int rows = 16;
	static constexpr int cols = 1800;
};

template<int, int>
struct is_same_scans : public std::false_type
{

};

template<int SCANS_>
struct is_same_scans<SCANS_, SCANS_> : public std::true_type
{

};


template<typename ValueType, 
	 int SCANS = Config::N_SCANS, 
	 typename = typename std::enable_if<is_same_scans<SCANS, Config::N_SCANS>::value>::type>
class RangeImage
{
public:
	struct RangeImageType_
	{
		//typename std::enable_if<lidar_type_trait<SCANS>::value, Eigen::Matrix<ValueType, lidar_type_trait<SCANS>::rows, lidar_type_trait<SCANS>::cols>>::type mat;
		RangeImageType_()
		{
			mat.resize( lidar_type_trait<SCANS>::rows, lidar_type_trait<SCANS>::cols );
		}		

		Eigen::Matrix<ValueType, Eigen::Dynamic, Eigen::Dynamic> mat;
		std::map<std::pair<int, int>, int> idx_map;
	};

	using RangeImageType = RangeImageType_;

	template<typename PointCloud>
	static void generateRangeImage( const PointCloud& point_cloud, RangeImageType& range_image )
	{
		range_image.mat.setZero();

		for ( size_t i = 0; i < point_cloud.points.size(); i ++ ) {
			auto pt = point_cloud.points[i];
	
			ValueType vertical_angle = ::atan2( pt.z, ::sqrt( pt.x * pt.x + pt.y * pt.y ) ) * 180 / M_PI;
	
			//int row_idx = -( vertical_angle + Config::vertical_angle_min<ValueType> ) / Config::vertical_angle_resolution<ValueType>;
			int row_idx = -1;	
			if constexpr( Config::N_SCANS == 64 ) {
                                if ( vertical_angle >= -24.33 && vertical_angle <= 2 ) {
                                        if ( vertical_angle >= -8.83 ) {
                                                row_idx = static_cast<int>( ( 2 - vertical_angle ) * 3.0 + 0.5 );
                                        }
                                        else {
                                                row_idx = Config::N_SCANS / 2 + static_cast<int>( ( -8.83 - vertical_angle ) * 2.0 + 0.5 );
                                        }
                                }
                        }


			if ( row_idx < 0 || row_idx >= SCANS ) continue;

			ValueType horizon_angle = ::atan2( pt.y, pt.x ) * 180 / M_PI;

			int cols_idx = -std::round( ( horizon_angle - 0.0 ) / Config::horizon_angle_resoluation<ValueType> ) + lidar_type_trait<SCANS>::cols / 2;

			if ( cols_idx >= lidar_type_trait<SCANS>::cols ) cols_idx -= lidar_type_trait<SCANS>::cols;

			if ( cols_idx < 0 || cols_idx >= lidar_type_trait<SCANS>::cols ) continue;

			ValueType range = std::sqrt( pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
        		if ( range < Config::range_min<ValueType> ) continue;
	
			range_image.mat( row_idx, cols_idx ) = range;
			range_image.idx_map[{row_idx, cols_idx}] = i;
		}
	}

};


}

#endif
