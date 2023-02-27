#ifndef __FEATURE_EXTRACT_H
#define __FEATURE_EXTRACT_H

#include <vector>

#include <any>
#include <iostream>
#include <cmath>

#include "config.h"

namespace slam
{

template<typename DerivedType>
class FeatureBase
{
public:
	FeatureBase()
	{
	
	}

	virtual ~FeatureBase()
	{
	
	}

	template<typename InputCloudType, typename ...OutputCloudsType>
	void extractFeaturesFromCloud( const InputCloudType& input_cloud, OutputCloudsType&&... output_clouds )
	{
		if ( auto ptr = static_cast<DerivedType*>( this ) ) {
			ptr->extractFeaturesFromCloud( input_cloud, output_clouds... );
		}
	}

	template<typename CloudType>
        void extractFeaturesFromCloud( const CloudType& input_cloud, CloudType& output_cloud )
        {
                if ( auto ptr = static_cast<DerivedType*>( this ) ) {
                        ptr->extractFeaturesFromCloud( input_cloud, output_cloud );
                }
        }

        template<typename CloudType>
        void extractFeaturesFromCloud( const CloudType& input_cloud, CloudType& output_cloud1, CloudType& output_cloud2 )
        {
                if ( auto ptr = static_cast<DerivedType*>( this ) ) {
                        ptr->extractFeaturesFromCloud( input_cloud, output_cloud1, output_cloud2 );
                }
        }

	template<typename CloudType>
        void extractFeaturesFromCloud( const CloudType& input_cloud,
				       CloudType& output_cloud1, 
				       CloudType& output_cloud2,
				       std::vector<int>& scan_ids_vec )
        {
                if ( auto ptr = static_cast<DerivedType*>( this ) ) {
                        ptr->extractFeaturesFromCloud( input_cloud, output_cloud1, output_cloud2 );
                }
        }

protected:
        template<typename OutputCloudType, typename ...OutputCloudsType>
        void extractFeaturesFromCloudHelper( OutputCloudType&& ouput_cloud, OutputCloudsType&&... output_clouds )
        {
		output_clouds_ptrs_vec_.push_back( &ouput_cloud );

                extractFeaturesFromCloudHelper( output_clouds... );
        }

        void extractFeaturesFromCloudHelper()
        {

        }

protected:
	std::vector<std::any> output_clouds_ptrs_vec_;
};

class CornerPlannerFeature : public FeatureBase<CornerPlannerFeature>
{
public:

	CornerPlannerFeature()
	{
	
	}

	template<typename InputCloudType, typename ...OutputCloudsType>
        void extractFeaturesFromCloud( const InputCloudType& input_cloud, OutputCloudsType&&... output_clouds )
        {
		static_assert( !(sizeof...( output_clouds ) != 2 ), "The number of arguments is too few !");
		using CloudType = InputCloudType;

                std::cout<<"Corner Planner Feature !"<<std::endl;

		// parse the paramters
                this->extractFeaturesFromCloudHelper( output_clouds... );
		
		//std::cout<<"typeid( InputCloudType ) = "<<typeid(InputCloudType).name()<<std::endl;
		auto output_cloud1_refer = std::any_cast<typename std::remove_reference<CloudType>::type *>( this->output_clouds_ptrs_vec_[0] );
                auto output_cloud2_refer = std::any_cast<typename std::remove_reference<CloudType>::type *>( this->output_clouds_ptrs_vec_[1] );

		// 1. 
		std::vector<std::vector<typename CloudType::PointType>> scans_row_data_vec( Config::N_SCANS );
		std::vector<std::vector<typename CloudType::PointType::ValueType>> point_index_vec( Config::N_SCANS );
		std::vector<std::vector<typename CloudType::PointType::ValueType>> scans_row_curv_vec( Config::N_SCANS );

		std::cout<<"input_cloud.points.size() = "<<input_cloud.points.size()<<std::endl;
		for ( size_t i = 0; i < input_cloud.points.size(); i ++ ) {
			typename CloudType::PointType pt = input_cloud.points[i];
			
			int scan_idx = -1;
			typename CloudType::PointType::ValueType angle = ::atan2( pt.z, ::sqrt( pt.x * pt.x + pt.y * pt.y ) ) * 180 / M_PI;
	
							
			if constexpr ( Config::N_SCANS == 16 ) {
				if ( angle >= -15 && angle <= 15 ) {
					scan_idx = static_cast<int>( ( angle + 15 ) / 2 + 0.5 );
				}
			}	
			else if constexpr( Config::N_SCANS == 64 ) {
				if ( angle >= -24.33 && angle <= 2 ) {
					if ( angle >= -8.83 ) {
						scan_idx = static_cast<int>( ( 2 - angle ) * 3.0 + 0.5 );
					}
					else {
						scan_idx = Config::N_SCANS / 2 + static_cast<int>( ( -8.83 - angle ) * 2.0 + 0.5 );
					}
				}
			}

			if ( scan_idx > -1 && scan_idx < Config::N_SCANS ) {
				scans_row_data_vec[scan_idx].push_back( pt );			
				point_index_vec[scan_idx].push_back( i );
			}
		}

		// 2. 
		for ( size_t i = Config::Row_Index_Start; i < Config::N_SCANS - Config::Row_Index_End; i ++ ) {
			scans_row_curv_vec[i].resize( scans_row_data_vec[i].size() );
		//	std::cout<<"scan idx = "<<i<<std::endl;
					
		//	std::cout<<"scans_row_data_vec["<<i<<"].size() = "<<scans_row_data_vec[i].size()<<std::endl;

			for( size_t j = 5; j < scans_row_data_vec[i].size() - 5; j ++ ) {
				typename CloudType::PointType::ValueType diff_x = scans_row_data_vec[i][j - 5].x + scans_row_data_vec[i][j - 4].x + scans_row_data_vec[i][j - 3].x + scans_row_data_vec[i][j - 2].x + scans_row_data_vec[i][j - 1].x - 10 * scans_row_data_vec[i][j].x + scans_row_data_vec[i][j + 1].x + scans_row_data_vec[i][j + 2].x + scans_row_data_vec[i][j + 3].x + scans_row_data_vec[i][j + 4].x + scans_row_data_vec[i][j + 5].x;

				typename CloudType::PointType::ValueType diff_y = scans_row_data_vec[i][j - 5].y + scans_row_data_vec[i][j - 4].y + scans_row_data_vec[i][j - 3].y + scans_row_data_vec[i][j - 2].y + scans_row_data_vec[i][j - 1].y - 10 * scans_row_data_vec[i][j].y + scans_row_data_vec[i][j + 1].y + scans_row_data_vec[i][j + 2].y + scans_row_data_vec[i][j + 3].y + scans_row_data_vec[i][j + 4].y + scans_row_data_vec[i][j + 5].y;

				typename CloudType::PointType::ValueType diff_z = scans_row_data_vec[i][j - 5].z + scans_row_data_vec[i][j - 4].z + scans_row_data_vec[i][j - 3].z + scans_row_data_vec[i][j - 2].z + scans_row_data_vec[i][j - 1].z - 10 * scans_row_data_vec[i][j].z + scans_row_data_vec[i][j + 1].z + scans_row_data_vec[i][j + 2].z + scans_row_data_vec[i][j + 3].z + scans_row_data_vec[i][j + 4].z + scans_row_data_vec[i][j + 5].z;
			
				scans_row_curv_vec[i][j] = diff_x * diff_x + diff_y * diff_y + diff_z * diff_z;
			} 		
		}
	
		// 3. 

		for ( size_t i = Config::Row_Index_Start; i < Config::N_SCANS - Config::Row_Index_End; i ++ ) {
			int j_start_index = 0;
			//int window_interval = static_cast<int>( scans_row_data_vec[i].size() / 6 );
			
			for ( size_t j = 0; j < scans_row_data_vec[i].size(); j ++ ) {
				if ( j >= j_start_index ) {
					// plane feature
					if ( scans_row_curv_vec[i][j] < 0.05 ) {
						output_cloud1_refer->points.push_back( input_cloud.points[ point_index_vec[i][j] ] );	
					}
					// corner feature
					else if( scans_row_curv_vec[i][j] > 2.0 ) {
						output_cloud2_refer->points.push_back( input_cloud.points[ point_index_vec[i][j] ] );
					}

					//j_start_index = j + window_interval;
					j_start_index = j + 10;
				}
			}
		}
		
		//std::cout<<"output_cloud1_refer.points.size() = "<<output_cloud1_refer->points.size()<<std::endl;
		//std::cout<<"output_cloud2_refer.points.size() = "<<output_cloud2_refer->points.size()<<std::endl;

		output_cloud1_refer->width = output_cloud1_refer->points.size();
		output_cloud1_refer->height = 1;

		output_cloud2_refer->width = output_cloud2_refer->points.size();
                output_cloud2_refer->height = 1;


        	this->output_clouds_ptrs_vec_.clear();
	}

	template<typename CloudType>
        void extractFeaturesFromCloud( const CloudType& input_cloud,
                                       CloudType& output_cloud1, 
                                       CloudType& output_cloud2, 
                                       std::vector<int>& scan_ids_vec )
        {
		std::vector<std::vector<typename CloudType::PointType>> scans_row_data_vec( Config::N_SCANS );
                std::vector<std::vector<typename CloudType::PointType::ValueType>> point_index_vec( Config::N_SCANS );
                std::vector<std::vector<typename CloudType::PointType::ValueType>> scans_row_curv_vec( Config::N_SCANS );
                std::vector<int> point_scan_id_vec( input_cloud.points.size(), -1 );

                std::cout<<"input_cloud.points.size() = "<<input_cloud.points.size()<<std::endl;
                for ( size_t i = 0; i < input_cloud.points.size(); i ++ ) {
                        typename CloudType::PointType pt = input_cloud.points[i];

                        int scan_idx = -1;
                        typename CloudType::PointType::ValueType angle = ::atan2( pt.z, ::sqrt( pt.x * pt.x + pt.y * pt.y ) ) * 180 / M_PI;


                        if constexpr ( Config::N_SCANS == 16 ) {
                                if ( angle >= -15 && angle <= 15 ) {
                                        scan_idx = static_cast<int>( ( angle + 15 ) / 2 + 0.5 );
                                }
                        }
                        else if constexpr( Config::N_SCANS == 64 ) {
                                if ( angle >= -24.33 && angle <= 2 ) {
                                        if ( angle >= -8.83 ) {
                                                scan_idx = static_cast<int>( ( 2 - angle ) * 3.0 + 0.5 );
                                        }
                                        else {
                                                scan_idx = Config::N_SCANS / 2 + static_cast<int>( ( -8.83 - angle ) * 2.0 + 0.5 );
                                        }
                                }
                        }

                        if ( scan_idx > -1 && scan_idx < Config::N_SCANS ) {
                                scans_row_data_vec[scan_idx].push_back( pt );
                                point_index_vec[scan_idx].push_back( i );

                                point_scan_id_vec[i] = scan_idx;
                        }
                }

		for ( size_t i = Config::Row_Index_Start; i < Config::N_SCANS - Config::Row_Index_End; i ++ ) {
                        scans_row_curv_vec[i].resize( scans_row_data_vec[i].size() );
                        std::cout<<"scan idx = "<<i<<std::endl;

                        std::cout<<"scans_row_data_vec["<<i<<"].size() = "<<scans_row_data_vec[i].size()<<std::endl;

                        for( size_t j = 5; j < scans_row_data_vec[i].size() - 5; j ++ ) {
                                typename CloudType::PointType::ValueType diff_x = scans_row_data_vec[i][j - 5].x + scans_row_data_vec[i][j - 4].x + scans_row_data_vec[i][j - 3].x + scans_row_data_vec[i][j - 2].x + scans_row_data_vec[i][j - 1].x - 10 * scans_row_data_vec[i][j].x + scans_row_data_vec[i][j + 1].x + scans_row_data_vec[i][j + 2].x + scans_row_data_vec[i][j + 3].x + scans_row_data_vec[i][j + 4].x + scans_row_data_vec[i][j + 5].x;

                                typename CloudType::PointType::ValueType diff_y = scans_row_data_vec[i][j - 5].y + scans_row_data_vec[i][j - 4].y + scans_row_data_vec[i][j - 3].y + scans_row_data_vec[i][j - 2].y + scans_row_data_vec[i][j - 1].y - 10 * scans_row_data_vec[i][j].y + scans_row_data_vec[i][j + 1].y + scans_row_data_vec[i][j + 2].y + scans_row_data_vec[i][j + 3].y + scans_row_data_vec[i][j + 4].y + scans_row_data_vec[i][j + 5].y;

                                typename CloudType::PointType::ValueType diff_z = scans_row_data_vec[i][j - 5].z + scans_row_data_vec[i][j - 4].z + scans_row_data_vec[i][j - 3].z + scans_row_data_vec[i][j - 2].z + scans_row_data_vec[i][j - 1].z - 10 * scans_row_data_vec[i][j].z + scans_row_data_vec[i][j + 1].z + scans_row_data_vec[i][j + 2].z + scans_row_data_vec[i][j + 3].z + scans_row_data_vec[i][j + 4].z + scans_row_data_vec[i][j + 5].z;

                                scans_row_curv_vec[i][j] = diff_x * diff_x + diff_y * diff_y + diff_z * diff_z;
                        }
                }
	
		for ( size_t i = Config::Row_Index_Start; i < Config::N_SCANS - Config::Row_Index_End; i ++ ) {
                        int j_start_index = 0;

                        for ( size_t j = 0; j < scans_row_data_vec[i].size(); j ++ ) {
                                if ( j >= j_start_index ) {
                                        // plane feature
                                        if ( scans_row_curv_vec[i][j] < 0.05 ) {
                                                output_cloud1.points.push_back( input_cloud.points[ point_index_vec[i][j] ] );

                                                scan_ids_vec.push_back( point_scan_id_vec[ point_index_vec[i][j] ] );
                                        }
                                        // corner feature
                                        else if( scans_row_curv_vec[i][j] > 2.0 ) {
                                                output_cloud2.points.push_back( input_cloud.points[ point_index_vec[i][j] ] );
                                        }

                                        //j_start_index = j + window_interval;
                                        j_start_index = j + 10;
                                }
                        }
                }


                output_cloud1.width = output_cloud1.points.size();
                output_cloud1.height = 1;

                output_cloud2.width = output_cloud2.points.size();
                output_cloud2.height = 1;	
        }
	

};

template<typename DerivedType, typename InputCloudType, typename ...OutputCloudsType>
void extractFeaturesFromCloud( FeatureBase<DerivedType> instance, const InputCloudType& input_cloud, OutputCloudsType&&... output_clouds )
{

        instance.extractFeaturesFromCloud( input_cloud, output_clouds... );
}


}

#endif
