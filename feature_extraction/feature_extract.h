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
	void extractFeaturesFromCloud( const InputCloudType& input_cloud, OutputCloudsType... output_clouds )
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


protected:
        template<typename OutputCloudType, typename ...OutputCloudsType>
        void extractFeaturesFromCloudHelper( OutputCloudType&& ouput_cloud, OutputCloudsType... output_clouds )
        {
		std::cout<<"typeid (output_cloud) = "<<typeid(ouput_cloud).name()<<std::endl;
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
        void extractFeaturesFromCloud( const InputCloudType& input_cloud, OutputCloudsType... output_clouds )
        {
		static_assert( !(sizeof...( output_clouds ) != 2 ), "The number of arguments is too few !");
		using CloudType = InputCloudType;

                std::cout<<"Corner Planner Feature !"<<std::endl;

		// parse the paramters
                this->extractFeaturesFromCloudHelper( output_clouds... );
		
		std::cout<<"typeid( InputCloudType ) = "<<typeid(InputCloudType).name()<<std::endl;

		// 1. 
		std::vector<std::vector<typename CloudType::PointType>> scans_row_data_vec( Config::N_SCANS );
		for ( size_t i = 0; i < input_cloud.points.size(); i ++ ) {
			typename CloudType::PointType pt = input_cloud.points[i];
			
			int scan_idx = -1;
			typename CloudType::PointType::ValueType angle = ::atan2( pt.z, ::sqrt( pt.x * pt.x + pt.y * pt.y ) );
	
							
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
				
			}
			
		}
	

		auto output_cloud1_ptr = *std::any_cast<typename std::remove_reference<CloudType>::type *>( this->output_clouds_ptrs_vec_[0] );
		auto output_cloud2_ptr = *std::any_cast<typename std::remove_reference<CloudType>::type *>( this->output_clouds_ptrs_vec_[1] );
        }

};

template<typename DerivedType, typename InputCloudType, typename ...OutputCloudsType>
void extractFeaturesFromCloud( FeatureBase<DerivedType> instance, const InputCloudType& input_cloud, OutputCloudsType... output_clouds )
{

        instance.extractFeaturesFromCloud( input_cloud, output_clouds... );
}


}

#endif
