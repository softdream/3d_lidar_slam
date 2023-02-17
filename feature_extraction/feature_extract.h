#ifndef __FEATURE_EXTRACT_H
#define __FEATURE_EXTRACT_H

#include <vector>

#include <any>
#include <iostream>

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

protected:
        template<typename OutputCloudType, typename ...OutputCloudsType>
        void extractFeaturesFromCloudHelper( OutputCloudType&& ouput_cloud, OutputCloudsType... output_clouds )
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
        void extractFeaturesFromCloud( const InputCloudType& input_cloud, OutputCloudsType... output_clouds )
        {
		static_assert( !(sizeof...( output_clouds ) != 2 ), "The number of arguments is too few !");

                std::cout<<"Corner Planner Feature !"<<std::endl;

		// parse the paramters
                this->extractFeaturesFromCloudHelper( input_cloud, output_clouds... );
	
		
        }

};

template<typename DerivedType, typename InputCloudType, typename ...OutputCloudsType>
void extractFeaturesFromCloud( FeatureBase<DerivedType> instance, const InputCloudType& input_cloud, OutputCloudsType... output_clouds )
{

        instance.extractFeaturesFromCloud( input_cloud, output_clouds... );
}


}

#endif
