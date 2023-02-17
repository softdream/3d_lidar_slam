#ifndef __FEATURE_EXTRACT_H
#define __FEATURE_EXTRACT_H

#include <vector>
#include "point_cloud.h"

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
		if constexpr ( auto ptr = static_cast<DerivedType*>( this ) ) {
			ptr->extractFeaturesFromCloud( input_cloud, output_clouds... );
		}
	}
	
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
                std::cout<<"Corner Planner Feature !"<<std::endl;

                this->extractFeaturesFromCloudHelper( input_cloud, output_clouds... );

        }

private:
	template<typename OutputCloudType, typename ...OutputCloudsType>
        void extractFeaturesFromCloudHelper( OutputCloudType&& ouput_cloud, OutputCloudsType... output_clouds )
        {
                extractFeaturesFromCloudHelper( output_clouds... );
        }

        void extractFeaturesFromCloudHelper()
        {

        }

};

template<typename DerivedType, typename InputCloudType, typename ...OutputCloudsType>
void extractFeaturesFromCloud( FeatureBase<DerivedType> instance, const InputCloudType& input_cloud, OutputCloudsType... output_clouds )
{

        instance.extractFeaturesFromCloud( input_cloud, output_clouds... );
}


}

#endif
