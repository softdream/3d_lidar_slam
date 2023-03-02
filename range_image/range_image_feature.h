#ifndef __RANGE_IMAGE_FEATURE_H
#define __RANGE_IMAGE_FEATURE_H

#include "feature_extract.h"
#include "range_image.h"

namespace slam
{

class RangeImageFeature : public FeatureBase<RangeImageFeature>
{
public:
	RangeImageFeature()
	{
	
	}

	template<typename InputCloudType, typename ...OutputCloudsType>
        void extractFeaturesFromCloud( const InputCloudType& input_cloud, OutputCloudsType&&... output_clouds )
        {
                static_assert( !(sizeof...( output_clouds ) != 2 ), "The number of arguments is too few !");
                using CloudType = InputCloudType;
		using ValueType = typename CloudType::PointType::ValueType;

                std::cout<<"Range Image Feature Feature !"<<std::endl;

                // parse the paramters
                this->extractFeaturesFromCloudHelper( output_clouds... );

                auto output_cloud1_refer = std::any_cast<typename std::remove_reference<CloudType>::type *>( this->output_clouds_ptrs_vec_[0] );
                auto output_cloud2_refer = std::any_cast<typename std::remove_reference<CloudType>::type *>( this->output_clouds_ptrs_vec_[1] );

		// 1. get the range image of the input point cloud
		RangeImage<ValueType>::RangeImageType range_image;
		RangeImage<ValueType>::generateRangeImage( input_cloud, range_image );

		// 2. feature exctraction from the range image
		Eigen::Matrix<ValueType, 5, 5> kernel;
		kernel << 0.04, 0.04, 0.04, 0.04, 0.04,
	       		  0.04, 0.04, 0.04, 0.04, 0.04,	
		   	  0.04, 0.04, 0.04, 0.04, 0.04,
			  0.04, 0.04, 0.04, 0.04, 0.04,
			  0.04, 0.04, 0.04, 0.04, 0.04;
			
		for ( int i = 2; i < range_image.mat.rows() - 2; i += 5 ) {
			for ( int j = 2; j < range_image.mat.cols() - 2; j += 5 ) {
				// 2.1 dot product
				ValueType dot_ret = 0;
				for ( int m = 0; m < 5; m ++ ) {
					for ( int n = 0; n < 5; n ++ ) {
						dot_ret += kernel( m, n ) * range_image.mat( i - 2 + m, j - 2 + n );
					}
				}
			} 
		}
				
	}	

private:


};

}

#endif
