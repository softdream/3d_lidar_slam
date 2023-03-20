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
                static_assert( !(sizeof...( output_clouds ) != 1 ), "The number of arguments is too few !");
                using CloudType = InputCloudType;
		using ValueType = typename CloudType::PointType::ValueType;

                std::cout<<"Range Image Feature Feature !"<<std::endl;

                // parse the paramters
                this->extractFeaturesFromCloudHelper( output_clouds... );

                auto output_cloud1_refer = std::any_cast<typename std::remove_reference<CloudType>::type *>( this->output_clouds_ptrs_vec_[0] );
                //auto output_cloud2_refer = std::any_cast<typename std::remove_reference<CloudType>::type *>( this->output_clouds_ptrs_vec_[1] );

		// 1. get the range image of the input point cloud
		typename RangeImage<ValueType>::RangeImageType range_image;
		RangeImage<ValueType>::generateRangeImage( input_cloud, range_image );

		// 2. feature exctraction from the range image
		for ( int i = 2; i < range_image.mat.rows() - 2; i += 5 ) {
			for ( int j = 2; j < range_image.mat.cols() - 2; j += 5 ) {
				if ( range_image.mat( i, j ) == 1000 ) continue; // invalid point

				// 2.1 dot product
				int valid_pt_num = 0;
				ValueType dot_ret = 0;
				for ( int m = 0; m < 5; m ++ ) {
					for ( int n = 0; n < 5; n ++ ) {
						auto r = range_image.mat( i - 2 + m, j - 2 + n );
						if ( r != 1000 && ( ( i - 2 + m != i ) && ( j - 2 + n != j ) ) ) {
							dot_ret += r;
							valid_pt_num ++;
						}
					}
				}
				//std::cout<<"valid num = "<<valid_pt_num<<std::endl;
				//std::cout<<"range_image.mat("<<i<<", "<<j<<") = "<< range_image.mat( i, j )<<std::endl;
				//std::cout<<"dot ret = "<<dot_ret<<std::endl;
				ValueType curv = dot_ret - static_cast<ValueType>( valid_pt_num ) * range_image.mat( i, j );
				if ( std::abs( curv ) < 0.5 ) { 
					//std::cout<<"curv = "<<curv<<std::endl;
					//output_cloud1_refer->points.push_back( input_cloud.points[range_image.idx_map[{i, j}]] );
				}
				else if ( std::abs( curv ) > 2 ) {
					//std::cout<<"curv = "<<curv<<std::endl;
					//output_cloud2_refer->points.push_back( input_cloud.points[range_image.idx_map[{i, j}]] );
				}
				output_cloud1_refer->points.push_back( input_cloud.points[range_image.idx_map[{i, j}]] );
			} 
		}
				
	}	

private:


};

}

#endif
