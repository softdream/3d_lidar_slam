#ifndef __CONFIG_H
#define __CONFIG_H

namespace slam
{

class Config
{
public:
	// lidar device parameters
	static constexpr int N_SCANS = 64; // for Velodyne 64

	static constexpr int sample_interval = 20;

	template<typename T>
	static constexpr T horizon_angle_resoluation = 0.09;
	
	template<typename T>
	static constexpr T vertical_angle_resolution = 0.4114; // 26.33 / 64

	template<typename T>
	static constexpr T vertical_angle_min = -24.33;
		
	template<typename T>
	static constexpr T vertical_angle_max = 2;

	template<typename T>
	static constexpr T range_max = 100;

	template<typename T>
	static constexpr T range_min = 1;

	// feature extraction paramters
	static constexpr int Row_Index_Start = 0;
        static constexpr int Row_Index_End = 0;	

	template<typename T>
	static constexpr T plane_feature_thresh = 0.5;
	
	template<typename T>
	static constexpr T corner_feature_thresh = 2.0;

	template<typename T>
	static constexpr T icp_pt_min_dist_thresh = 1;

};

//constexpr int Config::N_SCANS = 16;

}

#endif
