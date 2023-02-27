#ifndef __CONFIG_H
#define __CONFIG_H

namespace slam
{

class Config
{
public:
	static constexpr int N_SCANS = 16;
	static constexpr int Row_Index_Start = 0;
	static constexpr int Row_Index_End = 0;

	template<typename T>
	static constexpr T plane_feature_thresh = 0.5;
	
	template<typename T>
	static constexpr T corner_feature_thresh = 2.0;

	template<typename T>
	static constexpr T icp_pt_min_dist_thresh = 1.0;
};

//constexpr int Config::N_SCANS = 16;

}

#endif
