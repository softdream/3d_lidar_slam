#ifndef __CONFIG_H
#define __CONFIG_H

namespace slam
{

class Config
{
public:
	static constexpr int N_SCANS = 16;
	static constexpr int N_HORIZON = 1800;
	static constexpr float Angular_Resolution_Horizon = 0.2; // 360 / 1800
	static constexpr float Angular_Resolution_Vertical = 2; // 30/(16 -1 )
	static constexpr float Angle_Bottom = -15;
	static constexpr float Max_Range = 100.0;
	static constexpr int Row_Index_Start = 0;
	static constexpr int Row_Index_End = 0;

	template<typename T>
	static constexpr T plane_feature_thresh = 0.5;
	
	template<typename T>
	static constexpr T corner_feature_thresh = 2.0;
};

//constexpr int Config::N_SCANS = 16;

}

#endif
