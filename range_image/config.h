#ifndef __CONFIG_H
#define __CONFIG_H

namespace slam
{

class Config
{
public:
	static constexpr int N_SCANS = 64;
	static constexpr int N_HORIZON = 4000;
	static constexpr float Angular_Resolution_Horizon = 0.09; // 360 / 4000
	static constexpr float Angular_Resolution_Vertical = 0.41875; // 26.8/64
	static constexpr float Angle_Bottom = 13.4;
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
