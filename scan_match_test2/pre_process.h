#ifndef __PRE_PROCESS_h
#define __PRE_PROCESS_h

#include "point_cloud.h"

namespace slam
{

class PreProcess
{
public:
	template<typename T>
	static void removeClosedPoints( PointCloud<T>& cloud, const T thresh )
	{
		size_t j = 0;
		for( size_t i = 0; i < cloud.points.size(); i ++ ) {
			if( cloud.points[i].x * cloud.points[i].x + cloud.points[i].y * cloud.points[i].y + cloud.points[i].z * cloud.points[i].z < thresh ) {
				continue;
			}		
			
			cloud.points[j] = cloud.points[i];
			j ++;
		}
		
		if( j != cloud.points.size() ) {
			cloud.points.resize( j );
		}
	}

};

}

#endif
