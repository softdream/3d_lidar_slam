#ifndef __KD_TREE_POINT_CLOUD_ADAPTER_H
#define __KD_TREE_POINT_CLOUD_ADAPTER_H

#include "point_cloud.h"
#include "nanoflann.hpp"

namespace slam {

namespace kdtree{

template<typename Derived>
struct KdTreePointCloudAdaptor
{
	using coord_t = typename Derived::PointType::ValueType;

	const Derived& obj_;

	KdTreePointCloudAdaptor( const Derived& obj ) : obj_( obj )
	{
	
	}

	inline const Derived& derived() const 
	{
		return obj_;
	}

	inline size_t kdtree_get_point_count() const
    	{
        	return derived().points.size();
    	}

	inline coord_t kdtree_get_pt(const size_t idx, const size_t dim) const
    	{
        	if (dim == 0)
            		return derived().points[idx].x;
        	else if (dim == 1)
            		return derived().points[idx].y;
        	else
            		return derived().points[idx].z;
    	}

	template <class BBOX>
    	bool kdtree_get_bbox(BBOX& /*bb*/) const
    	{
       	 	return false;
    	}

	const std::vector<coord_t> point2vec( const int index ) const
	{
		return { obj_.points[index].x, obj_.points[index].y, obj_.points[index].z };
	}
};

template<typename T>
using KdTreePointCloudType = KdTreePointCloudAdaptor<PointCloud<Point3<T>>>;

template<typename T>
using KdTreeType = nanoflann::KDTreeSingleIndexAdaptor<
        		nanoflann::L2_Simple_Adaptor<T,
		       				     KdTreePointCloudType<T>>, 
						     KdTreePointCloudType<T>, 3 /* dim */
        	   >;

}

}
#endif
