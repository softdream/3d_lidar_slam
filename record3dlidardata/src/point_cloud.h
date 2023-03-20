#ifndef __POINT_CLOUD_H
#define __POINT_CLOUD_H

namespace slam
{

template<typename T>
struct Point3_
{
	Point3_()
	{

	}

	~Point3_()
	{

	}

	Point3_( const T& x_, const T& y_, const T& z_ ) : x( x_ ), y( y_ ), z( z_ )
	{

	}	
	
	T x = 0;
	T y = 0;
	T z = 0;	
};

template<typename T>
using Point3 = Point3_<T>;

using Point3i = Point3<int>;
using Point3d = Point3<double>;
using Point3f = Point3<float>;

template<typename T>
struct PointCloud
{
	PointCloud()
	{

	}
	
	PointCloud( const uint64_t& time_stamp_, const int& width_, const int& height_ ) : time_stamp( time_stamp_ ), width( width_ ), height( height_ )
	{
		points.resize( width * height );
	}
		

	uint64_t time_stamp = -1;
	int width = 0;
	int height = 0;

	//T* points = nullptr;
	std::vector<T> points;
};

}

#endif
