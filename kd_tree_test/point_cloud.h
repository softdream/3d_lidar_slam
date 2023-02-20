#ifndef __POINT_CLOUD_H
#define __POINT_CLOUD_H

#include <stdint.h>
#include <vector>

#include <type_traits>

namespace slam
{

template<typename T>
struct is_numerical_type
{
        static const bool value = false;
};

template<>
struct is_numerical_type<float>
{
        static const bool value = true;
};

template<>
struct is_numerical_type<double>
{
        static const bool value = true;
};

template<>
struct is_numerical_type<uint8_t>
{
	static const bool value = true;
};

template<>
struct is_numerical_type<int8_t>
{
	static const bool value = true;
};

template<>
struct is_numerical_type<uint32_t>
{
        static const bool value = true;
};


template<>
struct is_numerical_type<int32_t>
{
        static const bool value = true;
};



template<typename T, typename = typename std::enable_if<is_numerical_type<T>::value>::type>
struct Point3_
{
	using ValueType = T;

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

using Point3I = Point3<int>;
using Point3D = Point3<double>;
using Point3F = Point3<float>;

template<typename T>
struct PointCloud_
{
	using PointType = T;

	PointCloud_()
	{

	}
	
	PointCloud_( const uint64_t& time_stamp_, const int& width_, const int& height_ ) : time_stamp( time_stamp_ ), width( width_ ), height( height_ )
	{
		points.resize( width * height );
	}
		

	uint64_t time_stamp = -1;
	int width = 0;
	int height = 0;

	//T* points = nullptr;
	std::vector<T> points;
};

template<typename T>
using PointCloud = PointCloud_<T>;

using PointCloudI = PointCloud<Point3I>;
using PointCloudD = PointCloud<Point3D>;
using PointCloudF = PointCloud<Point3F>;

}

#endif
