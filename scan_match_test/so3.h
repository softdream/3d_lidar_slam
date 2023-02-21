#ifndef __SO3_H
#define __SO3_H

#include <iostream>
#include <Eigen/Dense>

namespace slam
{

class SO3
{
public:
	template<typename T>
        using Vector3 = typename Eigen::Matrix<T, 3, 1>;

	template<typename T>
        using Matrix3x3 = typename Eigen::Matrix<T, 3, 3>;
	
	template<typename T>
        using Matrix4x4 = typename Eigen::Matrix<T, 4, 4>;

	template<typename T>
	static const Matrix3x3<T> hat( const Vector3<T>& vector ) 
	{
		Matrix3x3<T> hat_mat;
		hat_mat << 0, -vector(2), vector(1),
			   vector(2), 0, -vector(0),
			   -vector(1), vector(0), 0;
		return hat_mat;
	}

	template<typename T>
	static const Matrix3x3<T> exp( const Vector3<T>& rotation_vector ) 
	{
		
		Eigen::AngleAxisd rotation_vec( rotation_vector.norm(), rotation_vector.normalized() );
		return rotation_vec.toRotationMatrix();
	}
};



}

#endif
