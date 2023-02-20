#ifndef __SO3_H
#define __SO3_H

#include <iostream>
#include <Eigen/Dense>

namespace slam
{

template<typename T>
class State
{
public:
	using ValueType = T;
	using Vector3 = typename Eigen::Matrix<ValueType, 3, 1>;
	using Matrix3x3 = typename Eigen::Matrix<ValueType, 3, 3>;
	using Matrix4x4 = typename Eigen::Matrix<ValueType, 4, 4>;

	using TransformationType = typename Matrix4x4;
	using RotationType = typename Matrix3x3;
	using Translation = typename Vector3;

};



class SO3
{
public:
	template<typename T>
	static const State<T>::Matrix3x3 hat( const State<T>::Vector3& vector ) 
	{
		State<T>::Matrix3x3 hat_mat;
		hat_mat << 0, -vector(2), vector(1),
			   vector(2), 0, -vector(0),
			   -vector(1), vector(0), 0;

		return hat_mat;
	}

	template<typename T>
	static const State<T>::RotationType exp( const State<T>::Vector3& rotation_vector ) 
	{
		
		Eigen::AngleAxisd rotation_vec( rotation_vector.norm(), rotation_vector.normalized() );
		return rotation_vec.toRotationMatrix();
	}
};



}

#endif
