#ifndef TYPE_DEF_H
#define TYPE_DEF_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>


typedef Eigen::Matrix<double, 2, 1> Vector2d;
typedef Eigen::Matrix<double, 2, 2> Matrix2x2d;

enum class UAVType
{
    QUAD,
    HEXA
};



#endif // TYPE_DEF_H