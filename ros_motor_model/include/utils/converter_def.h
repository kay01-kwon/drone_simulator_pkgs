#ifndef CONVERTER_DEF_H
#define CONVERTER_DEF_H

#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

using std::cout;
using std::endl;

#define RPS_TO_RPM 60.0/(2*M_PI)
#define RPM_TO_RPS 2*M_PI/60.0

double MAX_BIT = 8191;
double MAX_RPM = 9800.0;

#endif // CONVERTER_DEF_H