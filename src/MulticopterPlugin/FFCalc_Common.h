/**
   @author Japan Atomic Energy Agency
*/

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <string>
#include <sstream>

namespace Multicopter {
namespace FFCalc {

using namespace Multicopter;
typedef Eigen::Matrix3d Matrix3;
typedef Eigen::Vector3d Vector3;
typedef cnoid::Isometry3 Transform3;

template<typename T> std::string msgvalue (const std::string& message, const T& value)
{
	std::ostringstream ost;
	ost << message << value;
	return ost.str();
}


}}
