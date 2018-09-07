/**
   @author Japan Atomic Energy Agency
*/

#pragma once
#include "FFCalc_Common.h"

#include <Eigen/Core>
#include <array>

namespace Multicopter {
namespace FFCalc {

class GaussQuadratureTriangle
{
public:

    class PointParameter
    {
	public:
        const double weight;
        const double coef1;
        const double coef2;
        const double coef3;

        PointParameter (double weight, double coef1, double coef2, double coef3)
			: weight(weight), coef1(coef1), coef2(coef2), coef3(coef3) { }

		Vector3 getPosition (const Vector3& vertex1, const Vector3& vertex2, const Vector3& vertex3) const;
	};

    static const std::array<PointParameter,4> param4;
    static const std::array<PointParameter,1> param1;

};

}}

