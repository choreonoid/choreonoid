/**
   @author Japan Atomic Energy Agency
*/

#include "MulticopterPluginHeader.h"

namespace Multicopter {
namespace FFCalc {

typedef GaussQuadratureTriangle GQTri;
typedef GQTri::PointParameter   GQParam;


Vector3 GQParam::getPosition (
	const Vector3& vertex1,
	const Vector3& vertex2,
	const Vector3& vertex3) const
{
	return vertex1*coef1 + vertex2*coef2 + vertex3*coef3;
}


const std::array<GQParam,4> GQTri::param4 =
{
    GQParam (-27.0/48.0, 1.0/3.0, 1.0/3.0, 1.0/3.0),
    GQParam (+25.0/48.0, 3.0/5.0, 1.0/5.0, 1.0/5.0),
    GQParam (+25.0/48.0, 1.0/5.0, 3.0/5.0, 1.0/5.0),
    GQParam (+25.0/48.0, 1.0/5.0, 1.0/5.0, 3.0/5.0)
};

const std::array<GQParam,1> GQTri::param1 =
{
    GQParam (1.0, 1.0/3.0, 1.0/3.0, 1.0/3.0),
};


}}
