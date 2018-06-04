/**
   @author Japan Atomic Energy Agency
*/

#include "MulticopterPluginHeader.h"

namespace Multicopter {
namespace FFCalc {

 
CutoffCoef::CutoffCoef (
		const double cutoffDistance,
		const double normMiddleValue) :
	_impl (CutoffCoefImpl::createInstance (cutoffDistance, normMiddleValue)) { }

CutoffCoef::CutoffCoef (CutoffCoef&& rhs) :
	_impl (std::move(rhs._impl)) { }

CutoffCoef::~CutoffCoef()
{
	delete _impl;
}

void CutoffCoef::initialize (
		const double cutoffDistance,
		const double normMiddleValue)
{
	delete _impl;
	_impl = CutoffCoefImpl::createInstance (cutoffDistance, normMiddleValue);
	return;
}

double CutoffCoef::get (
	const Vector3& point,
	const Vector3& normalDir,
    const GaussTriangle3d& tri) const
{
    return _impl->get (point, normalDir, tri);
}

}}
