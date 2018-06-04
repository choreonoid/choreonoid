/**
   @author Japan Atomic Energy Agency
*/

#pragma once
#include "FFCalc_Common.h"

namespace Multicopter {
namespace FFCalc {

class LinkForce
{
private:

    Vector3 _point;
    Vector3 _force;
    Vector3 _moment;


public:

	LinkForce (const Vector3 loadingPoint)
		: _point(loadingPoint),
		_force(0.0, 0.0, 0.0), _moment(0.0, 0.0, 0.0) { }

	void addForce (const Vector3& force, const Vector3& position)
	{
		_force  += force;
		_moment += (position - _point).cross(force);
		return;
	}

	void add (const LinkForce& linkForce)
	{
		this->addForce  (linkForce._force, linkForce._point);
		this->addMoment (linkForce._moment);
		return;
	}

	void addMoment (const Vector3& moment)
	{
		_moment += moment;
		return;
	}

	const Vector3& point() const
	{
		return _point;
	}

	const Vector3& getForce() const
	{
		return _force;
	}

	const Vector3& getMoment() const
	{
		return _moment;
	}
};




}}
