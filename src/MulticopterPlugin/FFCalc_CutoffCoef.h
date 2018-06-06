/**
   @author Japan Atomic Energy Agency
*/

#pragma once
#include "FFCalc_Common.h"
#include "FFCalc_GaussTriangle3d.h"

namespace Multicopter{
namespace FFCalc {

class CutoffCoefImpl;

class CutoffCoef
{
private:
	typedef CutoffCoefImpl Impl;
	
    Impl* _impl;

	CutoffCoef();
	CutoffCoef(CutoffCoef&);
	CutoffCoef& operator= (const CutoffCoef&);
	CutoffCoef& operator= (CutoffCoef&&);


public:

	CutoffCoef (
		const double cutoffDistance,
		const double normMiddleValue);

	CutoffCoef (CutoffCoef&& rhs);

	virtual ~CutoffCoef();

	void initialize (
		const double cutoffDistance,
		const double normMiddleValue);

	double get (
		const Vector3& point,
		const Vector3& normalDir,
        const GaussTriangle3d& tri) const;
};


}}
