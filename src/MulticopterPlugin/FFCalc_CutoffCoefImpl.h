/**
   @author Japan Atomic Energy Agency
*/

#pragma once
#include "FFCalc_Common.h"
#include "FFCalc_GaussTriangle3d.h"

namespace Multicopter{
namespace FFCalc {

class CutoffCoefImpl
{
private:

	CutoffCoefImpl();
	CutoffCoefImpl(CutoffCoefImpl&);
	CutoffCoefImpl(CutoffCoefImpl&&);
	CutoffCoefImpl& operator= (const CutoffCoefImpl&);
	CutoffCoefImpl& operator= (CutoffCoefImpl&&);

private:
    static const double _rbarb;

    static const double _prc_eps;

	double _prc;

	std::unique_ptr<INormalizedFunction> _fcut;

	std::unique_ptr<INormalizedFunction> _fcorr;

public:

	CutoffCoefImpl (
		const double cutoffDistance,
		std::unique_ptr<INormalizedFunction>&& funcNormCutoff,
		std::unique_ptr<INormalizedFunction>&& funcNormCorr);

	static CutoffCoefImpl* createInstance (
		const double cutoffDistance,
		const double normMiddleValue);

	double get (
		const Vector3& point,
		const Vector3& normalDir,
        const GaussTriangle3d& tri) const;

	double eval (const double distance) const;

#ifndef NDEBUG

	double funcNormCutoff (const double rbar) const;

	double funcNormCorrection (const double rbar) const;
#endif
};



}}
