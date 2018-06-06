/**
   @author Japan Atomic Energy Agency
*/

#include "MulticopterPluginHeader.h"

namespace Multicopter {
namespace FFCalc {

const double CutoffCoefImpl::_rbarb = -5.0;

const double CutoffCoefImpl::_prc_eps = 1.0e-9;

CutoffCoefImpl::CutoffCoefImpl (
		const double cutoffDistance,
		std::unique_ptr<INormalizedFunction>&& funcNormCutoff,
		std::unique_ptr<INormalizedFunction>&& funcNormCorr) :
	_prc(cutoffDistance), _fcut(std::move(funcNormCutoff)), _fcorr(std::move(funcNormCorr))
{
	if (_prc <= 0.0)
		throw std::runtime_error (msgvalue (
			"Parameter prc must be positive: given prc=", _prc).c_str());
	return;
}

CutoffCoefImpl* CutoffCoefImpl::createInstance (
	const double cutoffDistance,
	const double normMiddleValue)
{

    if (cutoffDistance < -_prc_eps)
    {
        std::unique_ptr<INormalizedFunction> fcut  (new NoNormCutoffFunc());
        std::unique_ptr<INormalizedFunction> fcorr (new DefaultNormCorrectionFunc(0.5));
        CutoffCoefImpl* impl = new CutoffCoefImpl (1.0, std::move(fcut), std::move(fcorr));
        return impl;
    }

    const double  prc    = std::max (cutoffDistance, _prc_eps);
    const double& prmbar = normMiddleValue;

    std::unique_ptr<INormalizedFunction> fcut (new DefaultNormCutoffFunc());
    assert (std::abs (fcut->eval(0.0)-0.0) <= 1.0e-12);
    assert (std::abs (fcut->eval(1.0)-1.0) <= 1.0e-12);
    assert (std::abs (fcut->eval(0.5)-0.5) <= 1.0e-12);

    assert (0.0001<=prmbar && prmbar<=0.9999);
    std::unique_ptr<INormalizedFunction> fcorr (new DefaultNormCorrectionFunc(prmbar));
    assert (std::abs (fcorr->eval(0.0)-0.0) <= 1.0e-12);
    assert (std::abs (fcorr->eval(1.0)-1.0) <= 1.0e-12);
    assert (std::abs (fcorr->eval(prmbar)-0.5) <= 1.0e-12);

    CutoffCoefImpl* impl = new CutoffCoefImpl (prc, std::move(fcut), std::move(fcorr));

    assert (std::abs (impl->eval(0.0*prc)-0.0) <= 1.0e-12);
    assert (std::abs (impl->eval(1.0*prc)-1.0) <= 1.0e-12);
    assert (std::abs (impl->eval(prmbar*prc)-0.5) <= 1.0e-12);

    return impl;
}

double CutoffCoefImpl::get (
	const Vector3& point,
	const Vector3& normalDir,
    const GaussTriangle3d& tri
) const {

	const double& rc = _prc;

    const Vector3& vP = point;
    const Vector3& vA = tri[0];
    const Vector3& vB = tri[1];
    const Vector3& vC = tri[2];
    const Vector3& ePerpP   = normalDir;
    const Vector3& ePerpTri = tri.normal();

    if (ePerpP.dot(ePerpTri) > -0.5)
        return this->eval(_prc*5.0);

    Matrix3 mat;
    mat << vB[0]-vA[0], vC[0]-vA[0], ePerpTri[0],
           vB[1]-vA[1], vC[1]-vA[1], ePerpTri[1],
           vB[2]-vA[2], vC[2]-vA[2], ePerpTri[2];
    Vector3 coef = Eigen::FullPivLU<Matrix3>(mat).solve(vP-vA);
    const double cB = coef[0];
    const double cC = coef[1];
    const double cA = 1.0 - cB - cC;
    const double cPerpTri = coef[2];

    if (cPerpTri > rc)
        return this->eval(_prc*5.0);

    if (cA>=-1.0e-16 && cB>=-1.0e-16 && cC>=-1.0e-16)
        return this->eval(cPerpTri);

    const Vector3 vH = vP - ePerpTri * cPerpTri;

    int sft_index = (cA<0.0 ? 0 : (cB<0.0 ? 1 : 2));
    const Vector3* ptr_vertex[] = {&vA, &vB, &vC};
    const Vector3& vD = *(ptr_vertex[(0+sft_index)%3]);
    const Vector3& vE = *(ptr_vertex[(1+sft_index)%3]);
    const Vector3& vF = *(ptr_vertex[(2+sft_index)%3]);

    const Vector3 vEH = vH - vE;
    const Vector3 vEF = vF - vE;
    const Vector3 ePerpEF = (vEF.cross(ePerpTri)).normalized();
    const double  cPerpEF = vEH.dot(ePerpEF);
    const double  cF = vEH.dot(vEF) / vEF.dot(vEF);

	double distance;
    if (0.0<=cF && cF<=1.0)
        distance = std::sqrt(cPerpTri*cPerpTri + cPerpEF*cPerpEF);
    else if (cF<0.0)
        distance = (vP - vE).norm();
	else
        distance = (vP - vF).norm();
	return this->eval(distance);
}

double CutoffCoefImpl::eval(const double distance) const
{
    const double& rbarb = _rbarb;
    const double& rc    = _prc;

    double rbar =  distance / _prc;
    rbar = std::max (rbar, rbarb-rbar);

    if (rbar<=0.0)
        rbar = 0.0;
    if (rbar>=1.0)
        rbar = 1.0;

	rbar = _fcorr->eval(rbar);
	return _fcut->eval(rbar);
}

#ifndef NDEBUG
double CutoffCoefImpl::funcNormCutoff (const double rbar) const
{
	return _fcut->eval(rbar);
}

double CutoffCoefImpl::funcNormCorrection (const double rbar) const
{
	return _fcorr->eval(rbar);
}
#endif


}}
