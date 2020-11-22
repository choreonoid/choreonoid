/**
   @author Japan Atomic Energy Agency
*/

#include "MulticopterPluginHeader.h"

namespace Multicopter {
namespace FFCalc {

namespace
{

	double findParameter (std::function<double(double)> func, double x1, double x2, double ydest)
	{
		double y1 = func(x1);
		double y2 = func(x2);

		for (int count=0; count<10; ++count)
		{
			if (std::abs(y2-ydest) < 1.0e-13)
				return x2;
			if (std::abs(y2-y1) < 1.0e-13)
				throw std::runtime_error("findParameter() didn't converged.");

			double x0, y0;
			x0 = x1; y0 = y1;
			x1 = x2; y1 = y2;
			x2 = x1-(x1-x0)/(y1-y0)*(y1-ydest);
			y2 = func(x2);
		}
		throw std::runtime_error("findParameter() doesn't converged.");
	}
}


INormalizedFunction::~INormalizedFunction()
{

}


double DefaultNormCutoffFunc::eval(const double rbar) const
{
	assert(0.0<=rbar && rbar<=1.0);
	return (-2.0 * rbar + 3.0) * rbar * rbar;
}

double DefaultNormCorrectionFunc::_eval (const double rbar, const double cn)
{
	assert(0.0<=rbar && rbar<=1.0);
	assert(0.0<cn);
	return std::pow (1.0 - std::pow (1.0-rbar, cn), 1.0/cn);
}

DefaultNormCorrectionFunc::DefaultNormCorrectionFunc (const double normMiddleValue)
	: INormalizedFunction()
{
	const double& prmbar = normMiddleValue;

	if (prmbar <= 0.0001-1.0e-12 || 0.9999+1.0e-12 <= prmbar)
		throw std::runtime_error (msgvalue (
			"Parameter prmbar must be [0.0001, 0.9999]: given prmbar=", prmbar).c_str());

	auto func = std::bind (&_eval, prmbar, std::placeholders::_1);

	double cn2 = 1.0-std::log(2.0*prmbar);
	double cn1 = cn2*0.99;

	try
	{
		_cn = findParameter (func, cn1, cn2, 0.5);
	}
	catch (std::runtime_error& ex)
	{
		throw std::runtime_error (msgvalue (
			"Failed to find cn: ", ex.what()).c_str());
	}
}

double DefaultNormCorrectionFunc::eval(const double rbar) const
{
	return _eval (rbar, _cn);
}


}}
