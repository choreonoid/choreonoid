/**
   @author Japan Atomic Energy Agency
*/

#pragma once
#include "FFCalc_Common.h"

namespace Multicopter {
namespace FFCalc {

class INormalizedFunction
{
public:
        virtual ~INormalizedFunction();
	virtual double eval(const double rbar) const = 0;
};

class DefaultNormCutoffFunc : public INormalizedFunction
{
public:

	double eval(const double rbar) const;
};

class NoNormCutoffFunc : public INormalizedFunction
{
public:

    double eval(const double rbar) const { return 1.0; }
};

class DefaultNormCorrectionFunc : public INormalizedFunction
{
private:
	DefaultNormCorrectionFunc();

    double _cn;

	static double _eval (const double rbar, const double cn);


public:

	DefaultNormCorrectionFunc (const double normMiddleValue);

	double eval(const double rbar) const;

#ifndef NDEBUG

	double cn() { return _cn; }

	double funcNormCorrection (const double rbar) const;
#endif
};


}}
