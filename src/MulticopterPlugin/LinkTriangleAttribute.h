/**
   @author Japan Atomic Energy Agency
*/

#pragma once

#include <array>
#include "FFCalc_GaussQuadratureTriangle.h"

namespace Multicopter{

class LinkTriangleAttribute
{
public:
    LinkTriangleAttribute(const Triangle3d& tri){
        _tri = tri;

        for (int idx=0; idx<_cutoffCoefAry.size(); ++idx)
            _cutoffCoefAry[idx] = 1.0;
    }
    
    Triangle3d& triangle(){
        return _tri;
    }

    const Triangle3d& triangle() const{
        return _tri;
    }
    
    int cutoffCoeffientSize() const{ return _cutoffCoefAry.size(); }

    double cutoffCoefficient(int idx) const{
        return _cutoffCoefAry[idx];
    }

    void setCutoffoefficient(int idx, double coef){
        _cutoffCoefAry[idx] = coef;
    }

private:
    Triangle3d _tri;
    std::array<double,7> _cutoffCoefAry;
};
}
