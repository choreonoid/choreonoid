/**
   @author Japan Atomic Energy Agency
*/

#pragma once

#include "FFCalc_Common.h"
#include "FFCalc_GaussQuadratureTriangle.h"
#include "Triangle3.h"

namespace Multicopter {
namespace FFCalc {

class GaussTriangle3d : public Triangle3d
{
public:

private:
    double  _area;
    Vector3 _normal;

public:

    GaussTriangle3d (const Vector3& p0, const Vector3& p1, const Vector3& p2, const Transform3& trans)
        : Triangle3d (trans*p0, trans*p1, trans*p2)
    {
        Vector3 cross=(_pos[1]-_pos[0]).cross(_pos[2]-_pos[0]);
        _area   = 0.5 * cross.norm();
        _normal = cross / (2.0*_area);
    }

    GaussTriangle3d (const Triangle3d& tri, const Transform3& trans)
        : GaussTriangle3d (tri[0], tri[1], tri[2], trans) { }

    GaussTriangle3d (const Vector3& p0, const Vector3& p1, const Vector3& p2)
        : Triangle3d (p0, p1, p2)
    {
        Vector3 cross = (p1-p0).cross(p2-p0);
        _area   = 0.5 * cross.norm();
        _normal = cross / (2.0*_area);
    }

    GaussTriangle3d (const Triangle3d& tri)
        : GaussTriangle3d (tri[0], tri[1], tri[2]) { }

    const Vector3& normal() const
    {
        return _normal;
    }

    double area() const
    {
        return _area;
    }

    Vector3 getGaussPoint (const int idx,const int num) const {
        if(num==1) {
            return FFCalc::GaussQuadratureTriangle::param1[idx].getPosition((*this)[0], (*this)[1], (*this)[2]);
        }else if(num==4){
            return FFCalc::GaussQuadratureTriangle::param4[idx].getPosition((*this)[0], (*this)[1], (*this)[2]);
        }else{
            return FFCalc::GaussQuadratureTriangle::param4[idx].getPosition((*this)[0], (*this)[1], (*this)[2]);
        }
    }

    double getGaussWeight (const int idx,const int num) const {
        if(num==1) return FFCalc::GaussQuadratureTriangle::param1[idx].weight;
        else if(num==4) return FFCalc::GaussQuadratureTriangle::param4[idx].weight;
        else return FFCalc::GaussQuadratureTriangle::param4[idx].weight;
    }
};



}}
