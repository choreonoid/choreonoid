/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_POLYHEDRAL_REGION_H
#define CNOID_UTIL_POLYHEDRAL_REGION_H

#include "EigenTypes.h"
#include <vector>

namespace cnoid {

class PolyhedralRegion
{
public:
    PolyhedralRegion() { }
    PolyhedralRegion(const PolyhedralRegion& org) : planes(org.planes) { }
    PolyhedralRegion& operator=(const PolyhedralRegion& org) { planes = org.planes; return *this; }

    struct Plane {
        Vector3 normal;
        Vector3 point;
        double d;
        Plane(const Vector3& normal, const Vector3& point) : normal(normal), point(point) {
            d = normal.dot(point);
        }
    };

    int numBoundingPlanes() const { return planes.size(); }

    void clear() { planes.clear(); }
    
    void addBoundingPlane(const Vector3& normal, const Vector3& point){
        planes.push_back(Plane(normal, point));
    }

    const Plane& plane(int index) const { return planes[index]; }

    bool checkInside(const Vector3& point) const {
        for(size_t i=0; i < planes.size(); ++i){
            const Plane& p = planes[i];
            if(point.dot(p.normal) - p.d < 0.0){
                return false;
            }
        }
        return true;
    }

private:
    std::vector<Plane> planes;
};

}

#endif
