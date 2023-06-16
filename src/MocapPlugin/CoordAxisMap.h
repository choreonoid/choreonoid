#ifndef CNOID_MOCAP_PLUGIN_COORD_AXIS_MAP_H
#define CNOID_MOCAP_PLUGIN_COORD_AXIS_MAP_H

#include <cnoid/EigenTypes>

namespace cnoid {

class CoordAxisMap
{
public:
    enum Axis { X, Y, Z };

    // Create the map with no conversion
    CoordAxisMap() : CoordAxisMap(X, Y, Z) { }
    
    CoordAxisMap(int sourceAxisForX, int sourceAxisForY, int sourceAxisForZ){
        axisForSource_[sourceAxisForX] = X;
        axisForSource_[sourceAxisForY] = Y;
        axisForSource_[sourceAxisForZ] = Z;
        sourceAxis_[X] = sourceAxisForX;
        sourceAxis_[Y] = sourceAxisForY;
        sourceAxis_[Z] = sourceAxisForZ;
    }
        
    CoordAxisMap(const CoordAxisMap& org) = default;
    CoordAxisMap& operator=(const CoordAxisMap& rhs) = default;

    template<class AxisIndex>
    AxisIndex axisForSource(AxisIndex sourceAxis){
        return static_cast<AxisIndex>(axisForSource_[sourceAxis]);
    }
    template<class AxisIndex>
    AxisIndex axisForSource(AxisIndex sourceAxis, AxisIndex offset){
        return static_cast<AxisIndex>(axisForSource_[sourceAxis - offset] + offset);
    }
    template<class AxisIndex>
    AxisIndex sourceAxis(AxisIndex axis){
        return static_cast<AxisIndex>(sourceAxis_[axis]);
    }
    template<class AxisIndex>
    AxisIndex sourceAxis(AxisIndex axis, AxisIndex offset){
        return static_cast<AxisIndex>(sourceAxis_[axis - offset] + offset);
    }
    Vector3 getVectorForSource(const Vector3& v){
        return Vector3(v[sourceAxis(X)], v[sourceAxis(Y)], v[sourceAxis(Z)]);
    }
    Vector3 getSourceVector(const Vector3& v){
        return Vector3(v[axisForSource(X)], v[axisForSource(Y)], v[axisForSource(Z)]);
    }

private:
    int axisForSource_[3];
    int sourceAxis_[3];
};

}

#endif
