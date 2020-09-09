#ifndef CNOID_UTIL_WAYPOINT_ARRAY_H
#define CNOID_UTIL_WAYPOINT_ARRAY_H

#include "Referenced.h"
#include "EigenTypes.h"
#include "Signal.h"
#include <vector>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT Waypoint : public Referenced
{
public:
    Waypoint() { }
    Waypoint(const Vector3& p) : coord_(p) { }
    const Vector3& coord() { return coord_; }
    void setCoord(const Vector3& p) { coord_ = p; }

private:
    Vector3 coord_;
};

typedef ref_ptr<Waypoint> WaypointPtr;

class CNOID_EXPORT WaypointArray : public Referenced
{
public:
    WaypointArray();
    WaypointArray(const WaypointArray& org);
    virtual ~WaypointArray();

    int numPoints() const { return waypoints_.size(); }

    const Waypoint* pointAt(int index) const {
        return waypoints_[index];
    }
    Waypoint* pointAt(int index) {
        return waypoints_[index];
    }

    void insert(int index, Waypoint* point);
    void append(Waypoint* point);
    bool removeAt(int index);
    
    SignalProxy<void(int index)> sigPointAdded() { return sigPointAdded_; }
    SignalProxy<void(int index, Waypoint* point)> sigPointRemoved() { return sigPointRemoved_; }
    SignalProxy<void(int index)> sigPointUpdated() { return sigPointUpdated_; }

private:
    std::vector<WaypointPtr> waypoints_;
    Signal<void(int index)> sigPointAdded_;
    Signal<void(int index, Waypoint* point)> sigPointRemoved_;
    Signal<void(int index)> sigPointUpdated_;
};

typedef ref_ptr<WaypointArray> WaypointArrayPtr;

}

#endif
