#ifndef CNOID_BASE_GEOMETRY_MEASUREMENT_TRACKER_H
#define CNOID_BASE_GEOMETRY_MEASUREMENT_TRACKER_H

#include "Widget.h"
#include <cnoid/Referenced>
#include <cnoid/EigenTypes>
#include <cnoid/SceneGraph>
#include <cnoid/Signal>
#include <functional>
#include "exportdecl.h"

namespace cnoid {

class Item;

class CNOID_EXPORT GeometryMeasurementTracker : public Referenced
{
public:
    static void registerItemTrackerFactory(
        std::function<GeometryMeasurementTracker*(Item* item)> trackerFactory);
    static bool checkIfMeasureable(Item* item);
    static GeometryMeasurementTracker* createTracker(Item* item);
    
    virtual void resetMeasurementPoint() = 0;
    virtual Vector3 getMeasurementPoint() = 0;
    virtual bool setMeasurementPoint(const SgNodePath& path, const Vector3& point) = 0;
    virtual bool checkTrackable(const SgNodePath& path) = 0;

    // Optional functions to provide the sub-entries of the target item
    virtual int getNumSubEntries();
    virtual std::string getSubEntryName(int index);
    //! \return -1 if sub entry is not found
    virtual int findSubEntryIndex(const std::string& name);
    virtual int getCurrentSubEntryIndex();
    virtual bool setCurrentSubEntry(int index);

    // For shortest distance measurement
    virtual int getNumShapes() = 0;
    virtual SgNode* getShape(int index) = 0;
    virtual Isometry3 getShapePosition(int index) = 0;

    virtual SignalProxy<void()> sigGeometryChanged() = 0;
};

typedef ref_ptr<GeometryMeasurementTracker> GeometryMeasurementTrackerPtr;
    
}

#endif
