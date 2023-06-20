#ifndef CNOID_BODY_PLUGIN_BODY_GEOMETRY_MEASUREMENT_TRACKER_H
#define CNOID_BODY_PLUGIN_BODY_GEOMETRY_MEASUREMENT_TRACKER_H

#include "BodyItem.h"
#include <cnoid/GeometryMeasurementTracker>
#include <cnoid/CoordinateFrame>
#include <cnoid/ConnectionSet>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT BodyGeometryMeasurementTracker : public GeometryMeasurementTracker
{
public:
    static void initializeClass();

    BodyGeometryMeasurementTracker(BodyItem* bodyItem);
    virtual void resetMeasurementPoint() override;
    virtual Vector3 getMeasurementPoint() override;
    virtual bool setMeasurementPoint(const SgNodePath& path, const Vector3& localPoint) override;
    virtual bool checkTrackable(const SgNodePath& path) override;
    virtual int getNumSubEntries() override;
    virtual std::string getSubEntryName(int index) override;
    virtual int findSubEntryIndex(const std::string& name) override;
    virtual int getCurrentSubEntryIndex() override;
    virtual bool setCurrentSubEntry(int index) override;
    virtual int getNumShapes() override;
    virtual SgNode* getShape(int index) override;
    virtual Isometry3 getShapePosition(int index) override;
    virtual SignalProxy<void()> sigGeometryChanged() override;

private:
    void resetMeasurementPoint_(Link* link);
    void setMeasurementPoint_(Link* link, const Vector3& localPosition);
    int findSubEntryForSceneNode(const SgNodePath& path);
    void updateSubEntries();
    
    BodyItemPtr bodyItem;
    BodyPtr body;
    Link* linkOfPoint;
    Vector3 localPosition;

    struct SubEntry {
        std::string name;
        LinkPtr link;
        CoordinateFramePtr offsetFrame;
        SubEntry(const std::string& name, Link* link, CoordinateFrame* offsetFrame)
            : name(name), link(link), offsetFrame(offsetFrame) { }
    };
    std::vector<SubEntry> subEntries;
    ScopedConnectionSet subEntryConnections;
    int currentSubEntryIndex;
    bool needToUpdateSubEntries;
    
    Signal<void()> sigGeometryChanged_;
    ScopedConnectionSet connections;
};

}

#endif
