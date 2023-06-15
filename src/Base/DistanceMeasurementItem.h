#ifndef CNOID_BASE_DISTANCE_MEASUREMENT_ITEM_H
#define CNOID_BASE_DISTANCE_MEASUREMENT_ITEM_H

#include "Item.h"
#include "RenderableItem.h"
#include <cnoid/EigenTypes>
#include <cnoid/Signal>
#include "exportdecl.h"

namespace cnoid {

class GeometryMeasurementTracker;

class CNOID_EXPORT DistanceMeasurementItem : public Item, public RenderableItem
{
public:
    static void initializeClass(ExtensionManager* ext);

    DistanceMeasurementItem();
    virtual ~DistanceMeasurementItem();

    bool isMeasurementActive() const;

    /**
       \param tracker If nullptr is given, the target tracker is generated internally.
    */
    void setTargetItem(int which, Item* item, GeometryMeasurementTracker* tracker = nullptr);

    Item* targetItem(int which);
    GeometryMeasurementTracker* targetTracker(int which);
    bool hasValidTargetItems() const;

    void setShortestDistanceMode(bool on);
    bool isShortestDistanceMode() const;
    void setShortestDistanceFixOneSideMode(bool on);
    bool isShortestDistanceFixOneSideMode() const;
    void setShortestDistanceFixedSide(int which);
    int shortestDistanceFixedSide() const;
    SignalProxy<void()> sigMeasurementConfigurationChanged();
    void notifyMeasurmentConfigurationChange();
    
    bool startMeasurement();
    void stopMeasurement();

    const Vector3& measurementPoint(int which) const;
    double distance() const;
    
    SignalProxy<void(bool isValid)> sigDistanceUpdated();

    // RenderableItem
    virtual SgNode* getScene() override;

    void setDistanceLineOverlayEnabled(bool on);
    bool isDistanceLineOverlayEnabled() const;
    void setDistanceMarkerColor(const Vector3f& color);
    const Vector3f& distanceMarkerColor() const;
    void setDistanceLineWidth(int width);
    int distanceLineWidth() const;

protected:
    DistanceMeasurementItem(const DistanceMeasurementItem& org);
    virtual Item* doCloneItem(CloneMap* cloneMap) const override;
    virtual void onDisconnectedFromRoot() override;
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;
    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;

private:
    class Impl;
    Impl* impl;
};

typedef ref_ptr<DistanceMeasurementItem> DistanceMeasurementItemPtr;

}

#endif
