#ifndef CNOID_BODY_PLUGIN_REGION_INTRUSION_DETECTOR_ITEM_H
#define CNOID_BODY_PLUGIN_REGION_INTRUSION_DETECTOR_ITEM_H

#include "ControllerItem.h"
#include <cnoid/LocatableItem>
#include <cnoid/RenderableItem>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT RegionIntrusionDetectorItem : public ControllerItem, public LocatableItem, public RenderableItem
{
public:
    static void initializeClass(ExtensionManager* ext);
    
    RegionIntrusionDetectorItem();
    RegionIntrusionDetectorItem(const RegionIntrusionDetectorItem& org);
    ~RegionIntrusionDetectorItem();

    bool setBoxRegionSize(const Vector3& size);
    const Vector3& boxRegionSize() const;
    void setRegionOffset(const Isometry3& T);
    const Isometry3& regionOffset() const;

    void setDigitalIoSignalNumber(int no);
    int digitalIoSignalNumber() const;

    virtual bool initialize(ControllerIO* io) override;
    virtual void input() override;
    virtual bool control() override;
    virtual void output() override;
    virtual void stop() override;

    // LocatableItem function
    virtual LocationProxyPtr getLocationProxy() override;

    // RenderableItem function. This returns the coordinate marker.
    virtual SgNode* getScene() override;

    class Impl;

protected:
    virtual Item* doDuplicate() const override;
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;
    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;

private:
    Impl* impl;
};

}

#endif
