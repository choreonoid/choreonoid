/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_PLUGIN_BODY_TRACKING_CAMERA_ITEM_H
#define CNOID_BODY_PLUGIN_BODY_TRACKING_CAMERA_ITEM_H

#include <cnoid/Item>
#include <cnoid/SceneProvider>
#include "exportdecl.h"

namespace cnoid {

class BodyTrackingCameraItemImpl;
  
class CNOID_EXPORT BodyTrackingCameraItem : public Item, public SceneProvider
{
public:
    static void initializeClass(ExtensionManager* ext);
    
    enum CameraType { PERSPECTIVE = 0, ORTHOGRAPHIC, N_CAMERA_TYPE  };

    BodyTrackingCameraItem();
    BodyTrackingCameraItem(const BodyTrackingCameraItem& org);
    ~BodyTrackingCameraItem();

    virtual void setName(const std::string& name);
    virtual SgNode* getScene();

protected:
    virtual Item* doDuplicate() const override;
    virtual void onPositionChanged() override;
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;
    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;
    
private:
    BodyTrackingCameraItemImpl* impl;
};
  
typedef ref_ptr<BodyTrackingCameraItem> BodyTrackingCameraItemPtr;

}

#endif
