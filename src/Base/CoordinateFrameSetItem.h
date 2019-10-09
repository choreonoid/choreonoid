#ifndef CNOID_BODY_PLUGIN_COORDINATE_FRAME_SET_ITEM_H
#define CNOID_BODY_PLUGIN_COORDINATE_FRAME_SET_ITEM_H

#include <cnoid/Item>
#include <cnoid/SceneProvider>
#include "exportdecl.h"

namespace cnoid {

class CoordinateFrameContainer;
class CoordinateFrameSetPair;

class CNOID_EXPORT CoordinateFrameSetItem : public Item, public SceneProvider
{
public:
    static void initializeClass(ExtensionManager* ext);

    CoordinateFrameSetItem();
    CoordinateFrameSetItem(const CoordinateFrameSetItem& org);
    virtual ~CoordinateFrameSetItem();

    CoordinateFrameContainer* frames();
    const CoordinateFrameContainer* frames() const;

    virtual SgNode* getScene() override;    

    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;

protected:
    virtual Item* doDuplicate() const override;
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;

private:
    class Impl;
    Impl* impl;
};

typedef ref_ptr<CoordinateFrameSetItem> CoordinateFrameSetItemPtr;

}

#endif
