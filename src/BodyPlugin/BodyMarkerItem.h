/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_PLUGIN_BODY_MARKER_ITEM_H
#define CNOID_BODY_PLUGIN_BODY_MARKER_ITEM_H

#include <cnoid/Item>
#include <cnoid/SceneProvider>
#include "exportdecl.h"

namespace cnoid {

class BodyMarkerItemImpl;

class CNOID_EXPORT BodyMarkerItem : public Item, public SceneProvider
{
public:
    static void initializeClass(ExtensionManager* ext);
        
    BodyMarkerItem();
    BodyMarkerItem(const BodyMarkerItem& org);
    virtual ~BodyMarkerItem();

    virtual void setName(const std::string& name);
    virtual SgNode* getScene();

protected:
    virtual Item* doDuplicate() const;
    virtual void onPositionChanged();
    virtual void doPutProperties(PutPropertyFunction& putProperty);
    virtual bool store(Archive& archive);
    virtual bool restore(const Archive& archive);
            
private:
    BodyMarkerItemImpl* impl;
};

}

#endif
