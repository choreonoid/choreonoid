/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_PCL_PLUGIN_MULTI_POINT_SET_ITEM_H
#define CNOID_PCL_PLUGIN_MULTI_POINT_SET_ITEM_H

#include <cnoid/PointSetItem>
#include "exportdecl.h"

namespace cnoid {

class MultiPointSetItemImpl;

class CNOID_EXPORT MultiPointSetItem : public Item, public SceneProvider
{
public:
    static void initializeClass(ExtensionManager* ext);

    MultiPointSetItem();
    MultiPointSetItem(const MultiPointSetItem& org);
    virtual ~MultiPointSetItem();

    void setPointSize(double size);
    double pointSize() const;

    double voxelSize() const;
    void setVoxelSize(double size);

    virtual SgNode* getScene();
    
    virtual bool store(Archive& archive);
    virtual bool restore(const Archive& archive);

protected:
    virtual ItemPtr doDuplicate() const;
    virtual void doPutProperties(PutPropertyFunction& putProperty);

private:
    MultiPointSetItemImpl* impl;
    void initialize();
};

typedef ref_ptr<MultiPointSetItem> MultiPointSetItemPtr;
}
    
#endif
