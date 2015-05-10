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

    void setRenderingMode(int mode);
    int renderingMode() const;
    
    void setPointSize(double size);
    double pointSize() const;

    double voxelSize() const;
    void setVoxelSize(double size);

    int numPointSetItems() const;
    PointSetItem* pointSetItem(int index);

    void selectSinglePointSetItem(int index);

    SignalProxy<void(int index)> sigPointSetItemAdded();
    SignalProxy<void(int index)> sigPointSetUpdated();
    
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
