/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_POINT_SET_ITEM_H
#define CNOID_BASE_POINT_SET_ITEM_H

#include <cnoid/Item>
#include <cnoid/SceneShape>
#include <cnoid/SceneProvider>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT PointSetItem : public Item, public SceneProvider
{
public:
    static void initializeClass(ExtensionManager* ext);

    PointSetItem();
    PointSetItem(const PointSetItem& org);
    virtual ~PointSetItem();

    virtual void setName(const std::string& name);
    virtual SgNode* scene();

    const SgPointSet* pointSet() const { return pointSet_; }
    SgPointSet* pointSet() { return pointSet_; }

    Affine3& offsetPosition() { return topTransform->T(); }
    const Affine3& offsetPosition() const { return topTransform->T(); }

    void setPointSize(double size);

    virtual void notifyUpdate();
        
protected:
    virtual ItemPtr doDuplicate() const;
    virtual bool store(Archive& archive);
    virtual bool restore(const Archive& archive);
    virtual void doPutProperties(PutPropertyFunction& putProperty);
    void updateVisiblePointSet();

private:
    SgPointSetPtr pointSet_;
    SgPointSetPtr visiblePointSet;
    SgPosTransformPtr topTransform;
    SgInvariantGroupPtr invariant;

    void initMembers();
};

typedef ref_ptr<PointSetItem> PointSetItemPtr;
}
    
#endif
