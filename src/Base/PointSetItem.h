/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_POINT_SET_ITEM_H
#define CNOID_BASE_POINT_SET_ITEM_H

#include <cnoid/Item>
#include <cnoid/SceneShape>
#include <cnoid/SceneProvider>
#include <boost/optional.hpp>
#include "exportdecl.h"

namespace cnoid {

class PointSetItemImpl;

class CNOID_EXPORT PointSetItem : public Item, public SceneProvider
{
public:
    static void initializeClass(ExtensionManager* ext);

    PointSetItem();
    PointSetItem(const PointSetItem& org);
    virtual ~PointSetItem();

    virtual void setName(const std::string& name);
    virtual SgNode* getScene();

    const SgPointSet* pointSet() const;
    SgPointSet* pointSet();

    virtual void notifyUpdate();
        
    Affine3& offsetPosition();
    const Affine3& offsetPosition() const;

    void setPointSize(double size);
    double pointSize() const;

    double voxelSize() const;
    void setVoxelSize(double size);
    
    void setEditable(bool on);
    bool isEditable() const;

    boost::optional<Vector3> attentionPoint() const;
    SignalProxy<void()> sigAttentionPointChanged();
    void clearAttentionPoint();
    void setAttentionPoint(const Vector3& p);

    void removePointsSurroundedByPlanes(const Vector3 n[], const Vector3 p[], size_t numPlanes);

    virtual bool store(Archive& archive);
    virtual bool restore(const Archive& archive);

protected:
    virtual ItemPtr doDuplicate() const;
    virtual void doPutProperties(PutPropertyFunction& putProperty);

private:
    PointSetItemImpl* impl;
    void initialize();
};

typedef ref_ptr<PointSetItem> PointSetItemPtr;
}
    
#endif
