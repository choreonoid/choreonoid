/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_POINT_SET_ITEM_H
#define CNOID_BASE_POINT_SET_ITEM_H

#include <cnoid/Item>
#include <cnoid/RectRegionMarker>
#include <cnoid/SceneProvider>
#include <cnoid/stdx/optional>
#include "exportdecl.h"

namespace cnoid {

class SgPointSet;
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
        
    const Affine3& offsetTransform() const;
    void setOffsetTransform(const Affine3& T);
    SignalProxy<void(const Affine3& T)> sigOffsetTransformChanged();
    void notifyOffsetTransformChange();

    SgPointSet* getTransformedPointSet() const;

    enum RenderingMode {
        POINT, VOXEL, N_RENDERING_MODES
    };

    void setRenderingMode(int mode);
    int renderingMode() const;

    void setPointSize(double size);
    double pointSize() const;

    static double defaultVoxelSize();
    double voxelSize() const;
    void setVoxelSize(double size);
    
    void setEditable(bool on);
    bool isEditable() const;

    int numAttentionPoints() const;
    Vector3 attentionPoint(int index) const;
    void clearAttentionPoints();
    void addAttentionPoint(const Vector3& p);
    SignalProxy<void()> sigAttentionPointsChanged();
    void notifyAttentionPointChange();
    
    stdx::optional<Vector3> attentionPoint() const; // deprecated
    SignalProxy<void()> sigAttentionPointChanged();  // deprecated
    void clearAttentionPoint();  // deprecated
    void setAttentionPoint(const Vector3& p);  // deprecated

    void removePoints(const PolyhedralRegion& region);

    SignalProxy<void(const PolyhedralRegion& region)> sigPointsInRegionRemoved();

    virtual bool store(Archive& archive);
    virtual bool restore(const Archive& archive);

protected:
    virtual Item* doDuplicate() const;
    virtual void doPutProperties(PutPropertyFunction& putProperty);

private:
    PointSetItemImpl* impl;
    void initialize();
};

typedef ref_ptr<PointSetItem> PointSetItemPtr;
}
    
#endif
