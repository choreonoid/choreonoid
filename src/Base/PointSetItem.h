/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_POINT_SET_ITEM_H
#define CNOID_BASE_POINT_SET_ITEM_H

#include "Item.h"
#include "RenderableItem.h"
#include <cnoid/EigenTypes>
#include <cnoid/stdx/optional>
#include "exportdecl.h"

namespace cnoid {

class SgPointSet;
class PolyhedralRegion;

class CNOID_EXPORT PointSetItem : public Item, public RenderableItem
{
public:
    static void initializeClass(ExtensionManager* ext);

    PointSetItem();
    PointSetItem(const PointSetItem& org);
    virtual ~PointSetItem();

    virtual bool setName(const std::string& name) override;

    const SgPointSet* pointSet() const;
    SgPointSet* pointSet();

    virtual void notifyUpdate() override;

    virtual SgNode* getScene() override;

    const Isometry3& offsetPosition() const;
    void setOffsetPosition(const Isometry3& T);
    SignalProxy<void(const Isometry3& T)> sigOffsetPositionChanged();
    void notifyOffsetPositionChange(bool doNotifyScene = true);

    SgPointSet* getTransformedPointSet() const;

    [[deprecated("Use offsetPosition.")]]
    const Isometry3& offsetTransform() const { return offsetPosition(); }
    [[deprecated("Use setOffsetPosition.")]]
    void setOffsetTransform(const Isometry3& T) { setOffsetPosition(T); }
    [[deprecated("Use sigOffsetPositionChanged.")]]
     SignalProxy<void(const Isometry3& T)> sigOffsetTransformChanged() { return sigOffsetPositionChanged(); }
    [[deprecated("Use notifyOffsetPositionChange.")]]
    void notifyOffsetTransformChange() { notifyOffsetPositionChange(); }

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

    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;

    class Impl;

protected:
    virtual Item* doDuplicate() const override;
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;

private:
    Impl* impl;
    void initialize();
};

typedef ref_ptr<PointSetItem> PointSetItemPtr;
}
    
#endif
