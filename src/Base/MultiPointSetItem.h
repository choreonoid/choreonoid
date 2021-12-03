/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_MULTI_POINT_SET_ITEM_H
#define CNOID_BASE_MULTI_POINT_SET_ITEM_H

#include "PointSetItem.h"
#include <cnoid/SceneDrawables>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT MultiPointSetItem : public Item, public RenderableItem
{
public:
    static void initializeClass(ExtensionManager* ext);

    MultiPointSetItem();
    MultiPointSetItem(const MultiPointSetItem& org);
    virtual ~MultiPointSetItem();

    enum VisibilityMode {
        ShowAll, ShowSelected, NumVisibilityModes
    };
    void setVisibilityMode(int mode);
    
    enum RenderingMode {
        Point = PointSetItem::POINT,
        Voxel = PointSetItem::VOXEL,
        NumRenderingModes
    };

    void setRenderingMode(int mode);
    int renderingMode() const;
    
    void setPointSize(double size);
    double pointSize() const;

    double voxelSize() const;
    void setVoxelSize(double size);

    int numPointSetItems() const;
    PointSetItem* pointSetItem(int index);
    const PointSetItem* pointSetItem(int index) const;

    int numVisiblePointSetItems() const;
    PointSetItem* visiblePointSetItem(int index);
    const PointSetItem* visiblePointSetItem(int index) const;
    
    void selectSinglePointSetItem(int index);

    SignalProxy<void(int index)> sigPointSetItemAdded();
    SignalProxy<void(int index)> sigPointSetUpdated();

    const Isometry3& offsetPosition() const;
    void setOffsetPosition(const Isometry3& T);
    SignalProxy<void(const Isometry3& T)> sigOffsetPositionChanged();
    void notifyOffsetPositionChange();

    Isometry3 totalOffsetPositionOf(int index) const;
    SgPointSetPtr getTransformedPointSet(int index) const;

    [[deprecated("Use offsetPosition.")]]
    const Isometry3& topOffsetTransform() const { return offsetPosition(); }
    [[deprecated("Use setOffsetPosition.")]]
    void setTopOffsetTransform(const Isometry3& T) { return setOffsetPosition(T); }
    [[deprecated("Use sigOffsetPositionChanged.")]]
    SignalProxy<void(const Isometry3& T)> sigTopOffsetTransformChanged() { return sigOffsetPositionChanged(); }
    [[deprecated("Use notifyOffsetPositionChange.")]]
    void notifyTopOffsetTransformChange() { notifyOffsetPositionChange(); }
    [[deprecated("Use totalOffsetPositionOf.")]]
    Isometry3 offsetTransform(int index) const { return totalOffsetPositionOf(index); }
    
    int numAttentionPoints() const;
    Vector3 attentionPoint(int index) const;
    void clearAttentionPoints();
    void addAttentionPoint(const Vector3& p);
    SignalProxy<void()> sigAttentionPointsChanged();
    void notifyAttentionPointChange();

    // RenderableItem
    virtual SgNode* getScene() override;
    
    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;

    bool startAutomaticSave(const std::string& filename);
    void stopAutomaticSave();

    // deprecated. Use numVisiblePointSetItems();
    int numActivePointSetItems() const;
    // deprecated. Use visiblePointSetItem(int index);
    PointSetItem* activePointSetItem(int index);
    const PointSetItem* activePointSetItem(int index) const;

    class Impl;

protected:
    virtual Item* doDuplicate() const override;
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;

private:
    Impl* impl;
    void initialize();
};

typedef ref_ptr<MultiPointSetItem> MultiPointSetItemPtr;
}
    
#endif
