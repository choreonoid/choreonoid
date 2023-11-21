#ifndef CNOID_BASE_POINT_SET_ITEM_H
#define CNOID_BASE_POINT_SET_ITEM_H

#include "Item.h"
#include "RenderableItem.h"
#include "LocatableItem.h"
#include <cnoid/EigenTypes>
#include <cnoid/stdx/optional>
#include "exportdecl.h"

namespace cnoid {

class SgPointSet;
class PolyhedralRegion;

class CNOID_EXPORT PointSetItem : public Item, public RenderableItem, public LocatableItem
{
public:
    static void initializeClass(ExtensionManager* ext);

    PointSetItem();
    virtual ~PointSetItem();

    virtual bool setName(const std::string& name) override;

    const SgPointSet* pointSet() const;
    SgPointSet* pointSet();

    virtual void notifyUpdate() override;
    virtual SgNode* getScene() override;
    virtual LocationProxyPtr getLocationProxy() override;

    const Isometry3& offsetPosition() const;
    void setOffsetPosition(const Isometry3& T);
    Vector3 offsetTranslation() const;
    void setOffsetTranslation(const Vector3& p);
    SignalProxy<void()> sigOffsetPositionChanged();
    void notifyOffsetPositionChange(bool doNotifyScene = true);

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

    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;

    class Impl;

protected:
    PointSetItem(const PointSetItem& org, CloneMap* cloneMap);
    virtual Item* doCloneItem(CloneMap* cloneMap) const override;
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;

private:
    Impl* impl;
    void initialize();
};

typedef ref_ptr<PointSetItem> PointSetItemPtr;
}
    
#endif
