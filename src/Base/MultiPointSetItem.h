/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_MULTI_POINT_SET_ITEM_H
#define CNOID_BASE_MULTI_POINT_SET_ITEM_H

#include <cnoid/PointSetItem>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT MultiPointSetItem : public Item, public SceneProvider
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

    const Affine3& topOffsetTransform() const;
    void setTopOffsetTransform(const Affine3& T);
    SignalProxy<void(const Affine3& T)> sigTopOffsetTransformChanged();
    void notifyTopOffsetTransformChange();

    Affine3 offsetTransform(int index) const;
    SgPointSetPtr getTransformedPointSet(int index) const;
    
    int numAttentionPoints() const;
    Vector3 attentionPoint(int index) const;
    void clearAttentionPoints();
    void addAttentionPoint(const Vector3& p);
    SignalProxy<void()> sigAttentionPointsChanged();
    void notifyAttentionPointChange();
    
    virtual SgNode* getScene();
    
    virtual bool store(Archive& archive);
    virtual bool restore(const Archive& archive);

    bool startAutomaticSave(const std::string& filename);
    void stopAutomaticSave();

    // deprecated. Use numVisiblePointSetItems();
    int numActivePointSetItems() const;
    // deprecated. Use visiblePointSetItem(int index);
    PointSetItem* activePointSetItem(int index);
    const PointSetItem* activePointSetItem(int index) const;

    class Impl;

protected:
    virtual Item* doDuplicate() const;
    virtual void doPutProperties(PutPropertyFunction& putProperty);

private:
    Impl* impl;
    void initialize();
};

typedef ref_ptr<MultiPointSetItem> MultiPointSetItemPtr;
}
    
#endif
