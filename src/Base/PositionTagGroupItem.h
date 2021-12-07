#ifndef CNOID_BASE_POSITION_TAG_GROUP_ITEM_H
#define CNOID_BASE_POSITION_TAG_GROUP_ITEM_H

#include "Item.h"
#include "RenderableItem.h"
#include "LocatableItem.h"
#include <vector>
#include "exportdecl.h"

namespace cnoid {

class PositionTagGroup;

class CNOID_EXPORT PositionTagGroupItem : public Item, public RenderableItem, public LocatableItem
{
public:
    static void initializeClass(ExtensionManager* ext);
    static PositionTagGroupItem* findItemOf(PositionTagGroup* tagGroup);

    PositionTagGroupItem();
    PositionTagGroupItem(const PositionTagGroupItem& org);
    virtual ~PositionTagGroupItem();

    virtual bool setName(const std::string& name) override;

    const PositionTagGroup* tagGroup() const;
    PositionTagGroup* tagGroup();

    const Isometry3& parentFramePosition() const;
    const Isometry3& originOffset() const;
    void setOriginOffset(const Isometry3& T_offset, bool requestPreview = true);
    Isometry3 originPosition() const;
    SignalProxy<void()> sigOriginOffsetPreviewRequested();
    SignalProxy<void()> sigOriginOffsetUpdated();
    void notifyOriginOffsetUpdate(bool requestPreview = true);

    void clearTagSelection();
    void setTagSelected(int tagIndex, bool on = true);
    bool checkTagSelected(int tagIndex) const;
    const std::vector<int>& selectedTagIndices() const;
    void setSelectedTagIndices(const std::vector<int>& indices);
    SignalProxy<void()> sigTagSelectionChanged();

    // RenderableItem function
    virtual SgNode* getScene() override;

    // LocatableItem functions
    virtual std::vector<LocationProxyPtr> getLocationProxies() override;
    virtual SignalProxy<void()> getSigLocationProxiesChanged() override;
        
    double tagMarkerSize() const;
    void setTagMarkerSize(double s);

    bool originMarkerVisibility() const;
    void setOriginMarkerVisibility(bool on);

    bool edgeVisibility() const;
    void setEdgeVisiblility(bool on);

    float transparency() const;
    void setTransparency(float t);

    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;

    class Impl;

protected:
    virtual Item* doDuplicate() const override;
    virtual void onConnectedToRoot() override;
    virtual void onDisconnectedFromRoot() override;
    virtual bool onNewTreePositionCheck(
        bool isManualOperation, std::function<void()>& out_callbackWhenAdded) override;
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;

private:
    Impl* impl;
};

typedef ref_ptr<PositionTagGroupItem> PositionTagGroupItemPtr;

}

#endif
