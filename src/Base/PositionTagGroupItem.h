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

    PositionTagGroupItem();
    PositionTagGroupItem(const PositionTagGroupItem& org);
    virtual ~PositionTagGroupItem();

    virtual bool setName(const std::string& name) override;

    const PositionTagGroup* tagGroup() const;
    PositionTagGroup* tagGroup();

    const Position& parentFramePosition() const;
    const Position& originOffset() const;
    void setOriginOffset(const Position& T_offset);
    Position originPosition() const;
    
    void clearTagSelection();
    void setTagSelected(int tagIndex, bool on = true);
    bool checkTagSelected(int tagIndex) const;
    const std::vector<int>& selectedTagIndices() const;
    void setSelectedTagIndices(const std::vector<int>& indices);
    SignalProxy<void()> sigTagSelectionChanged();

    // RenderableItem function
    virtual SgNode* getScene() override;

    // LocatableItem function
    virtual LocationProxyPtr getLocationProxy() override;

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
    virtual bool onNewPositionCheck(bool isManualOperation, std::function<void()>& out_callbackWhenAdded) override;
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;

private:
    Impl* impl;
};

typedef ref_ptr<PositionTagGroupItem> PositionTagGroupItemPtr;

}

#endif
