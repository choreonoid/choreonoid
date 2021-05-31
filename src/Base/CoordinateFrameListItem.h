#ifndef CNOID_BASE_COORDINATE_FRAME_LIST_ITEM_H
#define CNOID_BASE_COORDINATE_FRAME_LIST_ITEM_H

#include "Item.h"
#include "RenderableItem.h"
#include "LocatableItem.h"
#include <cnoid/GeneralId>
#include <cnoid/Signal>
#include <cnoid/EigenTypes>
#include "exportdecl.h"

namespace cnoid {

class CoordinateFrameList;
class CoordinateFrameItem;
class CoordinateFrame;
class LocatableItem;

class CNOID_EXPORT CoordinateFrameListItem : public Item, public RenderableItem
{
public:
    static void initializeClass(ExtensionManager* ext);

    static SignalProxy<void(CoordinateFrameListItem* frameListItem, bool on)> sigListAssociationWith(Item* item);

    CoordinateFrameListItem();
    CoordinateFrameListItem(CoordinateFrameList* frameList);
    CoordinateFrameListItem(const CoordinateFrameListItem& org);
    virtual ~CoordinateFrameListItem();

    /**
       A coordinate frame list consists of coordinate frame elements and the list to hold them.
       To manage it on the Choreonoid GUI, CoordinateFrameListItem and CoordinateFrameItem are introduced.
       Basically, the former corresponds to the coordinate frame list and the latter corresponds to
       each coordinate frame element. However, how to use CoordinateFrameItems depends on ItemizationMode.
    */
    enum ItemizationMode {
        /**
           In this mode, CoordinateFrameItem is not used and a user cannot see each frame element
           in the item tree.
        */
        NoItemization,
        /**
           In this mode, each frame item is attached to the list item with the SubItem attribute,
           and the frame items are managed by the list item.
        */
        SubItemization,
        /**
           In this mode, each frame item is attached to the list item with the Attached attribute,
           and the frame items are handled as an item independent of the list item.
        */
        IndependentItemization
    };

    int itemizationMode() const;
    bool isNoItemizationMode() const;
    void setItemizationMode(int mode);
    void customizeFrameItemDisplayName(std::function<std::string(const CoordinateFrameItem* item)> func);
    std::string getFrameItemDisplayName(const CoordinateFrameItem* item) const;
    void updateFrameItems();
    CoordinateFrameItem* findFrameItemAt(int index);
    CoordinateFrameItem* findFrameItem(const GeneralId& id);

    CoordinateFrameList* frameList();
    const CoordinateFrameList* frameList() const;

    void useAsBaseFrames();
    void useAsOffsetFrames();
    bool isForBaseFrames() const;
    bool isForOffsetFrames() const;

    virtual LocationProxyPtr getFrameParentLocationProxy();
    bool getRelativeFramePosition(const CoordinateFrame* frame, Isometry3& out_T) const;
    bool getGlobalFramePosition(const CoordinateFrame* frame, Isometry3& out_T) const;
    bool switchFrameMode(CoordinateFrame* frame, int mode);

    // RenderableItem function
    virtual SgNode* getScene() override;

    void setFrameMarkerVisible(const CoordinateFrame* frame, bool on);
    ReferencedPtr transientFrameMarkerHolder(const CoordinateFrame* frame);
    bool isFrameMarkerVisible(const CoordinateFrame* frame) const;
    SignalProxy<void(int index, bool on)> sigFrameMarkerVisibilityChanged();

    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;

    class Impl;

protected:
    virtual Item* doDuplicate() const override;
    virtual void onTreePositionChanged() override;
    virtual void onDisconnectedFromRoot() override;
    virtual bool onChildItemAboutToBeAdded(Item* childItem, bool isManualOperation) override;
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;

private:
    friend class CoordinateFrameItem;
    // Called from CoordinateFrameItem::onPositionChanged
    bool onFrameItemAdded(CoordinateFrameItem* frameItem);
    void onFrameItemRemoved(CoordinateFrameItem* frameItem);
    
private:
    Impl* impl;
};

typedef ref_ptr<CoordinateFrameListItem> CoordinateFrameListItemPtr;

}

#endif
