/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_ROOT_ITEM_H_INCLUDED
#define CNOID_BASE_ROOT_ITEM_H_INCLUDED

#include "Item.h"
#include "ItemList.h"
#include "exportdecl.h"

namespace cnoid {

class Project;
class Item;
class RootItemImpl;

/**
   @if jp
   アイテムの木構造のルートとなるクラス。
   @endif
*/
class CNOID_EXPORT RootItem : public Item
{
public:
    static void initializeClass(ExtensionManager* ext);
    static RootItem* instance();
    static RootItem* mainInstance(); // obsolete

    RootItem();
    RootItem(const RootItem& org);
    virtual ~RootItem();

    SignalProxy< boost::signal<void(RootItem* rootItem)> > sigDestroyed();
    SignalProxy< boost::signal<void(Item* item)> > sigSubTreeAdded();
    SignalProxy< boost::signal<void(Item* item)> > sigItemAdded();
    SignalProxy< boost::signal<void(Item* item)> > sigSubTreeMoved();
    SignalProxy< boost::signal<void(Item* item)> > sigItemMoved();
    SignalProxy< boost::signal<void(Item* item, bool isMoving)> > sigSubTreeRemoving();
    SignalProxy< boost::signal<void(Item* item, bool isMoving)> > sigSubTreeRemoved();
    SignalProxy< boost::signal<void()> > sigTreeChanged();

    SignalProxy< boost::signal<void(Item* assigned, Item* srcItem)> > sigItemAssigned();

protected:

    virtual ItemPtr doDuplicate() const;
    virtual bool store(Archive& archive);
    virtual bool restore(const Archive& archive);

private:

    void initializeInstance();

    friend class Item;

    void notifyEventOnSubTreeAdded(Item* item);
    void notifyEventOnSubTreeMoved(Item* item);
    void notifyEventOnSubTreeRemoving(Item* item, bool isMoving);
    void notifyEventOnSubTreeRemoved(Item* item, bool isMoving);

    void emitSigItemAssinged(Item* assigned, Item* srcItem);

    friend class RootItemImpl;
    RootItemImpl* impl;
};

typedef ref_ptr<RootItem> RootItemPtr;

}

#endif
