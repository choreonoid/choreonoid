/**
   @author Shin'ichiro Nakaoka
*/

#include "RootItem.h"
#include "ExtensionManager.h"
#include "ItemManager.h"
#include "LazySignal.h"
#include <iostream>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {
const bool TRACE_FUNCTIONS = false;
}

namespace cnoid {

class RootItemImpl
{
public:
    RootItem* self;
	
    RootItemImpl(RootItem* self);
    ~RootItemImpl();

    Signal<void(RootItem* rootItem)> sigDestroyed;
    Signal<void(Item* item)> sigSubTreeAdded;
    Signal<void(Item* item)> sigItemAdded;
    Signal<void(Item* item)> sigSubTreeMoved;
    Signal<void(Item* item)> sigItemMoved;
    Signal<void(Item* item, bool isMoving)> sigSubTreeRemoving;
    Signal<void(Item* item, bool isMoving)> sigSubTreeRemoved;
    LazySignal< Signal<void()> > sigTreeChanged;
    Signal<void(Item* assigned, Item* srcItem)> sigItemAssigned;
    
    void emitSigItemAddedForItemTree(Item* item);
    void emitSigItemMovedForItemTree(Item* item);
    void notifyEventOnSubTreeRemoved(Item* item, bool isMoving);
};
}


void RootItem::initializeClass(ExtensionManager* ext)
{
    static bool initialized = false;
    if(!initialized){
        ext->itemManager().registerClass<RootItem>(N_("RootItem"));
        ext->manage(RootItemPtr(mainInstance()));
        initialized = true;
    }
}


RootItem* RootItem::instance()
{
    static RootItem* rootItem = new RootItem();
    rootItem->setName("Root");
    return rootItem;
}


RootItem* RootItem::mainInstance()
{
    return instance();
}


RootItem::RootItem()
{
    initializeInstance();
}


RootItem::RootItem(const RootItem& org)
    : Item(org)
{
    initializeInstance();
}


void RootItem::initializeInstance()
{
    impl = new RootItemImpl(this);
}


RootItemImpl::RootItemImpl(RootItem* self) :
    self(self)
{

}


RootItem::~RootItem()
{
    // This is also written in Item::~Item(), but this should be also written here
    // to notify detaching from the root even when the root is destroyed
    Item* child = childItem();
    while(child){
        Item* next = child->nextItem();
        child->detachFromParentItem();
        child = next;
    }

    delete impl;
}


RootItemImpl::~RootItemImpl()
{
    if(TRACE_FUNCTIONS){
        cout << "RootItemImpl::~RootItemImpl()" << endl;
    }
    sigDestroyed(self);
}


SignalProxy<void(RootItem* rootItem)> RootItem::sigDestroyed()
{
    return impl->sigDestroyed;
}


SignalProxy<void(Item* item)> RootItem::sigSubTreeAdded()
{
    return impl->sigSubTreeAdded;
}


SignalProxy<void(Item* item)> RootItem::sigItemAdded()
{
    return impl->sigItemAdded;
}


SignalProxy<void(Item* item)> RootItem::sigSubTreeMoved()
{
    return impl->sigSubTreeMoved;
}


SignalProxy<void(Item* item)> RootItem::sigItemMoved()
{
    return impl->sigItemMoved;
}


/**
   @if jp
   The signal that is emitted just before an item belonging to the path from
   the root item is being removed.
   
   @param isMoving It is true if the item is moving and again belongs to the
   path from the root item.
   
   @todo This signal should be replaced with the itemRemoved signal and deprecated.
*/
SignalProxy<void(Item* item, bool isMoving)> RootItem::sigSubTreeRemoving()
{
    return impl->sigSubTreeRemoving;
}


/**
   @if jp
   The signal that is emitted when an item belonging to the item tree from the
   root item is removed.
   
   @param isMoving It is true if the item is moving and again belongs to the
   path from the root item.
*/
SignalProxy<void(Item* item, bool isMoving)> RootItem::sigSubTreeRemoved()
{
    return impl->sigSubTreeRemoved;
}


/**
   @if jp
   The signal that is emitted when the structure of the item tree changes,
   such as adding and deleting items.

   Unlike sigItemAdded or sigItemRemoving, it is emitted only once for a series of
   operations performed at once. To be precise, it is emitted after the events in
   the queue are processed in the event loop of the framework.   
   
   @todo "Once all at once" is probably not being protected at the time of project
   loading etc, so improve this point.
*/
SignalProxy<void()> RootItem::sigTreeChanged()
{
    return impl->sigTreeChanged.signal();
}


/**
   This signal is emitted when Item::asign() is called.
*/
SignalProxy<void(Item* assigned, Item* srcItem)> RootItem::sigItemAssigned()
{
    return impl->sigItemAssigned;
}


void RootItem::notifyEventOnSubTreeAdded(Item* item)
{
    if(TRACE_FUNCTIONS){
        cout << "RootItem::notifyEventOnItemAdded()" << endl;
    }

    impl->sigSubTreeAdded(item);

    impl->emitSigItemAddedForItemTree(item);
    
    impl->sigTreeChanged.request();
}


void RootItemImpl::emitSigItemAddedForItemTree(Item* item)
{
    sigItemAdded(item);
    for(Item* child = item->childItem(); child; child = child->nextItem()){
        emitSigItemAddedForItemTree(child);
    }
}


void RootItem::notifyEventOnSubTreeMoved(Item* item)
{
    if(TRACE_FUNCTIONS){
        cout << "RootItem::notifyEventOnItemMoved()" << endl;
    }

    impl->sigSubTreeMoved(item);

    impl->emitSigItemMovedForItemTree(item);
    
    impl->sigTreeChanged.request();
}


void RootItemImpl::emitSigItemMovedForItemTree(Item* item)
{
    sigItemMoved(item);
    for(Item* child = item->childItem(); child; child = child->nextItem()){
        emitSigItemMovedForItemTree(child);
    }
}


void RootItem::notifyEventOnSubTreeRemoving(Item* item, bool isMoving)
{
    impl->sigSubTreeRemoving(item, isMoving);
    impl->sigTreeChanged.request();
}


void RootItem::notifyEventOnSubTreeRemoved(Item* item, bool isMoving)
{
    impl->notifyEventOnSubTreeRemoved(item, isMoving);
}


void RootItemImpl::notifyEventOnSubTreeRemoved(Item* item, bool isMoving)
{
    sigSubTreeRemoved(item, isMoving);
    sigTreeChanged.request();
}


void RootItem::emitSigItemAssinged(Item* assigned, Item* srcItem)
{
    impl->sigItemAssigned(assigned, srcItem);
}


Item* RootItem::doDuplicate() const
{
    return new RootItem(*this);
}


bool RootItem::store(Archive& archive)
{
    return true;
}


bool RootItem::restore(const Archive& archive)
{
    return true;
}

