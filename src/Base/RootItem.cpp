/**
   @author Shin'ichiro Nakaoka
*/

#include "RootItem.h"
#include "ExtensionManager.h"
#include "ItemManager.h"
#include "LazySignal.h"
#include <boost/bind.hpp>
#include <iostream>
#include "gettext.h"

using namespace std;
using namespace boost;
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
   本シグナルを所有するルートアイテムからのパスに所属する子アイテムがパスから
   取り除かれる直前に発行されるシグナル。
   
   @param isMoving アイテムが移動中であって、再び本ルートアイテムからのパスに
   所属する場合、true となる。
   
   @todo できれば本シグナルは itemRemoved() シグナルで置き換えてdeprecatedとしたい。
*/
SignalProxy<void(Item* item, bool isMoving)> RootItem::sigSubTreeRemoving()
{
    return impl->sigSubTreeRemoving;
}


/**
   @if jp
   本シグナルを所有するルートアイテムからのパスに所属する子アイテムがパスから
   取り除かれた後に呼ばれるスロットを接続する。
   
   @param isMoving アイテムが移動中であって、再び本ルートアイテムからのパスに
   所属する場合、true となる。
*/
SignalProxy<void(Item* item, bool isMoving)> RootItem::sigSubTreeRemoved()
{
    return impl->sigSubTreeRemoved;
}


/**
   @if jp
   アイテムの追加・削除など、アイテムツリーの構造が変化したときに呼ばれるスロットを接続する。
   
   sigItemAdded や sigItemRemoving とは異なり、一度に行われる一連の操作に対して
   １回だけまとめて発行される。正確には、フレームワークのイベントループでキューにあるイベント
   が処理されてから実行される。
   
   @todo 「1回だけまとめて」は恐らくプロジェクト読み込み時などには守られていないので，
   この点改善しておく．
   @endif
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


ItemPtr RootItem::doDuplicate() const
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

