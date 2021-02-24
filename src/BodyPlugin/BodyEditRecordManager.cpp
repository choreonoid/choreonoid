#include "BodyEditRecordManager.h"
#include "BodyItem.h"
#include <cnoid/BodyState>
#include <cnoid/UnifiedEditHistory>
#include <cnoid/EditRecord>
#include <cnoid/ExtensionManager>
#include <cnoid/RootItem>
#include <cnoid/ConnectionSet>
#include <fmt/format.h>
#include <unordered_map>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

namespace {

BodyEditRecordManager::Impl* manager;

class KinematicStateRecord;
typedef ref_ptr<KinematicStateRecord> KinematicStateRecordPtr;

class KinematicStateRecord : public EditRecord
{
public:
    BodyItemPtr bodyItem;
    BodyState newState;
    BodyState oldState;
    
    KinematicStateRecord(BodyItem* bodyItem);
    KinematicStateRecord(BodyItem* bodyItem, const BodyState& oldState);
    KinematicStateRecord(const KinematicStateRecord& org);

    virtual EditRecord* clone() const override;
    virtual std::string label() const override;
    virtual bool undo() override;
    virtual bool redo() override;
};

class BodyItemInfo : public Referenced
{
public:
    ScopedConnectionSet connections;
    BodyState lastState;
};

typedef ref_ptr<BodyItemInfo> BodyItemInfoPtr;
    
}

namespace cnoid {

class BodyEditRecordManager::Impl
{
public:
    UnifiedEditHistory* history;
    ScopedConnection rootItemConnection;
    unordered_map<ItemPtr, BodyItemInfoPtr> bodyItemInfoMap;

    Impl();
    void onItemAdded(Item* item);
    void onBodyItemDisconnectedFromRoot(BodyItem* bodyItem);
    void onKinematicStateEdited(BodyItem* bodyItem, BodyItemInfo* info);
    BodyItemInfo* findBodyItemInfo(BodyItem* bodyItem);
};

}


void BodyEditRecordManager::initializeClass(ExtensionManager* ext)
{
    static BodyEditRecordManager* instance = new BodyEditRecordManager;
    ext->manage(instance);
    manager = instance->impl;
}


BodyEditRecordManager::BodyEditRecordManager()
{
    impl = new Impl;
}


BodyEditRecordManager::~BodyEditRecordManager()
{
    delete impl;
}


BodyEditRecordManager::Impl::Impl()
{
    history = UnifiedEditHistory::instance();

    rootItemConnection =
        RootItem::instance()->sigItemAdded().connect(
            [&](Item* item){ onItemAdded(item); });
}


void BodyEditRecordManager::Impl::onItemAdded(Item* item)
{
    if(auto bodyItem = dynamic_cast<BodyItem*>(item)){
        auto& info = bodyItemInfoMap[bodyItem];
        if(!info){
            info = new BodyItemInfo;

            // Smart point should not be used here to avoid a circular reference
            BodyItemInfo* rawInfo = info;
            info->connections.add(
                bodyItem->sigKinematicStateEdited().connect(
                    [this, bodyItem, rawInfo](){ onKinematicStateEdited(bodyItem, rawInfo); }));
            
            info->connections.add(
                bodyItem->sigDisconnectedFromRoot().connect(
                    [this, bodyItem](){ onBodyItemDisconnectedFromRoot(bodyItem); }));

            bodyItem->storeKinematicState(info->lastState);
        }
    }
}


void BodyEditRecordManager::Impl::onBodyItemDisconnectedFromRoot(BodyItem* bodyItem)
{
    bodyItemInfoMap.erase(bodyItem);
}


void BodyEditRecordManager::Impl::onKinematicStateEdited(BodyItem* bodyItem, BodyItemInfo* info)
{
    auto record = new KinematicStateRecord(bodyItem, info->lastState);
    history->addRecord(record);
    bodyItem->storeKinematicState(info->lastState);
}


BodyItemInfo* BodyEditRecordManager::Impl::findBodyItemInfo(BodyItem* bodyItem)
{
    auto p = bodyItemInfoMap.find(bodyItem);
    if(p != bodyItemInfoMap.end()){
        return p->second;
    }
    return nullptr;
}


KinematicStateRecord::KinematicStateRecord(BodyItem* bodyItem)
    : bodyItem(bodyItem)
{
    bodyItem->storeKinematicState(newState);
    oldState = newState;
}


KinematicStateRecord::KinematicStateRecord(BodyItem* bodyItem, const BodyState& oldState)
    : bodyItem(bodyItem),
      oldState(oldState)
{
    bodyItem->storeKinematicState(newState);
}


KinematicStateRecord::KinematicStateRecord(const KinematicStateRecord& org)
    : EditRecord(org),
      bodyItem(org.bodyItem),
      newState(org.newState),
      oldState(org.oldState)
{

}


EditRecord* KinematicStateRecord::clone() const
{
    return new KinematicStateRecord(*this);
}


std::string KinematicStateRecord::label() const
{
    if(!isReverse()){
        return format(_("Change the position of \"{0}\""), bodyItem->displayName());
    } else {
        return format(_("Restore the position of \"{0}\""), bodyItem->displayName());
    }
}


bool KinematicStateRecord::undo()
{
    if(auto info = manager->findBodyItemInfo(bodyItem)){
        auto block = info->connections.scopedBlock();
        if(bodyItem->restoreKinematicState(oldState)){
            bodyItem->storeKinematicState(info->lastState);
            bodyItem->notifyKinematicStateChange();
        }
        return true;
    }
    return false;
}


bool KinematicStateRecord::redo()
{
    if(auto info = manager->findBodyItemInfo(bodyItem)){
        auto block = info->connections.scopedBlock();
        if(bodyItem->restoreKinematicState(newState)){
            bodyItem->storeKinematicState(info->lastState);
            bodyItem->notifyKinematicStateChange();
        }
        return true;
    }
    return false;
}
