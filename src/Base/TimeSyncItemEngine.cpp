#include "TimeSyncItemEngine.h"
#include "ItemClassRegistry.h"
#include "RootItem.h"
#include "ItemList.h"
#include "TimeBar.h"
#include "ExtensionManager.h"
#include "LazyCaller.h"
#include <cnoid/ConnectionSet>
#include <memory>
#include <unordered_map>
#include <map>
#include <set>

using namespace std;
using namespace cnoid;

namespace {

TimeSyncItemEngineManager* manager = nullptr;
TimeSyncItemEngineManager::Impl* managerImpl = nullptr;

typedef std::function<TimeSyncItemEngine*(Item* item, TimeSyncItemEngine* prevEngine)> Factory;
typedef shared_ptr<Factory> FactoryPtr;

}

namespace cnoid {

class TimeSyncItemEngineManager::Impl
{
public:
    RootItem* rootItem;
    TimeBar* timeBar;
    double currentTime;
    bool isDoingPlayback;
    ItemClassRegistry& itemClassRegistry;
    vector<vector<FactoryPtr>> classIdToFactoryListMap;

    typedef unordered_map<FactoryPtr, TimeSyncItemEnginePtr> FactoryToEngineMap;

    struct ItemInfo
    {
        FactoryToEngineMap factoryToEngineMap;
        vector<TimeSyncItemEnginePtr> activeEngines;
        ScopedConnectionSet connections;
        bool isActiveItem;
    };
    map<ItemPtr, ItemInfo> itemInfoMap;
    set<ItemInfo*> activeItemInfos;
    LazyCaller refreshActiveItemsLater;

    ScopedConnectionSet connections;

    Impl();
    void onItemAdded(Item* item);
    void onItemDisconnectedFromRoot(Item* item, ItemInfo* info);
    void updateItemEngines(Item* item, ItemInfo* info, bool forceUpdate);
    void activateItemEngines(Item* item, ItemInfo* info, bool forceUpdate);
    void deactivateItemEngines(Item* item, ItemInfo* info, bool forceUpdate);
    int createEngines(Item* item, std::vector<TimeSyncItemEnginePtr>& io_engines);
    void refreshActiveItems();
    void refreshItem(TimeSyncItemEngine* engine);
    bool onPlaybackInitialized(double time);
    void onPlaybackStarted(double time);
    bool onTimeChanged(double time);
    double onPlaybackStopped(double time, bool isStoppedManually);
};

}


void TimeSyncItemEngineManager::initializeClass(ExtensionManager* ext)
{
    if(!manager){
        manager = new TimeSyncItemEngineManager;
        managerImpl = manager->impl;
        ext->manage(manager);
    }
}


TimeSyncItemEngineManager* TimeSyncItemEngineManager::instance()
{
    return manager;
}


TimeSyncItemEngineManager::TimeSyncItemEngineManager()
{
    impl = new Impl;
}


TimeSyncItemEngineManager::Impl::Impl()
    : itemClassRegistry(ItemClassRegistry::instance()),
      refreshActiveItemsLater([this](){ refreshActiveItems(); }, LazyCaller::NormalPriority)
{
    currentTime = 0.0;
    isDoingPlayback = false;

    rootItem = RootItem::instance();

    connections.add(
        rootItem->sigItemAdded().connect(
            [this](Item* item){ onItemAdded(item); }));

    timeBar = TimeBar::instance();
    connections.add(
        timeBar->sigPlaybackInitialized().connect(
            [&](double time){ return onPlaybackInitialized(time);  }));
    connections.add(
        timeBar->sigPlaybackStarted().connect(
            [&](double time){ onPlaybackStarted(time);  }));
    connections.add(
        timeBar->sigTimeChanged().connect(
            [&](double time){ return onTimeChanged(time); }));
    connections.add(
        timeBar->sigPlaybackStoppedEx().connect(
            [&](double time, bool isStoppedManually){
                return onPlaybackStopped(time, isStoppedManually);
            }));
}


TimeSyncItemEngineManager::~TimeSyncItemEngineManager()
{
    delete impl;
}


void TimeSyncItemEngineManager::registerFactory_
(const std::type_info& type,
 const std::function<TimeSyncItemEngine*(Item* item, TimeSyncItemEngine* prevEngine)>& factory)
{
    int id = impl->itemClassRegistry.getClassId(type);
    if(id >= static_cast<int>(impl->classIdToFactoryListMap.size())){
        impl->classIdToFactoryListMap.resize(id + 1);
    }
    impl->classIdToFactoryListMap[id].push_back(make_shared<Factory>(factory));
}


void TimeSyncItemEngineManager::Impl::onItemAdded(Item* item)
{
    ItemInfo* info = nullptr;
    int id = item->classId();

    while(id > 0){
        if(id < static_cast<int>(classIdToFactoryListMap.size())){
            auto& factories = classIdToFactoryListMap[id];
            if(!factories.empty()){
                if(!info){
                    info = &itemInfoMap[item];
                    info->isActiveItem = false;
                }
                for(auto& factory : factories){
                    info->factoryToEngineMap[factory].reset();
                    info->connections.add(
                        item->sigSelectionChanged().connect(
                            [this, item, info](bool){
                                updateItemEngines(item, info, false);
                            }));
                    info->connections.add(
                        item->sigCheckToggled().connect(
                            [this, item, info](bool){
                                updateItemEngines(item, info, false);
                            }));
                    info->connections.add(
                        item->sigTreePositionChanged().connect(
                            [this, item, info](){
                                updateItemEngines(item, info, true);
                            }));
                    info->connections.add(
                        item->sigDisconnectedFromRoot().connect(
                            [this, item, info](){
                                onItemDisconnectedFromRoot(item, info);
                            }));
                }
            }
        }
        id = itemClassRegistry.getSuperClassId(id);
    }

    if(info){
        updateItemEngines(item, info, true);
    }
}


void TimeSyncItemEngineManager::Impl::onItemDisconnectedFromRoot(Item* item, ItemInfo* info)
{
    activeItemInfos.erase(info);
    itemInfoMap.erase(item);
}


void TimeSyncItemEngineManager::Impl::updateItemEngines(Item* item, ItemInfo* info, bool forceUpdate)
{
    bool isActiveItem = item->isSelected() || item->isChecked();
    if(forceUpdate || (isActiveItem != info->isActiveItem)){
        info->isActiveItem = isActiveItem;
        if(isActiveItem){
            activateItemEngines(item, info, forceUpdate);
        } else {
            deactivateItemEngines(item, info, forceUpdate);
        }
    }
}


void TimeSyncItemEngineManager::Impl::activateItemEngines(Item* item, ItemInfo* info, bool forceUpdate)
{
    bool updated = false;
    bool prevActiveEngineExistence = !info->activeEngines.empty();
    info->activeEngines.clear();

    for(auto& kv : info->factoryToEngineMap){
        auto existingEngine = kv.second;
        TimeSyncItemEnginePtr newEngine;
        if(existingEngine && !forceUpdate){
            newEngine = existingEngine;
        } else {
            auto& factory = kv.first;
            newEngine = (*factory)(item, existingEngine);
        }
        if(newEngine){
            newEngine->isTimeSyncForcedToBeMaintained_ = false;
            bool isPreExistingActiveEngine = false;
            kv.second = newEngine;
            if(newEngine != existingEngine){
                if(existingEngine){
                    existingEngine->deactivate();
                    updated = true;
                }
            }
            if(!newEngine->isActive_){
                newEngine->activate();
                updated = true;
            }
            info->activeEngines.push_back(newEngine);
        }
    }
    
    if(info->activeEngines.empty()){
        if(prevActiveEngineExistence){
            activeItemInfos.erase(info);
        }
    } else {
        if(!prevActiveEngineExistence){
            activeItemInfos.insert(info);
        }
    }

    if(updated){
        refreshActiveItemsLater();
    }
}


void TimeSyncItemEngineManager::Impl::deactivateItemEngines(Item* item, ItemInfo* info, bool forceUpdate)
{
    bool updated = false;
    bool prevActiveEngineExistence = !info->activeEngines.empty();

    auto it = info->activeEngines.begin();
    while(it != info->activeEngines.end()){
        auto engine = *it;
        if(forceUpdate || !engine->isTimeSyncAlwaysMaintained()){
            engine->deactivate();
            it = info->activeEngines.erase(it);
            updated = true;
        } else {            
            engine->isTimeSyncForcedToBeMaintained_ = true;
            ++it;
        }
    }

    for(auto& kv : info->factoryToEngineMap){
        auto& engine = kv.second;
        if(engine){
            if(forceUpdate || !engine->isTimeSyncForcedToBeMaintained_){
                engine.reset();
            }
        }
    }

    if(prevActiveEngineExistence && info->activeEngines.empty()){
        activeItemInfos.erase(info);
    }

    if(updated){
        refreshActiveItemsLater();
    }
}    


int TimeSyncItemEngineManager::createEngines(Item* item, std::vector<TimeSyncItemEnginePtr>& io_engines)
{
    return impl->createEngines(item, io_engines);
}


int TimeSyncItemEngineManager::Impl::createEngines(Item* item, std::vector<TimeSyncItemEnginePtr>& io_engines)
{
    int numCreatedEngines = 0;
    int id = item->classId();
    while(id > 0){
        if(id < static_cast<int>(classIdToFactoryListMap.size())){
            for(auto& factory : classIdToFactoryListMap[id]){
                if(auto engine = (*factory)(item, nullptr)){
                    engine->isTimeSyncForcedToBeMaintained_ = false;
                    io_engines.push_back(engine);
                    ++numCreatedEngines;
                }
            }
        }
        id = itemClassRegistry.getSuperClassId(id);
    }
    return numCreatedEngines;
}


void TimeSyncItemEngineManager::Impl::refreshActiveItems()
{
    if(!isDoingPlayback){
        for(auto& info : activeItemInfos){
            for(auto& engine : info->activeEngines){
                engine->onTimeChanged(currentTime);
            }
        }
    }
}


void TimeSyncItemEngineManager::Impl::refreshItem(TimeSyncItemEngine* engine)
{
    if(!isDoingPlayback){
        engine->onTimeChanged(currentTime);
    }
}


bool TimeSyncItemEngineManager::Impl::onPlaybackInitialized(double time)
{
    bool initialized = true;
    for(auto& info : activeItemInfos){
        for(auto& engine : info->activeEngines){
            if(!engine->onPlaybackInitialized(time)){
                initialized = false;
                break;
            }
        }
    }
    return initialized;
}


void TimeSyncItemEngineManager::Impl::onPlaybackStarted(double time)
{
    isDoingPlayback = true;
    for(auto& info : activeItemInfos){
        for(auto& engine : info->activeEngines){
            engine->onPlaybackStarted(time);
        }
    }
}


bool TimeSyncItemEngineManager::Impl::onTimeChanged(double time)
{
    bool isActive = false;
    currentTime = time;

    auto it1 = activeItemInfos.begin();
    while(it1 != activeItemInfos.end()){
        bool doErase = false;
        auto info = *it1;
        auto it2 = info->activeEngines.begin();
        while(it2 != info->activeEngines.end()){
            auto& engine = *it2;
            if(engine->isTimeSyncForcedToBeMaintained()){
                if(!engine->isTimeSyncAlwaysMaintained()){
                    if(isDoingPlayback){
                        engine->onPlaybackStopped(time, false);
                    }
                    engine->deactivate();
                    it2 = info->activeEngines.erase(it2);
                    if(info->activeEngines.empty()){
                        doErase = true;
                        break;
                    }
                    continue;
                }
            }
            isActive |= engine->onTimeChanged(time);
            ++it2;
        }
        if(doErase){
            it1 = activeItemInfos.erase(it1);
        } else {
            ++it1;
        }
    }

    return isActive;
}


double TimeSyncItemEngineManager::Impl::onPlaybackStopped(double time, bool isStoppedManually)
{
    double maxLastValidTime = 0.0;
    isDoingPlayback = false;

    auto it1 = activeItemInfos.begin();
    while(it1 != activeItemInfos.end()){
        bool doErase = false;
        auto info = *it1;
        auto it2 = info->activeEngines.begin();
        while(it2 != info->activeEngines.end()){
            auto& engine = *it2;
            double lastValidTime = engine->onPlaybackStopped(time, isStoppedManually);
            if(lastValidTime > maxLastValidTime){
                maxLastValidTime = lastValidTime;
            }
            if(engine->isTimeSyncForcedToBeMaintained()){
                engine->deactivate();
                it2 = info->activeEngines.erase(it2);
                if(info->activeEngines.empty()){
                    doErase = true;
                    break;
                }
                continue;
            }
            ++it2;
        }
        if(doErase){
            it1 = activeItemInfos.erase(it1);
        } else {
            ++it1;
        }
    }

    return maxLastValidTime;
}


TimeSyncItemEngine::TimeSyncItemEngine(Item* item)
    : item_(item)
{
    ongoingTimeId = -1;
    ongoingTime = 0.0;
    isActive_ = false;
    isTimeSyncForcedToBeMaintained_ = false;
    isUpdatingOngoingTime_ = false;
}


TimeSyncItemEngine::~TimeSyncItemEngine()
{
    deactivate();
}


bool TimeSyncItemEngine::onPlaybackInitialized(double /* time */)
{
    return true;
}


void TimeSyncItemEngine::onPlaybackStarted(double /* time */)
{

}


double TimeSyncItemEngine::onPlaybackStopped(double time, bool /* isStoppedManually */)
{
    return time;
}


bool TimeSyncItemEngine::isTimeSyncAlwaysMaintained() const
{
    return false;
}


void TimeSyncItemEngine::activate()
{
    isActive_ = true;
    if(isUpdatingOngoingTime()){
        setupOngoingTimeUpdate();
    }
}


void TimeSyncItemEngine::deactivate()
{
    isActive_ = false;
    if(ongoingTimeId >= 0){
        managerImpl->timeBar->stopOngoingTimeUpdate(ongoingTimeId);
        ongoingTimeId = -1;
    }
}


void TimeSyncItemEngine::setupOngoingTimeUpdate()
{
    if(ongoingTimeId < 0){
        ongoingTimeId = managerImpl->timeBar->startOngoingTimeUpdate(ongoingTime);
    }
}


void TimeSyncItemEngine::startOngoingTimeUpdate()
{
    startOngoingTimeUpdate(ongoingTime);
}


void TimeSyncItemEngine::startOngoingTimeUpdate(double time)
{
    ongoingTime = time;
    isUpdatingOngoingTime_ = true;

    if(isActive_){
        setupOngoingTimeUpdate();
        if(!managerImpl->isDoingPlayback){
            managerImpl->timeBar->startPlayback(ongoingTime);
        }
    }
}


void TimeSyncItemEngine::updateOngoingTime(double time)
{
    ongoingTime = time;
    if(ongoingTimeId >= 0 && isActive_){
        managerImpl->timeBar->updateOngoingTime(ongoingTimeId, time);
    }
}


void TimeSyncItemEngine::stopOngoingTimeUpdate()
{
    if(ongoingTimeId >= 0){
        managerImpl->timeBar->stopOngoingTimeUpdate(ongoingTimeId);
        ongoingTimeId = -1;
    }
    isUpdatingOngoingTime_ = false;
}


void TimeSyncItemEngine::refresh()
{
    managerImpl->refreshItem(this);
}
