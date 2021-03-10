/**
   @author Shin'ichiro Nakaoka
*/

#include "TimeSyncItemEngine.h"
#include "ItemClassRegistry.h"
#include "RootItem.h"
#include "ItemList.h"
#include "TimeBar.h"
#include "ExtensionManager.h"
#include <cnoid/ConnectionSet>
#include <vector>
#include <unordered_map>

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
    TimeBar* timeBar;
    double currentTime;
    bool isDoingPlayback;
    ItemClassRegistry& itemClassRegistry;
    vector<vector<FactoryPtr>> classIdToFactoryListMap;

    typedef unordered_map<FactoryPtr, TimeSyncItemEnginePtr> FactoryToEngineMap;
    typedef unordered_map<ItemPtr, FactoryToEngineMap> ItemToFactoryToEngineMap;
    ItemToFactoryToEngineMap itemToFactoryToEngineMap;
    ItemToFactoryToEngineMap itemToFactoryToEngineMap0;

    vector<TimeSyncItemEnginePtr> activeEngines;
    int numPreExistingActiveEngines;

    ScopedConnectionSet connections;

    Impl();
    void createManagedEngines(Item* item, std::vector<TimeSyncItemEnginePtr>& io_engines, bool itemTreeMayBeChanged);
    int createEngines(Item* item, std::vector<TimeSyncItemEnginePtr>& io_engines);
    void updateEnginesForSelectedItems(const ItemList<>& selectedItems, bool itemTreeMayBeChanged);
    bool onPlaybackInitialized(double time);
    void onPlaybackStarted(double time);
    bool onTimeChanged(double time);
    void onPlaybackStopped(double time, bool isStoppedManually);
    void refresh(TimeSyncItemEngine* engine);
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
    : itemClassRegistry(ItemClassRegistry::instance())
{
    currentTime = 0.0;
    isDoingPlayback = false;

    auto rootItem = RootItem::instance();

    connections.add(
        rootItem->sigSubTreeChanged().connect(
            [this, rootItem](){
                updateEnginesForSelectedItems(rootItem->selectedItems(), true); }));

    connections.add(
        rootItem->sigSelectedItemsChanged().connect(
            [this](const ItemList<>& selectedItems){
                updateEnginesForSelectedItems(selectedItems, false);
            }));

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
        timeBar->sigPlaybackStopped().connect(
            [&](double time, bool isStoppedManually){
                onPlaybackStopped(time, isStoppedManually);
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
    int id = impl->itemClassRegistry.classId(type);
    if(id >= static_cast<int>(impl->classIdToFactoryListMap.size())){
        impl->classIdToFactoryListMap.resize(id + 1);
    }
    impl->classIdToFactoryListMap[id].push_back(make_shared<Factory>(factory));
}


void TimeSyncItemEngineManager::Impl::updateEnginesForSelectedItems
(const ItemList<>& selectedItems, bool itemTreeMayBeChanged)
{
    itemToFactoryToEngineMap.swap(itemToFactoryToEngineMap0);

    // Clear the active engine list except for the engines with the 'isTimeSyncAlwaysMaintained' flag.
    auto iter = activeEngines.begin();
    while(iter != activeEngines.end()){
        auto& engine = *iter;
        if(engine->isTimeSyncAlwaysMaintained()){
            auto item = engine->item();
            if(!item->isSelected() && item->isConnectedToRoot()){
                // Force to keep the engine
                engine->isTimeSyncForcedToBeMaintained_ = true;
                auto p = itemToFactoryToEngineMap0.find(item);
                if(p != itemToFactoryToEngineMap0.end()){
                    itemToFactoryToEngineMap[item] = std::move(p->second);
                    itemToFactoryToEngineMap0.erase(p);
                }
                ++iter;
                continue;
            }
        }
        iter = activeEngines.erase(iter);
    }
    
    numPreExistingActiveEngines = activeEngines.size();
        
    for(auto& item : selectedItems){
        createManagedEngines(item, activeEngines, itemTreeMayBeChanged);
    }

    for(auto& kv1 : itemToFactoryToEngineMap0){
        for(auto& kv2 : kv1.second){
            kv2.second->deactivate();
        }
    }
    itemToFactoryToEngineMap0.clear();
}


void TimeSyncItemEngineManager::Impl::createManagedEngines
(Item* item, std::vector<TimeSyncItemEnginePtr>& io_engines, bool itemTreeMayBeChanged)
{
    int id = item->classId();

    while(id > 0){
        if(id < static_cast<int>(classIdToFactoryListMap.size())){

            FactoryToEngineMap* pFactoryToEngineMap0 = nullptr;
            auto p = itemToFactoryToEngineMap0.find(item);
            if(p != itemToFactoryToEngineMap0.end()){
                pFactoryToEngineMap0 = &p->second;
            }
            for(auto& factory : classIdToFactoryListMap[id]){
                TimeSyncItemEnginePtr existingEngine;
                if(pFactoryToEngineMap0){
                    auto q = pFactoryToEngineMap0->find(factory);
                    if(q != pFactoryToEngineMap0->end()){
                        existingEngine = q->second;
                        pFactoryToEngineMap0->erase(q);
                    }
                }
                TimeSyncItemEnginePtr engine;
                if(existingEngine && !itemTreeMayBeChanged){
                    engine = existingEngine;
                } else {
                    engine = (*factory)(item, existingEngine);
                }
                if(engine){
                    engine->isTimeSyncForcedToBeMaintained_ = false;
                    bool isPreExistingActiveEngine = false;
                    itemToFactoryToEngineMap[item][factory] = engine;
                    if(engine != existingEngine){
                        if(existingEngine){
                            existingEngine->deactivate();
                        }
                        engine->activate();
                    } else {
                        if(engine->isTimeSyncAlwaysMaintained()){
                            for(int i=0; i < numPreExistingActiveEngines; ++i){
                                if(io_engines[i] == engine){
                                    isPreExistingActiveEngine = true;
                                    break;
                                }
                            }
                        }
                    }
                    if(!isPreExistingActiveEngine){
                        io_engines.push_back(engine);
                    }
                }
            }
        }
        id = itemClassRegistry.superClassId(id);
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
        id = itemClassRegistry.superClassId(id);
    }
    return numCreatedEngines;
}


bool TimeSyncItemEngineManager::Impl::onPlaybackInitialized(double time)
{
    bool initialized = true;
    for(auto& engine : activeEngines){
        if(!engine->onPlaybackInitialized(time)){
            initialized = false;
            break;
        }
    }
    return initialized;
}


void TimeSyncItemEngineManager::Impl::onPlaybackStarted(double time)
{
    isDoingPlayback = true;
    for(auto& engine : activeEngines){
        engine->onPlaybackStarted(time);
    }
}


bool TimeSyncItemEngineManager::Impl::onTimeChanged(double time)
{
    bool isActive = false;
    currentTime = time;
    auto iter = activeEngines.begin();
    while(iter != activeEngines.end()){
        auto& engine = *iter;
        if(engine->isTimeSyncForcedToBeMaintained()){
            if(!engine->isTimeSyncAlwaysMaintained()){
                if(isDoingPlayback){
                    engine->onPlaybackStopped(time, false);
                }
                engine->deactivate();
                iter = activeEngines.erase(iter);
                continue;
            }
        }
        isActive |= engine->onTimeChanged(time);
        ++iter;
    }
    return isActive;
}


void TimeSyncItemEngineManager::Impl::onPlaybackStopped(double time, bool isStoppedManually)
{
    isDoingPlayback = false;
    auto iter = activeEngines.begin();
    while(iter != activeEngines.end()){
        auto& engine = *iter;
        engine->onPlaybackStopped(time, isStoppedManually);
        if(engine->isTimeSyncForcedToBeMaintained()){
            engine->deactivate();
            iter = activeEngines.erase(iter);
            continue;
        }
        ++iter;
    }
}


void TimeSyncItemEngineManager::Impl::refresh(TimeSyncItemEngine* engine)
{
    if(!isDoingPlayback){
        engine->onTimeChanged(currentTime);
    }
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

}


bool TimeSyncItemEngine::onPlaybackInitialized(double /* time */)
{
    return true;
}


void TimeSyncItemEngine::onPlaybackStarted(double /* time */)
{

}


void TimeSyncItemEngine::onPlaybackStopped(double /* time */, bool /* isStoppedManually */)
{

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
    if(!managerImpl->isDoingPlayback){
        onTimeChanged(managerImpl->currentTime);
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
    managerImpl->refresh(this);
}
