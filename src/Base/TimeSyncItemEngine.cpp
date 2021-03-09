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
    ItemClassRegistry& itemClassRegistry;

    vector<vector<FactoryPtr>> classIdToFactoryListMap;

    typedef unordered_map<FactoryPtr, TimeSyncItemEnginePtr> FactoryToEngineMap;
    typedef unordered_map<ItemPtr, FactoryToEngineMap> ItemToFactoryToEngineMap;
    ItemToFactoryToEngineMap itemToFactoryToEngineMap;
    ItemToFactoryToEngineMap itemToFactoryToEngineMap0;

    vector<TimeSyncItemEnginePtr> activeEngines;
    int numPreExistingActiveEngines;

    bool isDoingPlayback;

    ScopedConnectionSet connections;

    Impl();
    int createEngines(
        Item* item, std::vector<TimeSyncItemEnginePtr>& io_engines,
        bool doCheckExistingEngines, bool itemTreeMayBeChanged);
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
        rootItem->sigTreeChanged().connect(
            [this, rootItem](){
                updateEnginesForSelectedItems(rootItem->selectedItems(), true);
            }));

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


int TimeSyncItemEngineManager::createEngines(Item* item, std::vector<TimeSyncItemEnginePtr>& io_engines)
{
    return impl->createEngines(item, io_engines, false, true);
}


int TimeSyncItemEngineManager::Impl::createEngines
(Item* item, std::vector<TimeSyncItemEnginePtr>& io_engines, bool doCheckExistingEngines, bool itemTreeMayBeChanged)
{
    int numCreatedEngines = 0;
    
    int id = item->classId();
    while(id > 0){
        if(id < static_cast<int>(classIdToFactoryListMap.size())){

            FactoryToEngineMap* pFactoryToEngineMap0 = nullptr;
            if(doCheckExistingEngines){
                auto p = itemToFactoryToEngineMap0.find(item);
                if(p != itemToFactoryToEngineMap0.end()){
                    pFactoryToEngineMap0 = &p->second;
                }
            }
            
            for(auto& factory : classIdToFactoryListMap[id]){
                TimeSyncItemEnginePtr existingEngine;
                if(pFactoryToEngineMap0){
                    auto q = pFactoryToEngineMap0->find(factory);
                    if(q != pFactoryToEngineMap0->end()){
                        existingEngine = q->second;
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
                    if(doCheckExistingEngines){
                        itemToFactoryToEngineMap[item][factory] = engine;
                        if(engine != existingEngine){
                            engine->onTimeChanged(currentTime);
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
                    }
                    if(!isPreExistingActiveEngine){
                        io_engines.push_back(engine);
                    }
                    ++numCreatedEngines;
                }
            }
        }
        id = itemClassRegistry.superClassId(id);
    }
    
    return numCreatedEngines;
}


void TimeSyncItemEngineManager::Impl::updateEnginesForSelectedItems
(const ItemList<>& selectedItems, bool itemTreeMayBeChanged)
{
    itemToFactoryToEngineMap0.clear();
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
        createEngines(item, activeEngines, true, itemTreeMayBeChanged);
    }
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
        if(engine->isTimeSyncForcedToBeMaintained()){
            iter = activeEngines.erase(iter);
            continue;
        }
        engine->onPlaybackStopped(time, isStoppedManually);
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
    isTimeSyncForcedToBeMaintained_ = false;
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


bool TimeSyncItemEngine::startOngoingTimeUpdate()
{
    if(ongoingTimeId < 0){
        ongoingTimeId = managerImpl->timeBar->startOngoingTimeUpdate(managerImpl->currentTime);
    }
    return ongoingTimeId >= 0;
}


void TimeSyncItemEngine::updateOngoingTime(double time)
{
    if(ongoingTimeId >= 0){
        managerImpl->timeBar->updateOngoingTime(ongoingTimeId, time);
    }
}


void TimeSyncItemEngine::stopOngoingTimeUpdate()
{
    if(ongoingTimeId >= 0){
        managerImpl->timeBar->stopOngoingTimeUpdate(ongoingTimeId);
        ongoingTimeId = -1;
    }
}


void TimeSyncItemEngine::refresh()
{
    managerImpl->refresh(this);
}
