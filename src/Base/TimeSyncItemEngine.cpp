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

using namespace std;
using namespace cnoid;

namespace {

TimeSyncItemEngineManager* manager = nullptr;
TimeSyncItemEngineManager::Impl* managerImpl = nullptr;

}

namespace cnoid {

class TimeSyncItemEngineManager::Impl
{
public:
    TimeBar* timeBar;
    double currentTime;
    bool isDoingPlayback;
    ItemClassRegistry& itemClassRegistry;
    vector<vector<std::function<TimeSyncItemEngine*(Item* item)>>> classIdToFactoryListMap;
    vector<TimeSyncItemEnginePtr> activeEngines;
    ScopedConnectionSet connections;

    Impl();
    void onSelectedItemsChanged(const ItemList<>& selectedItems);
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

    connections.add(
        RootItem::instance()->sigSelectedItemsChanged().connect(
            [&](const ItemList<>& selectedItems){
                onSelectedItemsChanged(selectedItems);
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
(const std::type_info& type, std::function<TimeSyncItemEngine*(Item* item)> factory)
{
    int id = impl->itemClassRegistry.classId(type);
    if(id >= static_cast<int>(impl->classIdToFactoryListMap.size())){
        impl->classIdToFactoryListMap.resize(id + 1);
    }
    impl->classIdToFactoryListMap[id].push_back(factory);
}


void TimeSyncItemEngineManager::Impl::onSelectedItemsChanged(const ItemList<>& selectedItems)
{
    activeEngines.clear();

    for(auto& item : selectedItems){
        int id = item->classId();
        while(id > 0){
            if(id < static_cast<int>(classIdToFactoryListMap.size())){
                for(auto& factory : classIdToFactoryListMap[id]){
                    if(auto engine = factory(item)){
                        activeEngines.push_back(engine);
                    }
                }
            }
            id = itemClassRegistry.superClassId(id);
        }
    }

    onTimeChanged(currentTime);
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
    for(auto& engine : activeEngines){
        isActive |= engine->onTimeChanged(time);
    }
    return isActive;
}


void TimeSyncItemEngineManager::Impl::onPlaybackStopped(double time, bool isStoppedManually)
{
    isDoingPlayback = false;
    for(auto& engine : activeEngines){
        engine->onPlaybackStopped(time, isStoppedManually);
    }
}


void TimeSyncItemEngineManager::Impl::refresh(TimeSyncItemEngine* engine)
{
    if(!isDoingPlayback){
        engine->onTimeChanged(currentTime);
    }
}


TimeSyncItemEngine::TimeSyncItemEngine()
{
    fillLevelId = -1;
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


bool TimeSyncItemEngine::isPlaybackAlwaysMaintained() const
{
    return false;
}


bool TimeSyncItemEngine::startUpdatingTime()
{
    if(fillLevelId < 0){
        fillLevelId = managerImpl->timeBar->startFillLevelUpdate(managerImpl->currentTime);
    }
    return fillLevelId >= 0;
}


void TimeSyncItemEngine::updateTime(double time)
{
    if(fillLevelId >= 0){
        managerImpl->timeBar->updateFillLevel(fillLevelId, time);
    }
}


void TimeSyncItemEngine::stopUpdatingTime()
{
    if(fillLevelId >= 0){
        managerImpl->timeBar->stopFillLevelUpdate(fillLevelId);
        fillLevelId = -1;
    }
}


void TimeSyncItemEngine::refresh()
{
    managerImpl->refresh(this);
}
