/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_TIME_SYNC_ITEM_ENGINE_H
#define CNOID_BASE_TIME_SYNC_ITEM_ENGINE_H

#include <cnoid/Referenced>
#include <functional>
#include <typeinfo>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT TimeSyncItemEngine : public Referenced
{
public:
    TimeSyncItemEngine();
    virtual ~TimeSyncItemEngine();

    virtual bool onPlaybackInitialized(double time);
    virtual void onPlaybackStarted(double time);
    virtual bool onTimeChanged(double time) = 0;
    virtual void onPlaybackStopped(double time, bool isStoppedManually);
    virtual bool isPlaybackAlwaysMaintained() const;
    
    bool startUpdatingTime();
    void updateTime(double time);
    void stopUpdatingTime();
    
    void refresh();

private:
    int fillLevelId;
};

typedef ref_ptr<TimeSyncItemEngine> TimeSyncItemEnginePtr;

class ExtensionManager;
class Item;

class CNOID_EXPORT TimeSyncItemEngineManager
{
public:
    static void initializeClass(ExtensionManager* ext);
    static TimeSyncItemEngineManager* instance();

    ~TimeSyncItemEngineManager();

    template<class ItemType>
    void registerFactory(std::function<TimeSyncItemEngine*(ItemType* item)> factory){
        registerFactory_(
            typeid(ItemType),
            [factory](Item* item){ return factory(static_cast<ItemType*>(item)); });
    }

    class Impl;

private:
    TimeSyncItemEngineManager();

    void registerFactory_(
        const std::type_info& type, std::function<TimeSyncItemEngine*(Item* item)> factory);

    Impl* impl;
};

}

#endif
