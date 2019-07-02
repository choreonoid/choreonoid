/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_TIME_SYNC_ITEM_ENGINE_H
#define CNOID_BASE_TIME_SYNC_ITEM_ENGINE_H

#include <cnoid/Referenced>
#include <functional>
#include <string>
#include "exportdecl.h"

namespace cnoid {

class Item;

class CNOID_EXPORT TimeSyncItemEngine : public Referenced
{
public:
    virtual ~TimeSyncItemEngine();
    virtual bool onTimeChanged(double time);
    void notifyUpdate();
};

typedef ref_ptr<TimeSyncItemEngine> TimeSyncItemEnginePtr;


class CNOID_EXPORT TimeSyncItemEngineManager
{
public:
    static void initialize();
        
    TimeSyncItemEngineManager(const std::string& moduleName);
    ~TimeSyncItemEngineManager();
        
    void addEngineFactory(std::function<TimeSyncItemEngine*(Item* sourceItem)> factory);
        
private:
    std::string moduleName;
};

}

#endif
