/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_TIME_SYNC_ITEM_ENGINE_H
#define CNOID_BASE_TIME_SYNC_ITEM_ENGINE_H

#include "Item.h"
#include <boost/function.hpp>
#include "exportdecl.h"

namespace cnoid {

class AppImpl;

class CNOID_EXPORT TimeSyncItemEngine
{
public:
    virtual ~TimeSyncItemEngine();
    virtual bool onTimeChanged(double time);
    void notifyUpdate();
};
typedef boost::shared_ptr<TimeSyncItemEngine> TimeSyncItemEnginePtr;


class CNOID_EXPORT TimeSyncItemEngineManager
{
public:
    static void initialize();
        
    TimeSyncItemEngineManager(const std::string& moduleName);
    ~TimeSyncItemEngineManager();
        
    void addEngineFactory(boost::function<TimeSyncItemEnginePtr(Item* sourceItem)> factory);
        
private:
    std::string moduleName;
};

}

#endif
