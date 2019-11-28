/**
   @author Shin'ichiro Nakaoka
*/

#include "TimeSyncItemEngine.h"
#include "RootItem.h"
#include "ItemList.h"
#include "TimeBar.h"
#include "LazyCaller.h"
#include <vector>
#include <map>

using namespace std;
using namespace cnoid;

namespace {

double currentTime;

typedef std::function<TimeSyncItemEnginePtr(Item* sourceItem)> TimeSyncItemEngineFactory;

typedef vector<TimeSyncItemEngineFactory> FactoryArray;
typedef map<string, FactoryArray> FactoryArrayMap;
FactoryArrayMap allFactories;

vector<TimeSyncItemEnginePtr> engines;
Connection selectionConnection;
Connection connectionOfTimeChanged;

LazyCaller updateLater;

bool setTime(double time)
{
    bool isActive = false;

    currentTime = time;

    for(size_t i=0; i < engines.size(); ++i){
        isActive |= engines[i]->onTimeChanged(time);
    }

    return isActive;
}

void update() {
    setTime(currentTime);
}

void onSelectedItemsChanged(const ItemList<>& selectedItems)
{
    engines.clear();

    for(size_t i=0; i < selectedItems.size(); ++i){ 
        Item* sourceItem = selectedItems.get(i);
        for(FactoryArrayMap::iterator p = allFactories.begin(); p != allFactories.end(); ++p){
            FactoryArray& factories = p->second;
            for(FactoryArray::iterator q = factories.begin(); q != factories.end(); ++q){
                TimeSyncItemEngineFactory& factoryFunc = *q;
                TimeSyncItemEnginePtr engine = factoryFunc(sourceItem);
                if(engine){
                    engines.push_back(engine);
                }
            }
        }
    }

    setTime(currentTime);
}

}


TimeSyncItemEngine::~TimeSyncItemEngine()
{

}


bool TimeSyncItemEngine::onTimeChanged(double time)
{
    return false;
}


void TimeSyncItemEngine::notifyUpdate()
{
    updateLater();
}


void TimeSyncItemEngineManager::initialize()
{
    static bool initialized = false;
    
    if(!initialized){
        
        currentTime = 0.0;
        
        selectionConnection =
            RootItem::instance()->sigSelectedItemsChanged().connect(onSelectedItemsChanged);
        
        connectionOfTimeChanged = TimeBar::instance()->sigTimeChanged().connect(setTime);
        
        updateLater.setFunction(update);
        updateLater.setPriority(LazyCaller::PRIORITY_LOW);

        initialized = true;
    }
}


TimeSyncItemEngineManager::TimeSyncItemEngineManager(const std::string& moduleName)
    : moduleName(moduleName)
{

}


TimeSyncItemEngineManager::~TimeSyncItemEngineManager()
{
    allFactories.erase(moduleName);
    engines.clear();
}


void TimeSyncItemEngineManager::addEngineFactory(std::function<TimeSyncItemEngine*(Item* sourceItem)> factory)
{
    allFactories[moduleName].push_back(factory);
}
