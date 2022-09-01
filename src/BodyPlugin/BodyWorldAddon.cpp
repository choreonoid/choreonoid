#include "BodyWorldAddon.h"
#include <cnoid/ItemManager>
#include "WorldItem.h"
#include "SimulatorItem.h"

using namespace std;
using namespace cnoid;

namespace cnoid {

class BodyWorldAddon::Impl
{
public:
    WorldItem* worldItem;
    Signal<void(SimulatorItem* simulatorItem)> sigSimulationAboutToBeStarted_;
    ScopedConnection worldItemConnection;

    ItemList<SimulatorItem> simulatorItems;
    
    struct SimulatorItemInfo : public Referenced
    {
        ScopedConnection connection;
    };

    typedef map<SimulatorItem*, ref_ptr<SimulatorItemInfo>> SimulatorItemMap;
    SimulatorItemMap simulatorItemMap;

    void onSubTreeChanged();
};

}


void BodyWorldAddon::initializeClass(ExtensionManager* ext)
{
    ext->itemManager().registerAddon<BodyWorldAddon>("BodyWorld");
}


BodyWorldAddon::BodyWorldAddon()
{
    impl = new Impl;
    impl->worldItem = nullptr;
}


BodyWorldAddon::~BodyWorldAddon()
{
    delete impl;
}


ItemAddon* BodyWorldAddon::doClone(Item* /* newItem */, CloneMap* /* cloneMap */) const
{
    return nullptr;
}


bool BodyWorldAddon::setOwnerItem(Item* item)
{
    impl->worldItemConnection.disconnect();
    impl->worldItem = dynamic_cast<WorldItem*>(item);
    if(impl->worldItem){
        impl->worldItemConnection =
            impl->worldItem->sigSubTreeChanged().connect(
                [this](){ impl->onSubTreeChanged(); });
        impl->onSubTreeChanged();
    }
    return bool(impl->worldItem);
}


void BodyWorldAddon::Impl::onSubTreeChanged()
{
    auto newSimulatorItems = worldItem->descendantItems<SimulatorItem>();
    if(newSimulatorItems != simulatorItems){
        SimulatorItemMap newSimulatorItemMap;
        for(auto& item : newSimulatorItems){
            auto p = simulatorItemMap.find(item);
            if(p != simulatorItemMap.end()){
                newSimulatorItemMap[item] = p->second;
            } else {
                auto info = new SimulatorItemInfo;
                info->connection =
                    item->sigSimulationAboutToBeStarted().connect(
                        [this, item](){ sigSimulationAboutToBeStarted_(item); });
                newSimulatorItemMap[item] = info;
            }
        }
        simulatorItemMap.swap(newSimulatorItemMap);
        simulatorItems.swap(newSimulatorItems);
    }
}


SignalProxy<void(SimulatorItem* simulatorItem)> BodyWorldAddon::sigSimulationAboutToBeStarted()
{
    return impl->sigSimulationAboutToBeStarted_;
}
