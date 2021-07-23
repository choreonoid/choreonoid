#ifndef CNOID_BODY_PLUGIN_BODY_WORLD_ADDON_H
#define CNOID_BODY_PLUGIN_BODY_WORLD_ADDON_H

#include <cnoid/ItemAddon>
#include <cnoid/Signal>
#include "exportdecl.h"

namespace cnoid {

class WorldItem;
class SimulatorItem;

class CNOID_EXPORT BodyWorldAddon : public ItemAddon
{
public:
    static void initializeClass(ExtensionManager* ext);

    BodyWorldAddon();
    BodyWorldAddon(const BodyWorldAddon&) = delete;
    ~BodyWorldAddon();

    virtual bool setOwnerItem(Item* item) override;

    SignalProxy<void(SimulatorItem* simulatorItem)> sigSimulationAboutToBeStarted();

private:
    class Impl;
    Impl* impl;
};

}

#endif
