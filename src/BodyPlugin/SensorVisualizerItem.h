#ifndef CNOID_BODY_PLUGIN_SENSOR_VISUALIZER_ITEM_H
#define CNOID_BODY_PLUGIN_SENSOR_VISUALIZER_ITEM_H

#include <cnoid/Item>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT SensorVisualizerItem : public Item
{
public:
    static void initializeClass(ExtensionManager* ext);

    SensorVisualizerItem();
    virtual ~SensorVisualizerItem();

protected:
    SensorVisualizerItem(const SensorVisualizerItem& org);
    virtual Item* doCloneItem(CloneMap* cloneMap) const override;
    virtual void onTreePathChanged() override;
    virtual void onDisconnectedFromRoot() override;
    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;

private:
    class Impl;
    Impl* impl;
};
        
typedef ref_ptr<SensorVisualizerItem> SensorVisualizerItemPtr;

}

#endif
