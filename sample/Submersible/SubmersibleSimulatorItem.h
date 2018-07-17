/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_SAMPLE_SUBMERSIBLE_SIMULATOR_ITEM_H
#define CNOID_SAMPLE_SUBMERSIBLE_SIMULATOR_ITEM_H

#include <cnoid/SubSimulatorItem>
#include <cnoid/BodyItem>

namespace cnoid {

class Light;

class SubmersibleSimulatorItem : public SubSimulatorItem
{
public:
    static void initializeClass(ExtensionManager* ext);
        
    SubmersibleSimulatorItem();
    SubmersibleSimulatorItem(const SubmersibleSimulatorItem& org);
    ~SubmersibleSimulatorItem();

    virtual bool initializeSimulation(SimulatorItem* simulatorItem);

protected:
    virtual Item* doDuplicate() const;
    virtual void doPutProperties(PutPropertyFunction& putProperty);
    virtual bool store(Archive& archive);
    virtual bool restore(const Archive& archive);

private:
    SimulatorItem* simulatorItem;
    Body* submersible;
    Light* light;
    bool prevLightButtonState;
    int joystickIntervalCounter;

    void initialize();
    void applyResistanceForce();
};

typedef ref_ptr<SubmersibleSimulatorItem> SubmersibleSimulatorItemPtr;

}

#endif
