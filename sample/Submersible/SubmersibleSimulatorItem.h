/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_SAMPLE_SUBMERSIBLE_SIMULATOR_ITEM_H
#define CNOID_SAMPLE_SUBMERSIBLE_SIMULATOR_ITEM_H

#include <cnoid/SubSimulatorItem>
#include <cnoid/BodyItem>

namespace cnoid {

class CNOID_EXPORT SubmersibleSimulatorItem : public SubSimulatorItem
{
public:
    static void initializeClass(ExtensionManager* ext);
        
    SubmersibleSimulatorItem();
    SubmersibleSimulatorItem(const SubmersibleSimulatorItem& org);
    ~SubmersibleSimulatorItem();

    virtual bool isEnabled();
    virtual bool initializeSimulation(SimulatorItem* simulatorItem);

protected:
    virtual ItemPtr doDuplicate() const;
    virtual void onConnectedToRoot();
    virtual void doPutProperties(PutPropertyFunction& putProperty);
    virtual bool store(Archive& archive);
    virtual bool restore(const Archive& archive);

private:
    SimulatorItem* simulatorItem;
    Body* submersible;
    bool isEnabled_;
    int joystickIntervalCounter;

    void initialize();
    void applyResistanceForce();
};

typedef ref_ptr<SubmersibleSimulatorItem> SubmersibleSimulatorItemPtr;

}

#endif
