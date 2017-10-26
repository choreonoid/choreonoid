/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_OPENHRP_PLUGIN_OPENHRP_CLOCK_GENERATOR_ITEM_H
#define CNOID_OPENHRP_PLUGIN_OPENHRP_CLOCK_GENERATOR_ITEM_H

#include <cnoid/ControllerItem>

namespace cnoid {

class ExtensionManager;
class OpenHRPClockGenerator_impl;

class OpenHRPClockGeneratorItem : public ControllerItem
{
public:
    static void initialize(ExtensionManager* ext);
        
    OpenHRPClockGeneratorItem();
    OpenHRPClockGeneratorItem(const OpenHRPClockGeneratorItem& org);
    virtual ~OpenHRPClockGeneratorItem();

    virtual bool initialize(ControllerIO* io) override;
    virtual double timeStep() const override;
    virtual void input() override;
    virtual bool control() override;
    virtual void output() override;
    virtual void stop() override;

protected:
    virtual void onDisconnectedFromRoot() override;
    virtual Item* doDuplicate() const override;
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;
    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;

private:
    static OpenHRPClockGenerator_impl* clockGenerator;
    double timeStep_;
};
        
typedef ref_ptr<OpenHRPClockGeneratorItem> OpenHRPClockGeneratorItemPtr;
}

#endif
