/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_OPENRTM_PLUGIN_OPENHRP_CLOCK_GENERATOR_ITEM_H
#define CNOID_OPENRTM_PLUGIN_OPENHRP_CLOCK_GENERATOR_ITEM_H

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

    virtual bool start(ControllerItemIO* io);
    virtual double timeStep() const;
    virtual void input();
    virtual bool control();
    virtual void output();
    virtual void stop();

protected:
    virtual void onDisconnectedFromRoot();
    virtual Item* doDuplicate() const;
    virtual void doPutProperties(PutPropertyFunction& putProperty);
    virtual bool store(Archive& archive);
    virtual bool restore(const Archive& archive);

private:
    static OpenHRPClockGenerator_impl* clockGenerator;
    double timeStep_;
};
        
typedef ref_ptr<OpenHRPClockGeneratorItem> OpenHRPClockGeneratorItemPtr;
}

#endif
