#ifndef CNOID_BODY_PLUGIN_CONTROLLER_ITEM_H
#define CNOID_BODY_PLUGIN_CONTROLLER_ITEM_H

#include "SimulatorItem.h"
#include "BodyItem.h"
#include <cnoid/ControllerIO>
#include "exportdecl.h"

namespace cnoid {

class ControllerIO;
class ReferencedObjectSeqItem;

class CNOID_EXPORT ControllerItem : public Item
{
public:
    static void initializeClass(ExtensionManager* ext);
    
    // for the backward compatibility
    typedef ControllerIO Target;

    ControllerItem();
    ControllerItem(const ControllerItem& org);
    virtual ~ControllerItem();

    BodyItem* targetBodyItem() { return targetBodyItem_.lock(); }

    bool isActive() const;
    bool isNoDelayMode() const { return isNoDelayMode_; }
    virtual void setNoDelayMode(bool on);
    
    const std::string& optionString() const { return optionString_; }
    void setOptions(const std::string& options) { optionString_ = options; }

    virtual double timeStep() const;

    virtual bool checkIfSubController(ControllerItem* controllerItem) const;

    /**
       This function is called before the simulation world is initialized.

       \note If the io->body() returns a null pointer, a controller is not associated with a particular body.
       This is for a controller which does some general operations.
           
       \note This function is called from the main thread.
    */
    virtual bool initialize(ControllerIO* io);

    /**
       This function is called after the simulation world is initialized.
    */
    virtual bool start();

    /**
       \note This function is called from the simulation thread.
    */
    virtual void input();

    /*
      \return false to request stopping
      \note The body oject given in the initalized function() must not be accessed
      in this function. The access should be done in input() and output().
      \note This function is called from the simulation thread.
    */
    virtual bool control();

    /**
       \note This function is called from the simulation thread.
    */
    virtual void output();
        
    /**
       \note This function is called from the main thread.
    */
    virtual void stop();

    /**
       This function works with the enableLog and outputLogFrame functions of ControllerIO.
       The output log data is stored in the item created by this function.
       \note The logging form using this function and the above functions should be depcrated in the future.
    */
    virtual ReferencedObjectSeqItem* createLogItem();

    [[deprecated("Use isNoDelayMode.")]]
    bool isImmediateMode() const { return isNoDelayMode(); }
    [[deprecated("Use setNoDelayMode.")]]
    void setImmediateMode(bool on) { setNoDelayMode(on); }

    /**
       \note Controll items usually does not need access to the simulator item.
       This function is used only in special cases, such as when the controller controls multiple bodies.
       The use of the simulator item from a controller item requires knowlidge of the internal implementation
       and should not be used if you do not have such knowlidge.
    */
    SimulatorItem* simulatorItem() { return simulatorItem_.lock(); }

protected:
    virtual void onTargetBodyItemChanged(BodyItem* bodyItem);
    virtual void onOptionsChanged();
    
    virtual void onTreePathChanged() override;
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;
    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;

private:
    weak_ref_ptr<BodyItem> targetBodyItem_;
    weak_ref_ptr<SimulatorItem> simulatorItem_;
    std::string optionString_;
    bool isNoDelayMode_;

    friend class SimulatorItem;
    void setSimulatorItem(SimulatorItem* item);
};
        
typedef ref_ptr<ControllerItem> ControllerItemPtr;

}

#endif
