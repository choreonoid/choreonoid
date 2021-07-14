/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_PLUGIN_CONTROLLER_ITEM_H
#define CNOID_BODY_PLUGIN_CONTROLLER_ITEM_H

#include "SimulatorItem.h"
#include <cnoid/ControllerIO>
#include "exportdecl.h"

namespace cnoid {

class ControllerIO;
class ControllerLogItem;

class Body;

class CNOID_EXPORT ControllerItem : public Item
{
public:
    static void initializeClass(ExtensionManager* ext);
    
    // for the backward compatibility
    typedef ControllerIO Target;

    ControllerItem();
    ControllerItem(const ControllerItem& org);
    virtual ~ControllerItem();

    bool isActive() const;
    bool isNoDelayMode() const { return isNoDelayMode_; }
    virtual void setNoDelayMode(bool on);
    
    const std::string& optionString() const { return optionString_; }
    void setOptions(const std::string& options) { optionString_ = options; }

    virtual double timeStep() const;

    virtual bool checkIfSubController(ControllerItem* controllerItem) const;

    /**
       This function is called before the simulation world is initialized.

       @note If the io->body() returns a null pointer, a controller is not associated with a particular body.
       This is for a controller which does some general operations.
           
       @note This function is called from the main thread.
    */
    virtual bool initialize(ControllerIO* io);

    virtual ControllerLogItem* createLogItem();
    
    /**
       This function is called after the simulation world is initialized.
    */
    virtual bool start();

    /**
       \note This function is called from the simulation thread.
    */
    virtual void input();

    /*
      @return false to request stopping
      @note The body oject given in the initalized function() must not be accessed
      in this function. The access should be done in input() and output().
      @note This function is called from the simulation thread.
    */
    virtual bool control();

    virtual void log();
        
    /**
       @note This function is called from the simulation thread.
    */
    virtual void output();
        
    /**
       @note This function is called from the main thread.
    */
    virtual void stop();

    //! \deprecated Use isNoDelayMode.
    bool isImmediateMode() const { return isNoDelayMode(); }
    //! \deprecated Use setsNoDelayMode.
    void setImmediateMode(bool on) { setNoDelayMode(on); }

protected:
    virtual void onOptionsChanged();
    
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;
    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;

private:
    SimulatorItemPtr simulatorItem_;
    bool isNoDelayMode_;
    std::string optionString_;

    friend class SimulatorItem;
    void setSimulatorItem(SimulatorItem* item);
};
        
typedef ref_ptr<ControllerItem> ControllerItemPtr;

}

#endif
