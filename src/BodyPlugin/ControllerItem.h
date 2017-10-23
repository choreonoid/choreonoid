/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_PLUGIN_CONTROLLER_ITEM_H
#define CNOID_BODY_PLUGIN_CONTROLLER_ITEM_H

#include "SimulatorItem.h"
#include "exportdecl.h"

namespace cnoid {

class Body;

class CNOID_EXPORT ControllerItemIO
{
public:
    virtual ~ControllerItemIO();
    virtual Body* body() = 0;
    virtual double timeStep() const = 0;
    virtual double currentTime() const = 0;
    virtual std::string optionString() const = 0;

    //! \deprecated Use timeStep().
    virtual double worldTimeStep() const;
};
    
class CNOID_EXPORT ControllerItem : public Item
{
public:
    // for the backward compatibility
    typedef ControllerItemIO Target;
        
    ControllerItem();
    ControllerItem(const ControllerItem& org);
    virtual ~ControllerItem();

    bool isActive() const;
    bool isImmediateMode() const { return isImmediateMode_; }
    void setImmediateMode(bool on);

    const std::string& optionString() const { return optionString_; }
    bool splitOptionString(const std::string& optionString, std::vector<std::string>& out_options) const;

    /**
       This function is called before the simulation world is initialized.

       @note If the body() of the target returns a null pointer, a controller is not associated with a particular body.
       This is for a controller which does some general operations.
           
       @note This function is called from the main thread.
    */
    virtual bool initialize(ControllerItemIO* io);
    
    /**
       This function is called after the simulation world is initialized.
    */
    virtual bool start();

    virtual double timeStep() const = 0;

    /**
       \note This function is called from the simulation thread.
    */
    virtual void input() = 0;

    /*
      @return false to request stopping
      @note The body oject given in the initalized function() must not be accessed
      in this function. The access should be done in input() and output().
      @note This function is called from the simulation thread.
    */
    virtual bool control() = 0;
        
    /**
       @note This function is called from the simulation thread.
    */
    virtual void output() = 0;
        
    /**
       @note This function is called from the main thread.
    */
    virtual void stop();

    SignalProxy<void(const std::string& message)> sigMessage();
    std::string getMessage();

#ifdef ENABLE_SIMULATION_PROFILING
    virtual void getProfilingNames(std::vector<std::string>& profilingNames);
    virtual void getProfilingTimes(std::vector<double>& profilingTimes);
#endif

protected:
    void putMessage(const std::string& message);

    virtual void doPutProperties(PutPropertyFunction& putProperty);
    virtual bool store(Archive& archive);
    virtual bool restore(const Archive& archive);

private:
    SimulatorItemPtr simulatorItem_;
    bool isImmediateMode_;
    std::string message_;
    Signal<void(const std::string& message)> sigMessage_;
    std::string optionString_;

    friend class SimulatorItemImpl;

    void setSimulatorItem(SimulatorItem* item) {
        simulatorItem_ = item;
    }
};
        
typedef ref_ptr<ControllerItem> ControllerItemPtr;

}

#endif
