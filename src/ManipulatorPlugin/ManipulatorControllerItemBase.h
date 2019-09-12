#ifndef CNOID_MANIPULATOR_PLUGIN_MANIPULATOR_CONTROLLR_ITEM_BASE_H
#define CNOID_MANIPULATOR_PLUGIN_MANIPULATOR_CONTROLLR_ITEM_BASE_H

#include <cnoid/ControllerItem>
#include "exportdecl.h"

namespace cnoid {

class ManipulatorProgramItemBase;
class ManipulatorProgram;
class BodyManipulatorManager;

class CNOID_EXPORT ManipulatorControllerItemBase : public ControllerItem
{
public:
    ManipulatorControllerItemBase();
    ManipulatorControllerItemBase(const ManipulatorControllerItemBase& org);
    virtual ~ManipulatorControllerItemBase();

    double speedRatio() const;
        
protected:
    bool initializeManipulatorProgram(ControllerIO* io);

    /**
        \note The ManipulatorProgram instance owned by the following item must not be
              used in the controller. The instance returned by the getManipulatorProgram
              function can be used instead of it.
    */
    ManipulatorProgramItemBase* getManipulatorProgramItem();

    /**
       This function returns the program instance cloned by the controller
       for the purpose of control.
    */
    ManipulatorProgram* getManipulatorProgram();

    BodyManipulatorManager* getBodyManipulatorManager();

    void finalizeManipulatorProgram();
    
    virtual void onDisconnectedFromRoot() override;
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;
    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;
        
private:
    class Impl;
    Impl* impl;
};
        
typedef ref_ptr<ManipulatorControllerItemBase> ManipulatorControllerItemBasePtr;

}

#endif
