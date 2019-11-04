#ifndef CNOID_MANIPULATOR_PLUGIN_MANIPULATOR_CONTROLLR_ITEM_BASE_H
#define CNOID_MANIPULATOR_PLUGIN_MANIPULATOR_CONTROLLR_ITEM_BASE_H

#include <cnoid/ControllerItem>
#include "ManipulatorProgram.h"
#include <typeindex>
#include "exportdecl.h"

namespace cnoid {

class ManipulatorProgramItemBase;
class ManipulatorProgram;
class ManipulatorStatement;
class LinkKinematicsKit;
class ManipulatorVariableSet;

class CNOID_EXPORT ManipulatorControllerItemBase : public ControllerItem
{
public:
    ManipulatorControllerItemBase();
    ManipulatorControllerItemBase(const ManipulatorControllerItemBase& org);
    virtual ~ManipulatorControllerItemBase();

    virtual double timeStep() const override;
    double speedRatio() const;
        
protected:
    template<class StatementType>
    void registerStatementInterpreter(std::function<bool(StatementType* statement)> interpret){
        registerStatementInterpreter(
            typeid(StatementType),
            [interpret](ManipulatorStatement* statement){
                return interpret(static_cast<StatementType*>(statement)); });
    }
    void registerBaseStatementInterpreters();

    void setResidentInputFunction(std::function<void()> input);

    /**
        \note The ManipulatorProgram instance owned by the following item must not be
              used in the controller. The instance returned by the getManipulatorProgram
              function can be used instead of it.
    */
    ManipulatorProgramItemBase* getProgramItem();

    /**
       This function returns the program instance cloned by the controller
       for the purpose of control.
    */
    ManipulatorProgram* getMainProgram();

    ManipulatorProgram* getCurrentProgram();
    ManipulatorProgram::iterator getCurrentIterator();
    void setCurrent(ManipulatorProgram::iterator iter);
    void setCurrent(ManipulatorProgram* program, ManipulatorProgram::iterator iter);

    ManipulatorProgram* findProgram(const std::string& name);

    LinkKinematicsKit* getLinkKinematicsKit();
    ManipulatorVariableSet* getVariableSet();

    void pushControlFunctions(
        std::function<bool()> control, std::function<void()> input = nullptr, std::function<void()> output = nullptr);
        
    void pushOutputOnceFunction(std::function<void()> outputOnce);

    virtual bool onInitialize(ControllerIO* io);
    virtual bool onStart();
    virtual bool onStop();

    virtual void onDisconnectedFromRoot() override;
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;
    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;
        
private:
    void registerStatementInterpreter(
        std::type_index statementType, const std::function<bool(ManipulatorStatement* statement)>& interpret);
    
    virtual bool initialize(ControllerIO* io) override;
    virtual bool start() override;
    virtual void input() override;
    virtual bool control() override;
    virtual void output() override;
    virtual void stop() override;
    
    class Impl;
    Impl* impl;
};
        
typedef ref_ptr<ManipulatorControllerItemBase> ManipulatorControllerItemBasePtr;

}

#endif
