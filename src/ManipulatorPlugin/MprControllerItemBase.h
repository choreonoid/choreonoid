#ifndef CNOID_MANIPULATOR_PLUGIN_MPR_CONTROLLR_ITEM_BASE_H
#define CNOID_MANIPULATOR_PLUGIN_MPR_CONTROLLR_ITEM_BASE_H

#include <cnoid/ControllerItem>
#include "MprProgram.h"
#include "MprVariable.h"
#include <typeindex>
#include <memory>
#include "exportdecl.h"

namespace cnoid {

class MprProgramItemBase;
class MprProgram;
class MprStatement;
class KinematicBodyItemSet;
class KinematicBodySet;
class MprVariableSet;

class CNOID_EXPORT MprControllerItemBase : public ControllerItem
{
public:
    static void initializeClass(ExtensionManager* ext);
    
    MprControllerItemBase();
    MprControllerItemBase(const MprControllerItemBase& org);
    virtual ~MprControllerItemBase();

    /**
       This function should be defined in ControllerItem and the enabled state
       should be syncrhonized with the item check state.
    */
    bool isEnabled() const;
    void setEnabled(bool on);

    virtual KinematicBodyItemSet* kinematicBodyItemSet() = 0;
    
    virtual double timeStep() const override;
    double speedRatio() const;
    void setSpeedRatio(double r);

    void setActiveControlState(bool on);
    bool isActiveControlState() const;

    virtual bool initialize(ControllerIO* io) override final;
    virtual bool start() override final;
    virtual void input() override final;
    virtual bool control() override final;
    virtual void output() override final;
    virtual void stop() override final;

protected:
    template<class StatementType>
    void registerStatementInterpreter(std::function<bool(StatementType* statement)> interpret){
        registerStatementInterpreter(
            typeid(StatementType),
            [interpret](MprStatement* statement){
                return interpret(static_cast<StatementType*>(statement)); });
    }
    void registerBaseStatementInterpreters();

    void setResidentInputFunction(std::function<void()> input);

    /**
        \note The MprProgram instance owned by the following item must not be
              used in the controller. The instance returned by the getMprProgram
              function can be used instead of it.
    */
    MprProgramItemBase* getStartupProgramItem();

    /**
       This function returns the program instance cloned by the controller
       for the purpose of control.
    */
    MprProgram* getStartupProgram();

    MprProgram* getCurrentProgram();
    MprProgram::iterator getCurrentIterator();
    void setCurrent(MprProgram::iterator iter);
    void setCurrent(
        MprProgram* program, MprProgram::iterator iter, MprProgram::iterator upperNext);

    MprProgram* findProgram(const std::string& name);

    KinematicBodySet* kinematicBodySetForInternalUse();

    void pushControlFunctions(
        std::function<bool()> control, std::function<void()> input = nullptr, std::function<void()> output = nullptr);
        
    void pushOutputOnceFunction(std::function<void()> outputOnce);

    virtual bool onInitialize(ControllerIO* io);
    virtual bool onStart();
    virtual bool onStop();

    // Virtual functions for customizing variables
    virtual bool initializeVariables();
    void setVariableListSync(MprVariableList* listInGui, MprVariableList* listInController);
    virtual stdx::optional<MprVariable::Value> evalExpressionAsVariableValue(
        std::string::const_iterator& io_expressionBegin, std::string::const_iterator expressionEnd);
    virtual std::function<bool(MprVariable::Value value)> evalExpressionAsVariableToAssginValue(
        const std::string& expression);

    virtual void onDisconnectedFromRoot() override;
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;
    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;
        
private:
    void registerStatementInterpreter(
        std::type_index statementType, const std::function<bool(MprStatement* statement)>& interpret);
    
    class Impl;
    Impl* impl;
};

typedef ref_ptr<MprControllerItemBase> MprControllerItemBasePtr;


class CNOID_EXPORT MprControllerLog : public Referenced
{
public:
    MprControllerLog();
    const std::string& topLevelProgramName() const { return *topLevelProgramName_; }
    std::shared_ptr<const std::string> sharedTopLevelProgramName() const { return topLevelProgramName_; }
    const std::vector<int>& hierachicalPosition() const { return hierachicalPosition_; }
    bool isErrorState() const { return isErrorState_; }

private:
    std::shared_ptr<std::string> topLevelProgramName_;
    std::vector<int> hierachicalPosition_;
    bool isErrorState_;
    
    friend class MprControllerItemBase;
};

typedef ref_ptr<MprControllerLog> MprControllerLogPtr;
        
}

#endif
