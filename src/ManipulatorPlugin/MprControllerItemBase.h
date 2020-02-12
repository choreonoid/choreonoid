#ifndef CNOID_MANIPULATOR_PLUGIN_MPR_CONTROLLR_ITEM_BASE_H
#define CNOID_MANIPULATOR_PLUGIN_MPR_CONTROLLR_ITEM_BASE_H

#include <cnoid/ControllerItem>
#include "MprProgram.h"
#include <typeindex>
#include <memory>
#include "exportdecl.h"

namespace cnoid {

class MprProgramItemBase;
class MprProgram;
class MprStatement;
class LinkKinematicsKit;
class MprVariableSet;

class CNOID_EXPORT MprControllerItemBase : public ControllerItem
{
public:
    static void initializeClass(ExtensionManager* ext);
    
    MprControllerItemBase();
    MprControllerItemBase(const MprControllerItemBase& org);
    virtual ~MprControllerItemBase();

    virtual double timeStep() const override;
    double speedRatio() const;
        
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

    LinkKinematicsKit* getLinkKinematicsKit();
    MprVariableSet* getVariableSet();

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
        std::type_index statementType, const std::function<bool(MprStatement* statement)>& interpret);
    
    virtual bool initialize(ControllerIO* io) override;
    virtual bool start() override;
    virtual void input() override;
    virtual bool control() override;
    virtual void output() override;
    virtual void stop() override;
    
    class Impl;
    Impl* impl;
};

typedef ref_ptr<MprControllerItemBase> MprControllerItemBasePtr;


class CNOID_EXPORT MprControllerLog : public Referenced
{
public:
    std::shared_ptr<std::string> topLevelProgramName;
    std::vector<int> hierachicalPosition;
};

typedef ref_ptr<MprControllerLog> MprControllerLogPtr;
        
}

#endif
