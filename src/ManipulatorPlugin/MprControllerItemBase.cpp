#include "MprControllerItemBase.h"
#include "MprProgramItemBase.h"
#include "MprBasicStatements.h"
#include "MprVariableList.h"
#include "MprMultiVariableListItem.h"
#include <cnoid/ItemManager>
#include <cnoid/LinkKinematicsKit>
#include <cnoid/DigitalIoDevice>
#include <cnoid/ItemList>
#include <cnoid/BodyItem>
#include <cnoid/ControllerIO>
#include <cnoid/MessageView>
#include <cnoid/CloneMap>
#include <cnoid/LazyCaller>
#include <cnoid/PutPropertyFunction>
#include <cnoid/Archive>
#include <fmt/format.h>
#include <unordered_map>
#include <regex>
#include <cctype>
#include <algorithm>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

namespace {

class Processor : public Referenced
{
public:
    virtual void input() { }
    virtual bool control() = 0;
    virtual void output() { }
};

class ControlFunctionSetProcessor : public Processor
{
public:
    function<bool()> control_;
    function<void()> input_;
    function<void()> output_;

    ControlFunctionSetProcessor(
        function<bool()> control, function<void()> input, function<void()> output)
        : control_(control), input_(input), output_(output)
    { }

    virtual void input() override { if(input_) input_(); }
    virtual bool control() override { return control_(); }
    virtual void output() override { if(output_) output_(); }
};

class OutputOnceFunctionProcessor : public Processor
{
public:
    function<void()> output_;

    OutputOnceFunctionProcessor(function<void()> output): output_(output) { }

    virtual bool control() override {
        return output_ != nullptr;
    }
    virtual void output() override {
        if(output_){
            output_();
            output_ = nullptr;
        }
    }
};

}
    
    
namespace cnoid {

class MprControllerItemBase::Impl
{
public:
    MprControllerItemBase* self;
    ControllerIO* io;
    MprProgramItemBasePtr startupProgramItem;
    MprProgramPtr startupProgram;
    MprProgramPtr currentProgram;
    unordered_map<string, MprProgramPtr> otherProgramMap;
    CloneMap cloneMap;
    LinkKinematicsKitPtr kinematicsKit;

    // Used for the default variable mappings
    vector<MprVariableListPtr> variableLists;

    bool isControlActive;
    typedef MprProgram::iterator Iterator;
    Iterator iterator;

    struct ProgramPosition {
        MprProgramPtr program;
        Iterator current;
        Iterator next;
        vector<int> hierachicalPosition;
    };
    vector<ProgramPosition> programStack;
    
    vector<ref_ptr<Processor>> processorStack;
    
    vector<function<void()>> residentInputFunctions;

    typedef function<bool(MprStatement* statement)> InterpretFunction;
    unordered_map<type_index, InterpretFunction> interpreterMap;
    DigitalIoDevicePtr ioDevice;
    double speedRatio;

    MprControllerLogPtr currentLog;
    unordered_map<MprProgramPtr, shared_ptr<string>> topLevelProgramToSharedNameMap;
    bool isLogEnabled;

    regex termPattern;
    regex operatorPattern;
    regex cmpOperatorPattern;
    regex intPattern;
    regex floatPattern;
    regex boolPattern;
    regex stringPattern;
    regex variablePattern; // for the default variable expression syntax

    Impl(MprControllerItemBase* self);
    bool initialize(ControllerIO* io);
    bool createKinematicsKitForControl();

    // Default variable mappings
    bool initializeDefaultVariableMappings();
    void extractProgramLocalVariables(Item* item, vector<MprVariableListPtr>& variableLists);
    void extractControllerGlobalVariables(Item* item, vector<MprVariableListPtr>& variableLists);
    void addVariableList(vector<MprVariableListPtr>& variableLists, MprMultiVariableListItem* listItem);
    void notifyGuiOfVariableUpdate(
        MprVariableList* listInController, MprVariableList* listInGui, int variableIndex);
    void updateVariableListInGui(MprVariableList* listInGui, MprVariable* variableForNotification);
    MprVariable* findVariable(const GeneralId& id);

    void setCurrent(MprProgram* program, MprProgram::iterator iter, MprProgram::iterator upperNext);
    bool control();
    void setCurrentProgramPositionToLog(MprControllerLog* log);
    void clear();
    bool interpretCommentStatement(MprCommentStatement* statement);
    bool interpretGroupStatement(MprGroupStatement* statement);
    bool interpretIfStatement(MprIfStatement* statement);
    bool interpretWhileStatement(MprWhileStatement* statement);
    bool interpretCallStatement(MprCallStatement* statement);
    stdx::optional<bool> evalConditionalExpression(const string& expression);
    stdx::optional<MprVariable::Value> getTermValue(string::const_iterator& pos, string::const_iterator end);
    stdx::optional<string> getComparisonOperator(string::const_iterator& pos, string::const_iterator end);
    bool interpretAssignStatement(MprAssignStatement* statement);
    bool applyBinaryOperation(MprVariable::Value& lhsValue, char op, const MprVariable::Value& rhsValue);
    bool interpretSetSignalStatement(MprSignalStatement* statement);
    bool interpretWaitStatement(MprWaitStatement* statement);
    bool interpretDelayStatement(MprDelayStatement* statement);
};

}


void MprControllerItemBase::initializeClass(ExtensionManager* ext)
{
    ext->itemManager().registerAbstractClass<MprControllerItemBase, ControllerItem>();
}



MprControllerItemBase::MprControllerItemBase()
{
    impl = new Impl(this);
}


MprControllerItemBase::MprControllerItemBase(const MprControllerItemBase& org)
    : ControllerItem(org)
{
    impl = new Impl(this);
}


MprControllerItemBase::Impl::Impl(MprControllerItemBase* self)
    : self(self)
{
    isControlActive = false;
    speedRatio = 1.0;
    currentLog = new MprControllerLog;
}


MprControllerItemBase::~MprControllerItemBase()
{
    delete impl;
}


void MprControllerItemBase::registerStatementInterpreter
(std::type_index statementType, const std::function<bool(MprStatement* statement)>& interpret)
{
    impl->interpreterMap[statementType] = interpret;
}


void MprControllerItemBase::registerBaseStatementInterpreters()
{
    MprControllerItemBase::Impl* impl_ = impl;

    registerStatementInterpreter<MprCommentStatement>(
        [impl_](MprCommentStatement* statement){
            return impl_->interpretCommentStatement(statement); });

    registerStatementInterpreter<MprGroupStatement>(
        [impl_](MprGroupStatement* statement){
            return impl_->interpretGroupStatement(statement); });

    registerStatementInterpreter<MprIfStatement>(
        [impl_](MprIfStatement* statement){
            return impl_->interpretIfStatement(statement); });

    registerStatementInterpreter<MprWhileStatement>(
        [impl_](MprWhileStatement* statement){
            return impl_->interpretWhileStatement(statement); });
 
    registerStatementInterpreter<MprCallStatement>(
        [impl_](MprCallStatement* statement){
            return impl_->interpretCallStatement(statement); });

    registerStatementInterpreter<MprAssignStatement>(
        [impl_](MprAssignStatement* statement){
            return impl_->interpretAssignStatement(statement); });

    registerStatementInterpreter<MprSignalStatement>(
        [impl_](MprSignalStatement* statement){
            return impl_->interpretSetSignalStatement(statement); });

    registerStatementInterpreter<MprWaitStatement>(
        [impl_](MprWaitStatement* statement){
            return impl_->interpretWaitStatement(statement); });
    
    registerStatementInterpreter<MprDelayStatement>(
        [impl_](MprDelayStatement* statement){
            return impl_->interpretDelayStatement(statement); });

    impl->termPattern.assign("^\\s*(.+)\\s*");
    impl->operatorPattern.assign("^\\s*([+-])\\s*");
    impl->cmpOperatorPattern.assign("^\\s*(=|==|!=|<|>|<=|>=)\\s*");
    impl->intPattern.assign("^[+-]?\\d+");
    impl->floatPattern.assign("^[+-]?(\\d+\\.\\d*|\\.\\d+)");
    impl->boolPattern.assign("^([Tt][Rr][Uu][Ee]|[Ff][Aa][Ll][Ss][Ee])");
    impl->stringPattern.assign("^\"(.*)\"");
    impl->variablePattern.assign("^var\\[(\\d+)\\]");
}


void MprControllerItemBase::setResidentInputFunction(std::function<void()> input)
{
    impl->residentInputFunctions.push_back(input);
}


double MprControllerItemBase::timeStep() const
{
    return impl->io ? impl->io->timeStep() : 0.0;
}
        

bool MprControllerItemBase::initialize(ControllerIO* io)
{
    impl->isControlActive = false;
    if(impl->initialize(io)){
        if(onInitialize(io)){
            impl->isControlActive = true;
        }
    }
    return impl->isControlActive;
}    


bool MprControllerItemBase::Impl::initialize(ControllerIO* io)
{
    this->io = io;
    
    auto mv = MessageView::instance();

    clear();
    
    auto programItems = self->descendantItems<MprProgramItemBase>();
    if(programItems.empty()){
        mv->putln(format(_("Any program item for {} is not found."),
                         self->displayName()), MessageView::Error);
        return false;
    }
    
    startupProgramItem.reset();
    for(auto& programItem : programItems){
        if(!programItem->resolveAllReferences()){
            mv->putln(format(_("Program \"{0}\" is incomplete due to unresolved references."),
                             programItem->displayName()), MessageView::Warning);
        }
        if(programItem->isStartupProgram()){
            startupProgramItem = programItem;
        }
    }
    if(!startupProgramItem){
        mv->putln(format(_("The startup program for {0} is not specified."),
                         self->displayName()), MessageView::Error);
        return false;
    }
    startupProgram = cloneMap.getClone(startupProgramItem->program());
    currentProgram = startupProgram;

    if(!createKinematicsKitForControl()){
        return false;
    }

    for(auto& item : programItems){
        if(item != startupProgramItem){
            auto program = cloneMap.getClone(item->program());
            otherProgramMap.insert(make_pair(item->name(), program));
        }
    }
    
    if(!self->initializeVariables()){
        mv->putln(format(_("Variables for {} cannot be initialized."), self->displayName()),
                  MessageView::Error);
        return false;
    }

    iterator = currentProgram->begin();

    auto body = io->body();
    ioDevice = body->findDevice<DigitalIoDevice>();

    if(ioDevice){
        // Reset all the signals
        const int n = ioDevice->numSignalLines();
        for(int i=0; i < n; ++i){
            ioDevice->setOut(i, false, false);
            ioDevice->setIn(i, false, false);
        }
    }

    isLogEnabled = io->enableLog();
    
    return true;
}


bool MprControllerItemBase::Impl::createKinematicsKitForControl()
{
    kinematicsKit = cloneMap.getClone(startupProgramItem->kinematicsKit());
    if(!kinematicsKit->jointPath()){
        return false;
    }

    if(kinematicsKit->isCustomIkDisabled()){
        io->os() << format(_("Warning: The custom inverse kinematics is disabled in controller \"{0}\" for robot \"{1}\"."),
                           self->displayName(), io->body()->name())
                 << endl;
    }

    return true;
}


bool MprControllerItemBase::initializeVariables()
{
    return impl->initializeDefaultVariableMappings();
}


bool MprControllerItemBase::Impl::initializeDefaultVariableMappings()
{
    variableLists.clear();
    
    extractProgramLocalVariables(startupProgramItem, variableLists);
    extractControllerGlobalVariables(self, variableLists);
    
    if(variableLists.empty()){
        variableLists.push_back(new MprVariableList);
    }

    return true;
}


void MprControllerItemBase::Impl::extractProgramLocalVariables
(Item* item, vector<MprVariableListPtr>& variableLists)
{
    for(Item* child = item->childItem(); child; child = child->nextItem()){
        if(auto listItem = dynamic_cast<MprMultiVariableListItem*>(child)){
            addVariableList(variableLists, listItem);
        }
        extractProgramLocalVariables(child, variableLists);
    }
}


void MprControllerItemBase::Impl::extractControllerGlobalVariables
(Item* item, vector<MprVariableListPtr>& variableLists)
{
    for(Item* child = item->childItem(); child; child = child->nextItem()){
        if(auto programItem = dynamic_cast<MprProgramItemBase*>(child)){
            continue;
        }
        if(auto listItem = dynamic_cast<MprMultiVariableListItem*>(child)){
            addVariableList(variableLists, listItem);
        }
        extractControllerGlobalVariables(child, variableLists);
    }
}


void MprControllerItemBase::Impl::addVariableList
(vector<MprVariableListPtr>& variableLists, MprMultiVariableListItem* listItem)
{
    MprVariableListPtr listInGui = listItem->findVariableList(MprVariableList::GeneralVariable);
    if(listInGui){
        auto listInController = listInGui->clone();
        self->setVariableListSync(listInGui, listInController);
        variableLists.push_back(listInController);
    }
}


void MprControllerItemBase::setVariableListSync
(MprVariableList* listInGui, MprVariableList* listInController)
{
    listInController->sigVariableAdded().connect(
        [this, listInController, listInGui](int index){
            impl->notifyGuiOfVariableUpdate(listInController, listInGui, index); });

    listInController->sigVariableUpdated().connect(
        [this, listInController, listInGui](int index, int flags){
            if(flags & MprVariable::ValueUpdate){
                impl->notifyGuiOfVariableUpdate(listInController, listInGui, index);
            }
        });
}


void MprControllerItemBase::Impl::notifyGuiOfVariableUpdate
(MprVariableList* listInController, MprVariableList* listInGui, int variableIndex)
{
    auto variableInController = listInController->variableAt(variableIndex);
    MprVariablePtr variableForNotification = new MprVariable(*variableInController);
    callLater(
        [this, listInGui, variableForNotification](){
            updateVariableListInGui(listInGui, variableForNotification); });
}
    

/**
   This is a temporary implementation to update the original variables managed in the main GUI thread.
   The variables should be updated with the simulation log data which records the variable states for
   each time frame.
*/
void MprControllerItemBase::Impl::updateVariableListInGui
(MprVariableList* listInGui, MprVariable* variableForNotification)
{
    auto variable = listInGui->findVariable(variableForNotification->id());
    if(!variable){
        listInGui->append(variableForNotification);
    } else {
        variable->setValue(variableForNotification->value());
        variable->notifyUpdate(MprVariable::ValueUpdate);
    }
}


MprVariable* MprControllerItemBase::Impl::findVariable(const GeneralId& id)
{
    for(auto& list : variableLists){
        if(auto variable = list->findVariable(id)){
            return variable;
        }
    }
    return nullptr;
}


stdx::optional<MprVariable::Value> MprControllerItemBase::evalExpressionAsVariableValue
(std::string::const_iterator& io_expressionBegin, std::string::const_iterator expressionEnd)
{
    std::smatch match;
    if(regex_search(io_expressionBegin, expressionEnd, match, impl->variablePattern)){
        io_expressionBegin = match[0].second;
        GeneralId id(std::stoi(match.str(1)));
        if(auto variable = impl->findVariable(id)){
            return variable->value();
        }
        impl->io->os() << format(_("Variable {0} is not defined."), id.label()) << endl;
    }
    return stdx::nullopt;
}


std::function<bool(MprVariable::Value value)> MprControllerItemBase::evalExpressionAsVariableToAssginValue
(const std::string& expression)
{
    std::smatch match;
    if(regex_search(expression, match, impl->variablePattern)){
        GeneralId id(std::stoi(match.str(1)));
        auto variable = impl->findVariable(id);
        if(!variable){
            auto list = impl->variableLists.front();
            MprVariablePtr newVariable = new MprVariable(id, list->defaultValue());
            if(list->append(newVariable)){
                variable = newVariable;
            }
        }
        if(variable){
            return
                [this, variable](MprVariable::Value value){
                    if(variable->setValue(value)){
                        pushOutputOnceFunction([variable](){ variable->notifyUpdate(); });
                        return true;
                    }
                    return false;
                };
        }
    }
    return nullptr;
}
    
    
bool MprControllerItemBase::onInitialize(ControllerIO* /* io */)
{
    return true;
}


MprProgramItemBase* MprControllerItemBase::getStartupProgramItem()
{
    return impl->startupProgramItem;
}


MprProgram* MprControllerItemBase::getStartupProgram()
{
    return impl->startupProgram;
}


MprProgram* MprControllerItemBase::getCurrentProgram()
{
    return impl->currentProgram;
}


MprProgram::iterator MprControllerItemBase::getCurrentIterator()
{
    return impl->iterator;
}


void MprControllerItemBase::setCurrent(MprProgram::iterator iter)
{
    impl->iterator = iter;
}


void MprControllerItemBase::setCurrent
(MprProgram* program, MprProgram::iterator iter, MprProgram::iterator upperNext)
{
    impl->setCurrent(program, iter, upperNext);
}


void MprControllerItemBase::Impl::setCurrent
(MprProgram* program, MprProgram::iterator iter, MprProgram::iterator upperNext)
{
    ProgramPosition upper;
    upper.program = currentProgram;
    upper.current = iterator;
    upper.next = upperNext;

    if(!programStack.empty()){
        upper.hierachicalPosition = programStack.back().hierachicalPosition;
    }
    int statementIndex = upper.current - currentProgram->begin();
    upper.hierachicalPosition.push_back(statementIndex);

    programStack.push_back(upper);
    
    currentProgram = program;
    iterator = iter;
}


MprProgram* MprControllerItemBase::findProgram(const std::string& name)
{
    auto iter = impl->otherProgramMap.find(name);
    if(iter != impl->otherProgramMap.end()){
        return iter->second;
    }
    return nullptr;
}


LinkKinematicsKit* MprControllerItemBase::linkKinematicsKitForControl()
{
    return impl->kinematicsKit;
}


bool MprControllerItemBase::start()
{
    impl->currentLog->isErrorState_ = false;
    return onStart();
}


bool MprControllerItemBase::onStart()
{
    return true;
}


void MprControllerItemBase::input()
{
    for(auto& input : impl->residentInputFunctions){
        input();
    }
    if(!impl->processorStack.empty()){
        impl->processorStack.back()->input();
    }
}


bool MprControllerItemBase::control()
{
    return impl->control();
}


bool MprControllerItemBase::Impl::control()
{
    if(!isControlActive){
        return false;
    }

    int loopCounter = 0;
    bool stateChanged = false;

    while(true){
        bool isProcessorActive = false;
        while(!processorStack.empty()){
            isProcessorActive = processorStack.back()->control();
            if(isProcessorActive){
                break;
            }
            processorStack.pop_back();
        }
        if(isProcessorActive){
            break;
        }

        if(loopCounter > 10){
            break;
        }

        bool hasNextStatement = (iterator != currentProgram->end());
        
        while(!hasNextStatement){
            if(programStack.empty()){
                break;
            }
            auto& upper = programStack.back();
            iterator = upper.next;
            currentProgram = upper.program;
            programStack.pop_back();
            hasNextStatement = (iterator != currentProgram->end());
        }
        if(!hasNextStatement){
            isControlActive = false;
            break;
        }

        if(isLogEnabled){
            setCurrentProgramPositionToLog(currentLog);
            stateChanged = true;
        }
        
        auto statement = iterator->get();

        auto p = interpreterMap.find(typeid(*statement));
        if(p == interpreterMap.end()){
            io->os() << format(_("{0} cannot be executed because the interpreter for it is not found."),
                               statement->label(0)) << endl;
            ++iterator;
        } else {
            auto& interpret = p->second;
            if(!interpret(statement)){
                isControlActive = false;
                if(isLogEnabled){
                    currentLog->isErrorState_ = true;
                }
                io->os() << format(_("Failed to execute {0} statement. The control was terminated."),
                                   statement->label(0)) << endl;
                break;
            }
            isControlActive = true;
        }
        ++loopCounter;
    }

    if(isLogEnabled && stateChanged){
        io->outputLog(new MprControllerLog(*currentLog));
    }
        
    return isControlActive;
}


void MprControllerItemBase::Impl::setCurrentProgramPositionToLog(MprControllerLog* log)
{
    auto topLevelProgram = currentProgram->topLevelProgram();
    auto it = topLevelProgramToSharedNameMap.find(topLevelProgram);
    if(it != topLevelProgramToSharedNameMap.end()){
        log->topLevelProgramName_ = it->second;
    } else {
        log->topLevelProgramName_ = make_shared<string>(topLevelProgram->name());
        topLevelProgramToSharedNameMap[topLevelProgram] = log->topLevelProgramName_;
    }
        
    if(currentProgram->isTopLevelProgram()){
        log->hierachicalPosition_.clear();
    } else if(!programStack.empty()){
        log->hierachicalPosition_ = programStack.back().hierachicalPosition;
    }
    int statementIndex = iterator - currentProgram->begin();
    log->hierachicalPosition_.push_back(statementIndex);
}


void MprControllerItemBase::pushControlFunctions
(std::function<bool()> control, std::function<void()> input, std::function<void()> output)
{
    impl->processorStack.push_back(new ControlFunctionSetProcessor(control, input, output));
}
        

void MprControllerItemBase::pushOutputOnceFunction(std::function<void()> outputOnce)
{
    impl->processorStack.push_back(new OutputOnceFunctionProcessor(outputOnce));
}


void MprControllerItemBase::output()
{
    if(!impl->processorStack.empty()){
        impl->processorStack.back()->output();
    }
}


void MprControllerItemBase::stop()
{
    impl->clear();
    onStop();
}


bool MprControllerItemBase::onStop()
{
    return true;
}


void MprControllerItemBase::onDisconnectedFromRoot()
{
    impl->clear();
}


void MprControllerItemBase::Impl::clear()
{
    startupProgramItem.reset();
    startupProgram.reset();
    currentProgram.reset();
    otherProgramMap.clear();
    cloneMap.clear();
    kinematicsKit.reset();
    variableLists.clear();
    programStack.clear();
    processorStack.clear();
    topLevelProgramToSharedNameMap.clear();
}


double MprControllerItemBase::speedRatio() const
{
    return impl->speedRatio;
}


void MprControllerItemBase::setSpeedRatio(double r)
{
    if(r > 0.0 && r <= 1.0){
        impl->speedRatio = r;
    }
}


bool MprControllerItemBase::Impl::interpretCommentStatement(MprCommentStatement*)
{
    ++iterator;
    return true;
}


bool MprControllerItemBase::Impl::interpretGroupStatement(MprGroupStatement* statement)
{
    auto program = statement->lowerLevelProgram();
    self->setCurrent(program, program->begin(), ++self->getCurrentIterator());
    return true;
}


bool MprControllerItemBase::Impl::interpretIfStatement(MprIfStatement* statement)
{
    auto condition = evalConditionalExpression(statement->condition());

    if(!condition){
        return false;
    }

    auto next = iterator;
    ++next;
                           
    MprElseStatement* nextElseStatement = nullptr;
    if(next != currentProgram->end()){
        nextElseStatement = dynamic_cast<MprElseStatement*>(next->get());
        if(nextElseStatement){
            ++next;
        }
    }

    if(*condition){
        auto program = statement->lowerLevelProgram();
        setCurrent(program, program->begin(), next);

    } else {
        ++iterator;
        if(nextElseStatement){
            auto program = nextElseStatement->lowerLevelProgram();
            setCurrent(program, program->begin(), next);
        }
    }

    return true;
}


bool MprControllerItemBase::Impl::interpretWhileStatement(MprWhileStatement* statement)
{
    auto condition = evalConditionalExpression(statement->condition());

    if(!condition){
        return false;
    }

    if(*condition){
        auto program = statement->lowerLevelProgram();
        setCurrent(program, program->begin(), iterator);
    } else {
        ++iterator;
    }
    return true;
}


bool MprControllerItemBase::Impl::interpretCallStatement(MprCallStatement* statement)
{
    auto& programName = statement->programName();

    if(programName.empty()){
        io->os() << _("A program specified in a call statement is empty.") << endl;
        return false;
    }
        
    auto program = self->findProgram(programName);

    if(!program){
        io->os() << format(_("Program \"{0}\" specified in a call statement does not exist."),
                           programName) << endl;
        return false;
    }

    setCurrent(program, program->begin(), iterator + 1);

    return true;
}


template<class LhsType, class RhsType>
static stdx::optional<bool> checkNumericalComparison(const string& op, LhsType lhs, RhsType rhs)
{
    if(op == "="){
        return lhs == rhs;
    } else if(op == "=="){
        return lhs == rhs;
    } else if(op == "!="){
        return lhs != rhs;
    } else if(op == "<"){
        return lhs < rhs;
    } else if(op == ">"){
        return lhs > rhs;
    } else if(op == "<="){
        return lhs <= rhs;
    } else if(op == ">="){
        return lhs >= rhs;
    }
    return stdx::nullopt;
}


stdx::optional<bool> MprControllerItemBase::Impl::evalConditionalExpression(const string& expression)
{
    if(expression.empty()){
        io->os() << _("Empty conditional expression.") << endl;
        return stdx::nullopt;
    }
    auto pos = expression.cbegin();
    auto end = expression.cend();

    bool isExpressionValid = false;
    stdx::optional<string> pCmpOp;
    stdx::optional<MprVariable::Value> pRhs;
    stdx::optional<MprVariable::Value> pLhs = getTermValue(pos, end);
    if(pLhs){
        if(pos == end){
            isExpressionValid = true;
        } else if((pCmpOp = getComparisonOperator(pos, end))){
            if(pos != end){
                if((pRhs = getTermValue(pos, end))){
                    if(pos == end){
                        isExpressionValid = true;
                    }
                }
            }
        }
    }
    if(!isExpressionValid){
        io->os() << format(_("Conditional expression \"{0}\" is invalid."), expression) << endl;
        return stdx::nullopt;
    }

    auto& lhs = *pLhs;

    if(!pRhs){
        return MprVariable::toBool(lhs);
    }
            
    stdx::optional<bool> pResult;
    auto& rhs = *pRhs;
    string& cmpOp = *pCmpOp;

    int rhsValueType = MprVariable::valueType(rhs);
    switch(MprVariable::valueType(lhs)){
    case MprVariable::Int:
    {
        auto lhsValue = MprVariable::intValue(lhs);
        if(rhsValueType == MprVariable::Int){
            pResult = checkNumericalComparison(cmpOp, lhsValue, MprVariable::intValue(rhs));
        } else if(rhsValueType == MprVariable::Double){
            pResult = checkNumericalComparison(cmpOp, lhsValue, MprVariable::doubleValue(rhs));
        }
        break;
    }
    case MprVariable::Double:
    {
        auto lhsValue = MprVariable::doubleValue(lhs);
        if(rhsValueType == MprVariable::Int){
            pResult = checkNumericalComparison(cmpOp, lhsValue, MprVariable::intValue(rhs));
        } else if(rhsValueType == MprVariable::Double){
            pResult = checkNumericalComparison(cmpOp, lhsValue, MprVariable::doubleValue(rhs));
        }
        break;
    }
    case MprVariable::Bool:
        if(rhsValueType == MprVariable::Bool && cmpOp == "="){
            pResult = checkNumericalComparison(
                cmpOp, MprVariable::boolValue(lhs), MprVariable::boolValue(rhs));
        }
        break;

    case MprVariable::String:
        if(rhsValueType == MprVariable::String && cmpOp == "="){
            pResult = checkNumericalComparison(
                cmpOp, MprVariable::stringValue(lhs), MprVariable::stringValue(rhs));
        }
        break;
    }

    if(!pResult){
        io->os() << _("Type / operator mismatch in the conditional expression of a While statement.") << endl;
    }

    return pResult;
}
    
    
stdx::optional<MprVariable::Value> MprControllerItemBase::Impl::getTermValue
(string::const_iterator& pos, string::const_iterator end)
{
    stdx::optional<MprVariable::Value> value;
    std::smatch match;

    if(regex_search(pos, end,  match, stringPattern)){
        value = match.str(1);
        pos = match[0].second;
                
    } else if(regex_search(pos, end, match, floatPattern)){
        value = std::stod(match.str(0));
        pos = match[0].second;
            
    } else if(regex_search(pos, end, match, intPattern)){
        errno = 0;
        long number = strtol(match.str(0).c_str(), nullptr, 10);
        if(errno == ERANGE || number < INT_MIN || number > INT_MAX){
            io->os() << format(_("Integer value {0} is out of range."), match.str(0)) << endl;
        } else {
            value = std::stoi(match.str(0));
            pos = match[0].second;
        }
    } else if(regex_search(pos, end, match, boolPattern)){
        auto label = match.str(1);
        std::transform(label.begin(), label.end(), label.begin(), ::tolower);
        value = (label == "true") ? true : false;
        pos = match[0].second;
                
    } else {
        value = self->evalExpressionAsVariableValue(pos, end);
    }
    
    return value;
}


stdx::optional<string> MprControllerItemBase::Impl::getComparisonOperator
(string::const_iterator& pos, string::const_iterator end)
{
    std::smatch match;
    if(regex_search(pos, end,  match, cmpOperatorPattern)){
        pos = match[0].second;
        return match.str(1);
    }
    return stdx::nullopt;
}


bool MprControllerItemBase::Impl::interpretAssignStatement(MprAssignStatement* statement)
{
    auto& expression = statement->valueExpression();
    if(expression.empty()){
        io->os() << format(_("Expression assigned to variable {0} is empty."),
                           statement->variableExpression()) << endl;
        return false;
    }

    vector<MprVariable::Value> termValues;
    vector<char> operators;
    vector<string> termStrings;
    string invalidTerm;
    auto pos = expression.cbegin();
    auto end = expression.cend();
    std::smatch match;
    bool isValidExpression = true;
    bool isNextTermOperator = false;
        
    while(pos != end){
        if(!isNextTermOperator){
            auto pos0 = pos;
            if(auto term = getTermValue(pos, end)){
                termValues.push_back(*term);
                termStrings.push_back(string(pos0, pos));
                isNextTermOperator = true;
            } else {
                if(regex_search(pos, end, match, termPattern)){
                    invalidTerm = match.str(1);
                } else {
                    invalidTerm = string(match[0].first, match[0].second);
                }
                isValidExpression = false;
            }
        } else {
            if(regex_search(pos, end, match, operatorPattern)){
                char op = match.str(1)[0];
                operators.push_back(op);
                pos = match[0].second;
                isNextTermOperator = false;
            } else {
                invalidTerm = string(match[0].first, match[0].second);
                isValidExpression = false;
            }
        }
        if(!isValidExpression){
            io->os() << format(_("Term \"{0}\" is invalid."), invalidTerm) << endl;
            break;
        }
    }

    if(termValues.empty()){
        isValidExpression = false;
    } else if(!isNextTermOperator){
        io->os() << format(_("Expression ends with operator {0}."), operators.back()) << endl;
        isValidExpression = false;
    }
    if(!isValidExpression){
        return false;
    }

    size_t termIndex = 0;
    size_t operatorIndex = 0;
    MprVariable::Value value = termValues[termIndex++];
    while(operatorIndex < operators.size()){
        char op = operators[operatorIndex++];
        if(!applyBinaryOperation(value, op, termValues[termIndex++])){
            isValidExpression = false;
            io->os() << format(_("Type mismatch in expresion \"{0} {1} {2}\""),
                               termStrings[termIndex-2],
                               op,
                               termStrings[termIndex-1]) << endl;
            break;
        }
    }
    
    bool assigned = false;
    if(isValidExpression){
        auto assignValue =
            self->evalExpressionAsVariableToAssginValue(statement->variableExpression());
        if(assignValue && assignValue(value)){
            assigned = true;
            ++iterator;
        }
    }

    return assigned;
}


template<class ResultType, class LhsType, class RhsType>
static ResultType applyNumericalOperation(char op, LhsType lhs, RhsType rhs)
{
    if(op == '+'){
        return lhs + rhs;
    } else if(op == '-'){
        return lhs - rhs;
    }
    return 0;
}


static string applyStringOperation(char op, const string& lhs, const string& rhs)
{
    if(op == '='){
        return rhs;
    } else if(op == '+'){
        return lhs + rhs;
    }
    return string();
}


bool MprControllerItemBase::Impl::applyBinaryOperation
(MprVariable::Value& lhsValue, char op, const MprVariable::Value& rhsValue)
{
    bool result = false;
    int lhsType = MprVariable::valueType(lhsValue);
    int rhsType = MprVariable::valueType(rhsValue);
    
    switch(rhsType){
        
    case MprVariable::Int:
    {
        int rhs = MprVariable::intValue(rhsValue);
        if(lhsType == MprVariable::Int){
            lhsValue = applyNumericalOperation<int>(op, MprVariable::intValue(lhsValue), rhs);
            result = true;
        } else if(lhsType == MprVariable::Double){
            lhsValue = applyNumericalOperation<double>(op, MprVariable::doubleValue(lhsValue), rhs);
            result = true;
        }
        break;
    }
    case MprVariable::Double:
    {
        double rhs = MprVariable::doubleValue(rhsValue);
        if(lhsType == MprVariable::Int){
            lhsValue = applyNumericalOperation<double>(op, MprVariable::intValue(lhsValue), rhs);
            result = true;
        } else if(lhsType == MprVariable::Double){
            lhsValue = applyNumericalOperation<double>(op, MprVariable::doubleValue(lhsValue), rhs);
            result = true;
        }
        break;
    }
    case MprVariable::Bool:
    {
        bool rhs = MprVariable::boolValue(rhsValue);
        if(lhsType == MprVariable::Bool && op == '+'){
            lhsValue = MprVariable::boolValue(lhsValue) || rhs;
            result = true;
        }
        break;
    }
    case MprVariable::String:
        if(lhsType == MprVariable::String && op == '+'){
            lhsValue = MprVariable::stringValue(lhsValue) + MprVariable::stringValue(rhsValue);
            result = true;
        }
        break;
    }

    return result;
}


bool MprControllerItemBase::Impl::interpretSetSignalStatement(MprSignalStatement* statement)
{
    if(!ioDevice){
        io->os() << format(_("{0} cannot be executed because {1} does not have a signal I/O device"),
                           statement->label(0), io->body()->name()) << endl;
    } else {
        self->pushOutputOnceFunction(
            [this, statement](){
                ioDevice->setOut(statement->signalIndex(), statement->on(), true); });
    }

    ++iterator;
    
    return true;
}


bool MprControllerItemBase::Impl::interpretWaitStatement(MprWaitStatement* statement)
{
    if(statement->conditionType() == MprWaitStatement::SignalInput){
        if(!ioDevice){
            self->pushControlFunctions([]() { return true; });
        } else {
            self->pushControlFunctions(
                [this, statement](){
                    return ioDevice->in(statement->signalIndex()) != statement->signalStateCondition();
                });
        }
    }

    ++iterator;
    
    return true;
}

    
bool MprControllerItemBase::Impl::interpretDelayStatement(MprDelayStatement* statement)
{
    int remainingFrames = statement->time() / io->timeStep();
    self->pushControlFunctions([remainingFrames]() mutable { return (remainingFrames-- > 0); });
    ++iterator;
    return true;
}


void MprControllerItemBase::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("Speed ratio"), impl->speedRatio, changeProperty(impl->speedRatio));
}


bool MprControllerItemBase::store(Archive& archive)
{
    archive.write("speed_ratio", impl->speedRatio);
    return true;
}
    

bool MprControllerItemBase::restore(const Archive& archive)
{
    if(!archive.read("speed_ratio", impl->speedRatio)){
        archive.read("speedRatio", impl->speedRatio); // old
    }
    return true;
}


MprControllerLog::MprControllerLog()
{
    isErrorState_ = false;
}
