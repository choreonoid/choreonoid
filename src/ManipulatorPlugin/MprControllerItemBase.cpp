#include "MprControllerItemBase.h"
#include "MprProgramItemBase.h"
#include "BasicMprStatements.h"
#include "MprVariableList.h"
#include "MprVariableListItemBase.h"
#include "MprVariableSetGroup.h"
#include <cnoid/ItemManager>
#include <cnoid/LinkKinematicsKit>
#include <cnoid/LinkCoordinateFrameSet>
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

enum ExpressionTermId {
    Int, Double, Bool, String, Variable, Char };

typedef stdx::variant<int, double, bool, string, GeneralId, char> ExpressionTerm;

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
    MprVariableSetPtr variables;

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
    regex variablePattern;

    Impl(MprControllerItemBase* self);
    bool initialize(ControllerIO* io);
    bool createKinematicsKitForControl();
    MprVariableSetPtr createVariableSet(MprProgramItemBase* programItem);
    void extractProgramLocalVariables(MprVariableSetGroup* variables, Item* item);
    void extractControllerLocalVariables(MprVariableSetGroup* variables, Item* item);
    void extractBodyLocalVariables(MprVariableSetGroup* variables, Item* item);
    void addVariableList(MprVariableSetGroup* group, MprVariableListItemBase* listItem);
    void updateOrgVariableList(MprVariableList* orgList, MprVariable* variable);
    void setCurrent(MprProgram* program, MprProgram::iterator iter, MprProgram::iterator upperNext);
    bool control();
    void setCurrentProgramPositionToLog(MprControllerLog* log);
    void clear();
    bool interpretCommentStatement(MprCommentStatement* statement);
    bool interpretIfStatement(MprIfStatement* statement);
    bool interpretWhileStatement(MprWhileStatement* statement);
    bool interpretCallStatement(MprCallStatement* statement);
    stdx::optional<bool> evalConditionalExpression(const string& expression);
    stdx::optional<ExpressionTerm> getTermValue(string::const_iterator& iter, string::const_iterator& end);
    stdx::optional<string> getComparisonOperator(string::const_iterator& iter, string::const_iterator& end);
    bool interpretAssignStatement(MprAssignStatement* statement);
    bool applyExpressionTerm(MprVariable::Value& value, ExpressionTerm term, char op);
    bool interpretSetSignalStatement(MprSignalStatement* statement);
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
    if(impl->initialize(io)){
        return onInitialize(io);
    }
    return false;
}    


bool MprControllerItemBase::Impl::initialize(ControllerIO* io)
{
    this->io = io;
    
    auto mv = MessageView::instance();

    clear();
    
    auto programItems = self->descendantItems<MprProgramItemBase>();

    if(programItems.empty()){
        mv->putln(format(_("Any program item for {} is not found."),
                         self->name()), MessageView::ERROR);
        return false;
    }
    
    startupProgramItem = programItems.front();
    for(auto& programItem : programItems){
        if(programItem->isStartupProgram()){
            startupProgramItem = programItem;
            break;
        }
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
    
    variables = createVariableSet(startupProgramItem);

    iterator = currentProgram->begin();

    auto body = io->body();
    ioDevice = body->findDevice<DigitalIoDevice>();

    isLogEnabled = io->enableLog();
    
    return true;
}


bool MprControllerItemBase::Impl::createKinematicsKitForControl()
{
    auto orgKit = startupProgramItem->kinematicsKit();

    if(!orgKit->baseLink()){
        return false;
    }

    auto body = orgKit->body()->clone();
    auto targetLink = body->link(orgKit->link()->index());
    auto baseLink = body->link(orgKit->baseLink()->index());

    kinematicsKit = new LinkKinematicsKit(targetLink);
    kinematicsKit->setBaseLink(baseLink);
    kinematicsKit->setFrameSets(new LinkCoordinateFrameSet(*orgKit->frameSets()));

    return true;
}


MprVariableSetPtr MprControllerItemBase::Impl::createVariableSet
(MprProgramItemBase* programItem)
{
    MprVariableSetGroupPtr group = new MprVariableSetGroup;
    
    extractProgramLocalVariables(group, programItem);
    extractControllerLocalVariables(group, self);

    if(auto bodyItem = self->findOwnerItem<BodyItem>()){
        extractBodyLocalVariables(group, bodyItem);
    }

    MprVariableSetPtr variableSet;
    int n = group->numVariableSets();
    if(n == 0){
        variableSet = new MprVariableList;
    } else if(n == 1){
        variableSet = group->variableSet(0);
    } else {
        variableSet = group;
    }

    return variableSet;
}


void MprControllerItemBase::Impl::extractProgramLocalVariables
(MprVariableSetGroup* group, Item* item)
{
    for(Item* child = item->childItem(); child; child = child->nextItem()){
        if(auto variableListItem = dynamic_cast<MprVariableListItemBase*>(child)){
            addVariableList(group, variableListItem);
        }
        extractProgramLocalVariables(group, child);
    }
}


void MprControllerItemBase::Impl::extractControllerLocalVariables
(MprVariableSetGroup* group, Item* item)
{
    for(Item* child = item->childItem(); child; child = child->nextItem()){
        if(auto programItem = dynamic_cast<MprProgramItemBase*>(child)){
            continue;
        }
        if(auto variableListItem = dynamic_cast<MprVariableListItemBase*>(child)){
            addVariableList(group, variableListItem);
        }
        extractControllerLocalVariables(group, child);
    }
}


void MprControllerItemBase::Impl::extractBodyLocalVariables
(MprVariableSetGroup* group, Item* item)
{
    for(Item* child = item->childItem(); child; child = child->nextItem()){
        if(auto controllerItem = dynamic_cast<ControllerItem*>(child)){
            continue;
        }
        if(auto variableListItem = dynamic_cast<MprVariableListItemBase*>(child)){
            addVariableList(group, variableListItem);
        }
        extractBodyLocalVariables(group, child);
    }
}


void MprControllerItemBase::Impl::addVariableList
(MprVariableSetGroup* group, MprVariableListItemBase* listItem)
{
    MprVariableListPtr orgList = listItem->variableList();
    auto cloneList = orgList->clone();

    cloneList->sigVariableUpdated().connect(
        [this, orgList](MprVariableSet*, MprVariable* variable){
            MprVariablePtr clone = variable->clone();
            callLater([this, &orgList, clone](){ updateOrgVariableList(orgList, clone); });
        });

    group->addVariableSet(cloneList);
}


/**
   This is a temporary implementation to update the original variables managed in the main GUI thread.
   The variables should be updated with the simulation log data which records the variable states for
   each time frame.
*/
void MprControllerItemBase::Impl::updateOrgVariableList
(MprVariableList* orgList, MprVariable* variable)
{
    auto orgVariable = orgList->findVariable(variable->id());
    if(!orgVariable){
        orgList->append(variable);
        orgVariable = variable;
    } else {
        *orgVariable = *variable;
    }
    orgList->notifyVariableUpdate(orgVariable);
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


LinkKinematicsKit* MprControllerItemBase::getLinkKinematicsKit()
{
    return impl->kinematicsKit;
}


MprVariableSet* MprControllerItemBase::getVariableSet()
{
    return impl->variables;
}


bool MprControllerItemBase::start()
{
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
    bool isActive = false;
    bool stateChanged = false;

    while(true){
        while(!processorStack.empty()){
            isActive = processorStack.back()->control();
            if(isActive){
                break;
            }
            processorStack.pop_back();
        }
        if(isActive){
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
            continue;
        }
        auto& interpret = p->second;
        bool result = interpret(statement);

        if(!result){
            isActive = false;
            break;
        }
    }

    if(isLogEnabled && stateChanged){
        io->outputLog(new MprControllerLog(*currentLog));
    }
        
    return isActive;
}


void MprControllerItemBase::Impl::setCurrentProgramPositionToLog(MprControllerLog* log)
{
    auto topLevelProgram = currentProgram->topLevelProgram();
    auto it = topLevelProgramToSharedNameMap.find(topLevelProgram);
    if(it != topLevelProgramToSharedNameMap.end()){
        log->topLevelProgramName = it->second;
    } else {
        log->topLevelProgramName = make_shared<string>(topLevelProgram->name());
        topLevelProgramToSharedNameMap[topLevelProgram] = log->topLevelProgramName;
    }
        
    if(currentProgram->isTopLevelProgram()){
        log->hierachicalPosition.clear();
    } else if(!programStack.empty()){
        log->hierachicalPosition = programStack.back().hierachicalPosition;
    }
    int statementIndex = iterator - currentProgram->begin();
    log->hierachicalPosition.push_back(statementIndex);
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
    cloneMap.clear();
    kinematicsKit.reset();
    variables.reset();
    programStack.clear();
    processorStack.clear();
    topLevelProgramToSharedNameMap.clear();
}


double MprControllerItemBase::speedRatio() const
{
    return impl->speedRatio;
}


bool MprControllerItemBase::Impl::interpretCommentStatement(MprCommentStatement*)
{
    ++iterator;
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
        if(nextElseStatement = dynamic_cast<MprElseStatement*>(next->get())){
            ++next;
        }
    }

    if(*condition){
        auto program = statement->lowerLevelProgram();
        setCurrent(program, program->begin(), next);

    } else if(nextElseStatement){
        ++iterator;
        auto program = nextElseStatement->lowerLevelProgram();
        setCurrent(program, program->begin(), next);
    }

    return true;
}


bool MprControllerItemBase::Impl::interpretWhileStatement(MprWhileStatement* statement)
{
    auto condition = evalConditionalExpression(statement->condition());

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
    stdx::optional<bool> pResult;

    if(expression.empty()){
        io->os() << _("The conditional expression of a While statement is empty.") << endl;
        return pResult;
    }
    auto iter = expression.cbegin();
    auto end = expression.cend();

    stdx::optional<ExpressionTerm> pLhs = getTermValue(iter, end);
    stdx::optional<string> pCmpOp = getComparisonOperator(iter, end);
    stdx::optional<ExpressionTerm> pRhs = getTermValue(iter, end);

    if(!pLhs || !pCmpOp || !pRhs){
        io->os() << _("The conditional expression of a While statement is invalid.") << endl;
        return pResult;
    }

    ExpressionTerm& lhs = *pLhs;
    ExpressionTerm& rhs = *pRhs;
    string& cmpOp = *pCmpOp;

    int rhsValueType = stdx::get_variant_index(rhs);
    switch(stdx::get_variant_index(lhs)){
    case Int:
    {
        int lhsValue = stdx::get<int>(lhs);
        if(rhsValueType == Int){
            pResult = checkNumericalComparison(cmpOp, lhsValue, stdx::get<int>(rhs));
        } else if(rhsValueType == Double){
            pResult = checkNumericalComparison(cmpOp, lhsValue, stdx::get<double>(rhs));
        }
        break;
    }
    case Double:
    {
        int lhsValue = stdx::get<double>(lhs);
        if(rhsValueType == Int){
            pResult = checkNumericalComparison(cmpOp, lhsValue, stdx::get<int>(rhs));
        } else if(rhsValueType == Double){
            pResult = checkNumericalComparison(cmpOp, lhsValue, stdx::get<double>(rhs));
        }
        break;
    }
    case Bool:
        if(rhsValueType == Bool && cmpOp == "="){
            pResult = checkNumericalComparison(cmpOp, stdx::get<bool>(lhs), stdx::get<bool>(rhs));
        }
        break;

    case String:
        if(rhsValueType == String && cmpOp == "="){
            pResult = checkNumericalComparison(cmpOp, stdx::get<string>(lhs), stdx::get<string>(rhs));
        }
        break;
    }

    if(!pResult){
        io->os() << _("Type / operator mismatch in the conditional expression of a While statement.") << endl;
    }

    return pResult;
}
    
    
stdx::optional<ExpressionTerm> MprControllerItemBase::Impl::getTermValue
(string::const_iterator& iter, string::const_iterator& end)
{
    stdx::optional<ExpressionTerm> value;
    std::smatch match;

    if(regex_search(iter, end,  match, stringPattern)){
        value = ExpressionTerm(match.str(1));
                
    } else if(regex_search(iter, end, match, floatPattern)){
        value = ExpressionTerm(std::stod(match.str(0)));
            
    } else if(regex_search(iter, end, match, intPattern)){
        value = ExpressionTerm(std::stoi(match.str(0)));

    } else if(regex_search(iter, end, match, boolPattern)){
        auto label = match.str(1);
        std::transform(label.begin(), label.end(), label.begin(), ::tolower);
        value = ExpressionTerm(label == "true" ? true : false);
                
    } else if(regex_search(iter, end, match, variablePattern)){
        GeneralId id(std::stoi(match.str(1)));
        auto variable = variables->findVariable(id);
        if(!variable){
            io->os() << format(_("Variable {0} is not defined."), id.label()) << endl;
        } else {
            switch(variable->valueTypeId()){
            case MprVariable::Int:
                value = ExpressionTerm(variable->toInt());
                break;
            case MprVariable::Double:
                value = ExpressionTerm(variable->toDouble());
                break;
            case MprVariable::Bool:
                value = ExpressionTerm(variable->toBool());
                break;
            case MprVariable::String:
                value = ExpressionTerm(variable->toString());
                break;
            default:
                break;
            }
        }
    }

    if(value){
        iter = match[0].second;
    }
    
    return value;
}


stdx::optional<string> MprControllerItemBase::Impl::getComparisonOperator
(string::const_iterator& iter, string::const_iterator& end)
{
    std::smatch match;
    if(regex_search(iter, end,  match, cmpOperatorPattern)){
        iter = match[0].second;
        return match.str(1);
    }
    return stdx::nullopt;
}


bool MprControllerItemBase::Impl::interpretAssignStatement(MprAssignStatement* statement)
{
    auto variable = statement->variable(variables);

    if(!variable){
        io->os() << format(_("Variable {0} is not available."), statement->variableId().label()) << endl;
        return false;
    }
        
    std::vector<ExpressionTerm> terms;
    auto& expression = statement->expression();
    if(expression.empty()){
        io->os() << format(_("Expression assigned to variable {0} is empty."), variable->id().label()) << endl;
        return false;
    }

    auto iter = expression.cbegin();
    auto end = expression.cend();
    std::smatch match;
    bool isExpressionValid = true;
        
    while(iter != end){
        if(terms.empty() || stdx::get_variant_index(terms.back()) == Char){
            if(regex_search(iter, end,  match, stringPattern)){
                terms.push_back(match.str(1));
                
            } else if(regex_search(iter, end, match, floatPattern)){
                terms.push_back(std::stod(match.str(0)));
            
            } else if(regex_search(iter, end, match, intPattern)){
                terms.push_back(stoi(match.str(0)));

            } else if(regex_search(iter, end, match, boolPattern)){
                auto label = match.str(1);
                std::transform(label.begin(), label.end(), label.begin(), ::tolower);
                terms.push_back(label == "true" ? true : false);
                
            } else if(regex_search(iter, end, match, variablePattern)){
                GeneralId id(std::stoi(match.str(1)));
                if(!variables->findVariable(id)){
                    io->os() << format(_("Variable {0} is not defined."), id.label()) << endl;
                    isExpressionValid = false;
                } else {
                    terms.push_back(id);
                }
            } else {
                string term;
                if(regex_search(iter, end, match, termPattern)){
                    term = match.str(1);
                } else {
                    term = string(match[0].first, match[0].second);
                }
                io->os() << format(_("Term {0} is invalid."), term) << endl;
                isExpressionValid = false;
            }
        } else {
            if(regex_search(iter, end, match, operatorPattern)){
                char op = match.str(1)[0];
                terms.push_back(op);
            }
        }
        if(!isExpressionValid){
            break;
        }
        iter = match[0].second;
    }

    if(terms.empty()){
        isExpressionValid = false;

    } else if(stdx::get_variant_index(terms.back()) == Char){
        io->os() << format(_("Expression ends with operator {0}."), stdx::get<char>(terms.back())) << endl;
        isExpressionValid = false;
    }

    if(!isExpressionValid){
        return false;
    }

    MprVariable::Value value = variable->variantValue();
    if(applyExpressionTerm(value, terms[0], '=')){
        int index = 1;
        while(index + 1 < terms.size()){
            char op = stdx::get<char>(terms[index++]);
            if(!applyExpressionTerm(value, terms[index++], op)){
                isExpressionValid = false;
                break;
            }
        }
    }

    if(!isExpressionValid){
        io->os() << format(_("Type mismatch in expresion {0}"), expression) << endl;
        return false;
    }

    variable->setValue(value);
    self->pushOutputOnceFunction([variable](){ variable->notifyUpdate(); });

    ++iterator;

    return true;
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


bool MprControllerItemBase::Impl::applyExpressionTerm
(MprVariable::Value& value, ExpressionTerm term, char op)
{
    int termType = stdx::get_variant_index(term);

    if(termType == Variable){
        auto id = stdx::get<GeneralId>(term);
        auto variable = variables->findVariable(id);
        switch(variable->valueTypeId()){
        case MprVariable::Int:    term = variable->toInt();    break;
        case MprVariable::Double: term = variable->toDouble(); break;
        case MprVariable::Bool:   term = variable->toBool();   break;
        case MprVariable::String: term = variable->toString(); break;
        default: return false;
        }
        termType = stdx::get_variant_index(term);
    }

    int valueType = stdx::get_variant_index(value);

    switch(termType){
    case Int:
    {
        int rhs = stdx::get<int>(term);
        if(op == '='){
            value = rhs;
        } else if(valueType == MprVariable::Int){
            value = applyNumericalOperation<int>(op, stdx::get<int>(value), rhs);
        } else if(valueType == MprVariable::Double){
            value = applyNumericalOperation<double>(op, stdx::get<double>(value), rhs);
        } else {
            return false;
        }
        break;
    }
    case Double:
    {
        double rhs = stdx::get<double>(term);
        if(op == '='){
            value = rhs;
        } else if(valueType == MprVariable::Int){
            value = applyNumericalOperation<double>(op, stdx::get<int>(value), rhs);
        } else if(valueType == MprVariable::Double){
            value = applyNumericalOperation<double>(op, stdx::get<double>(value), rhs);
        } else {
            return false;
        }
        break;
    }
    case Bool:
    {
        bool rhs = stdx::get<bool>(term);
        if(op == '='){
            value = rhs;
        } else if(valueType == MprVariable::Bool && op == '+'){
            value = stdx::get<bool>(value) || rhs;
        } else {
            return false;
        }
        break;
    }
    case String:
        if(op == '='){
            value = stdx::get<string>(term);
        } else if(valueType == MprVariable::String && op == '+'){
            value = stdx::get<string>(value) + stdx::get<string>(term);
        } else {
            return false;
        }
        break;
    }

    return true;
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
