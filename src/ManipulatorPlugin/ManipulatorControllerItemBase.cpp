#include "ManipulatorControllerItemBase.h"
#include "ManipulatorProgramItemBase.h"
#include "BasicManipulatorStatements.h"
#include "ManipulatorVariableList.h"
#include "ManipulatorVariableListItemBase.h"
#include "ManipulatorVariableSetGroup.h"
#include <cnoid/LinkKinematicsKit>
#include <cnoid/LinkCoordinateFrameSet>
#include <cnoid/DigitalIoDevice>
#include <cnoid/ItemList>
#include <cnoid/ItemTreeView>
#include <cnoid/BodyItem>
#include <cnoid/ControllerIO>
#include <cnoid/MessageView>
#include <cnoid/CloneMap>
#include <cnoid/LazyCaller>
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

class ManipulatorControllerItemBase::Impl
{
public:
    ManipulatorControllerItemBase* self;
    ControllerIO* io;
    ManipulatorProgramItemBasePtr programItem;
    ManipulatorProgramPtr mainProgram;
    ManipulatorProgramPtr currentProgram;
    unordered_map<string, ManipulatorProgramPtr> otherProgramMap;
    CloneMap cloneMap;
    LinkKinematicsKitPtr kinematicsKit;
    ManipulatorVariableSetPtr variables;

    typedef ManipulatorProgram::iterator Iterator;
    Iterator iterator;

    struct ProgramPosition {
        ManipulatorProgramPtr program;
        Iterator iterator;
        ProgramPosition(ManipulatorProgram* program, Iterator iterator)
            : program(program), iterator(iterator) { }
    };
    vector<ProgramPosition> programStack;
    
    vector<ref_ptr<Processor>> processorStack;
    
    vector<function<void()>> residentInputFunctions;

    typedef function<bool(ManipulatorStatement* statement)> InterpretFunction;
    unordered_map<type_index, InterpretFunction> interpreterMap;
    DigitalIoDevicePtr ioDevice;
    double speedRatio;

    regex termPattern;
    regex operatorPattern;
    regex cmpOperatorPattern;
    regex intPattern;
    regex floatPattern;
    regex boolPattern;
    regex stringPattern;
    regex variablePattern;

    Impl(ManipulatorControllerItemBase* self);
    bool initialize(ControllerIO* io);
    bool createKinematicsKitForControl();
    ManipulatorVariableSetPtr createVariableSet(ManipulatorProgramItemBase* programItem);
    void extractProgramLocalVariables(ManipulatorVariableSetGroup* variables, Item* item);
    void extractControllerLocalVariables(ManipulatorVariableSetGroup* variables, Item* item);
    void extractBodyLocalVariables(ManipulatorVariableSetGroup* variables, Item* item);
    void addVariableList(ManipulatorVariableSetGroup* group, ManipulatorVariableListItemBase* listItem);
    void updateOrgVariableList(ManipulatorVariableList* orgList, ManipulatorVariable* variable);
    void setCurrent(ManipulatorProgram* program, ManipulatorProgram::iterator iter);
    bool control();
    void clear();
    bool interpretCommentStatement(CommentStatement* statement);
    bool interpretIfStatement(IfStatement* statement);
    bool interpretElseStatement(ElseStatement* statement);
    bool interpretWhileStatement(WhileStatement* statement);
    bool interpretCallStatement(CallStatement* statement);
    stdx::optional<bool> evalConditionalExpression(const string& expression);
    stdx::optional<ExpressionTerm> getTermValue(string::const_iterator& iter, string::const_iterator& end);
    stdx::optional<string> getComparisonOperator(string::const_iterator& iter, string::const_iterator& end);
    bool interpretAssignStatement(AssignStatement* statement);
    bool applyExpressionTerm(ManipulatorVariable::Value& value, ExpressionTerm term, char op);
    bool interpretSetSignalStatement(SetSignalStatement* statement);
    bool interpretDelayStatement(DelayStatement* statement);
};

}


ManipulatorControllerItemBase::ManipulatorControllerItemBase()
{
    impl = new Impl(this);
}


ManipulatorControllerItemBase::ManipulatorControllerItemBase(const ManipulatorControllerItemBase& org)
    : ControllerItem(org)
{
    impl = new Impl(this);
}


ManipulatorControllerItemBase::Impl::Impl(ManipulatorControllerItemBase* self)
    : self(self)
{
    speedRatio = 1.0;
}


ManipulatorControllerItemBase::~ManipulatorControllerItemBase()
{
    delete impl;
}


void ManipulatorControllerItemBase::registerStatementInterpreter
(std::type_index statementType, const std::function<bool(ManipulatorStatement* statement)>& interpret)
{
    impl->interpreterMap[statementType] = interpret;
}


void ManipulatorControllerItemBase::registerBaseStatementInterpreters()
{
    ManipulatorControllerItemBase::Impl* impl_ = impl;

    registerStatementInterpreter<CommentStatement>(
        [impl_](CommentStatement* statement){
            return impl_->interpretCommentStatement(statement); });

    registerStatementInterpreter<IfStatement>(
        [impl_](IfStatement* statement){
            return impl_->interpretIfStatement(statement); });

    registerStatementInterpreter<ElseStatement>(
        [impl_](ElseStatement* statement){
            return impl_->interpretElseStatement(statement); });

    registerStatementInterpreter<WhileStatement>(
        [impl_](WhileStatement* statement){
            return impl_->interpretWhileStatement(statement); });
 
    registerStatementInterpreter<CallStatement>(
        [impl_](CallStatement* statement){
            return impl_->interpretCallStatement(statement); });

    registerStatementInterpreter<AssignStatement>(
        [impl_](AssignStatement* statement){
            return impl_->interpretAssignStatement(statement); });

    registerStatementInterpreter<SetSignalStatement>(
        [impl_](SetSignalStatement* statement){
            return impl_->interpretSetSignalStatement(statement); });

    registerStatementInterpreter<DelayStatement>(
        [impl_](DelayStatement* statement){
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


void ManipulatorControllerItemBase::setResidentInputFunction(std::function<void()> input)
{
    impl->residentInputFunctions.push_back(input);
}


double ManipulatorControllerItemBase::timeStep() const
{
    return impl->io ? impl->io->timeStep() : 0.0;
}
        

bool ManipulatorControllerItemBase::initialize(ControllerIO* io)
{
    if(impl->initialize(io)){
        return onInitialize(io);
    }
    return false;
}    


bool ManipulatorControllerItemBase::Impl::initialize(ControllerIO* io)
{
    this->io = io;
    
    auto mv = MessageView::instance();

    clear();
    
    ItemList<ManipulatorProgramItemBase> programItems;
    if(!programItems.extractChildItems(self)){
        mv->putln(
            MessageView::ERROR,
            format(_("Any program item for {} is not found."), self->name()));
        return false;
    }

    programItem = programItems.front();
    // find the first checked item
    ItemTreeView* itv = ItemTreeView::instance();
    for(size_t i=0; i < programItems.size(); ++i){
        if(itv->isItemChecked(programItems[i])){
            programItem = programItems[i];
            break;
        }
    }

    mainProgram = cloneMap.getClone(programItem->program());
    currentProgram = mainProgram;

    if(!createKinematicsKitForControl()){
        return false;
    }

    for(auto& item : programItems){
        if(item != programItem){
            auto program = cloneMap.getClone(item->program());
            otherProgramMap.insert(make_pair(item->name(), program));
        }
    }
    
    variables = createVariableSet(programItem);

    iterator = currentProgram->begin();

    auto body = io->body();
    ioDevice = body->findDevice<DigitalIoDevice>();
    
    return true;
}


bool ManipulatorControllerItemBase::Impl::createKinematicsKitForControl()
{
    auto orgKit = programItem->kinematicsKit();

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


ManipulatorVariableSetPtr ManipulatorControllerItemBase::Impl::createVariableSet
(ManipulatorProgramItemBase* programItem)
{
    ManipulatorVariableSetGroupPtr group = new ManipulatorVariableSetGroup;
    
    extractProgramLocalVariables(group, programItem);
    extractControllerLocalVariables(group, self);

    if(auto bodyItem = self->findOwnerItem<BodyItem>()){
        extractBodyLocalVariables(group, bodyItem);
    }

    ManipulatorVariableSetPtr variableSet;
    int n = group->numVariableSets();
    if(n == 0){
        variableSet = new ManipulatorVariableList;
    } else if(n == 1){
        variableSet = group->variableSet(0);
    } else {
        variableSet = group;
    }

    return variableSet;
}


void ManipulatorControllerItemBase::Impl::extractProgramLocalVariables
(ManipulatorVariableSetGroup* group, Item* item)
{
    for(Item* child = item->childItem(); child; child = child->nextItem()){
        if(auto variableListItem = dynamic_cast<ManipulatorVariableListItemBase*>(child)){
            addVariableList(group, variableListItem);
        }
        extractProgramLocalVariables(group, child);
    }
}


void ManipulatorControllerItemBase::Impl::extractControllerLocalVariables
(ManipulatorVariableSetGroup* group, Item* item)
{
    for(Item* child = item->childItem(); child; child = child->nextItem()){
        if(auto programItem = dynamic_cast<ManipulatorProgramItemBase*>(child)){
            continue;
        }
        if(auto variableListItem = dynamic_cast<ManipulatorVariableListItemBase*>(child)){
            addVariableList(group, variableListItem);
        }
        extractControllerLocalVariables(group, child);
    }
}


void ManipulatorControllerItemBase::Impl::extractBodyLocalVariables
(ManipulatorVariableSetGroup* group, Item* item)
{
    for(Item* child = item->childItem(); child; child = child->nextItem()){
        if(auto controllerItem = dynamic_cast<ControllerItem*>(child)){
            continue;
        }
        if(auto variableListItem = dynamic_cast<ManipulatorVariableListItemBase*>(child)){
            addVariableList(group, variableListItem);
        }
        extractBodyLocalVariables(group, child);
    }
}


void ManipulatorControllerItemBase::Impl::addVariableList
(ManipulatorVariableSetGroup* group, ManipulatorVariableListItemBase* listItem)
{
    ManipulatorVariableListPtr orgList = listItem->variableList();
    auto cloneList = orgList->clone();

    cloneList->sigVariableUpdated().connect(
        [this, orgList](ManipulatorVariableSet*, ManipulatorVariable* variable){
            ManipulatorVariablePtr clone = variable->clone();
            callLater([this, &orgList, clone](){ updateOrgVariableList(orgList, clone); });
        });

    group->addVariableSet(cloneList);
}


/**
   This is a temporary implementation to update the original variables managed in the main GUI thread.
   The variables should be updated with the simulation log data which records the variable states for
   each time frame.
*/
void ManipulatorControllerItemBase::Impl::updateOrgVariableList
(ManipulatorVariableList* orgList, ManipulatorVariable* variable)
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
    
    
bool ManipulatorControllerItemBase::onInitialize(ControllerIO* /* io */)
{
    return true;
}


ManipulatorProgramItemBase* ManipulatorControllerItemBase::getProgramItem()
{
    return impl->programItem;
}


ManipulatorProgram* ManipulatorControllerItemBase::getMainProgram()
{
    return impl->mainProgram;
}


ManipulatorProgram* ManipulatorControllerItemBase::getCurrentProgram()
{
    return impl->currentProgram;
}


ManipulatorProgram::iterator ManipulatorControllerItemBase::getCurrentIterator()
{
    return impl->iterator;
}


void ManipulatorControllerItemBase::setCurrent(ManipulatorProgram::iterator iter)
{
    impl->iterator = iter;
}


void ManipulatorControllerItemBase::setCurrent(ManipulatorProgram* program, ManipulatorProgram::iterator iter)
{
    impl->setCurrent(program, iter);
}


void ManipulatorControllerItemBase::Impl::setCurrent(ManipulatorProgram* program, ManipulatorProgram::iterator iter)    
{
    programStack.push_back(ProgramPosition(currentProgram, iterator));
    currentProgram = program;
    iterator = iter;
}


ManipulatorProgram* ManipulatorControllerItemBase::findProgram(const std::string& name)
{
    auto iter = impl->otherProgramMap.find(name);
    if(iter != impl->otherProgramMap.end()){
        return iter->second;
    }
    return nullptr;
}


LinkKinematicsKit* ManipulatorControllerItemBase::getLinkKinematicsKit()
{
    return impl->kinematicsKit;
}


ManipulatorVariableSet* ManipulatorControllerItemBase::getVariableSet()
{
    return impl->variables;
}


bool ManipulatorControllerItemBase::start()
{
    return onStart();
}


bool ManipulatorControllerItemBase::onStart()
{
    return true;
}


void ManipulatorControllerItemBase::input()
{
    for(auto& input : impl->residentInputFunctions){
        input();
    }
    if(!impl->processorStack.empty()){
        impl->processorStack.back()->input();
    }
}


bool ManipulatorControllerItemBase::control()
{
    return impl->control();
}


bool ManipulatorControllerItemBase::Impl::control()
{
    bool isActive = false;

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
            auto& pos = programStack.back();
            iterator = pos.iterator;
            currentProgram = pos.program;
            programStack.pop_back();
            hasNextStatement = (iterator != currentProgram->end());
        }
        if(!hasNextStatement){
            break;
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
        
    return isActive;
}


void ManipulatorControllerItemBase::pushControlFunctions
(std::function<bool()> control, std::function<void()> input, std::function<void()> output)
{
    impl->processorStack.push_back(new ControlFunctionSetProcessor(control, input, output));
}
        

void ManipulatorControllerItemBase::pushOutputOnceFunction(std::function<void()> outputOnce)
{
    impl->processorStack.push_back(new OutputOnceFunctionProcessor(outputOnce));
}


void ManipulatorControllerItemBase::output()
{
    if(!impl->processorStack.empty()){
        impl->processorStack.back()->output();
    }
}


void ManipulatorControllerItemBase::stop()
{
    impl->clear();
    onStop();
}


bool ManipulatorControllerItemBase::onStop()
{
    return true;
}


void ManipulatorControllerItemBase::onDisconnectedFromRoot()
{
    impl->clear();
}


void ManipulatorControllerItemBase::Impl::clear()
{
    programItem.reset();
    mainProgram.reset();
    currentProgram.reset();
    programStack.clear();
    processorStack.clear();
    cloneMap.clear();
    kinematicsKit.reset();
    variables.reset();
}


double ManipulatorControllerItemBase::speedRatio() const
{
    return impl->speedRatio;
}


bool ManipulatorControllerItemBase::Impl::interpretCommentStatement(CommentStatement*)
{
    ++iterator;
    return true;
}


bool ManipulatorControllerItemBase::Impl::interpretIfStatement(IfStatement* statement)
{
    auto condition = evalConditionalExpression(statement->condition());

    if(!condition){
        return false;
    }

    ++iterator;

    ElseStatement* nextElseStatement = nullptr;
    if(iterator != currentProgram->end()){
        nextElseStatement = dynamic_cast<ElseStatement*>(iterator->get());
    }

    bool processed = true;

    if(*condition){
        if(nextElseStatement){
            ++iterator;
        }
        auto program = statement->lowerLevelProgram();
        setCurrent(program, program->begin());

    } else if(nextElseStatement){
        processed = interpretElseStatement(nextElseStatement);
    }

    return processed;
}


bool ManipulatorControllerItemBase::Impl::interpretElseStatement(ElseStatement* statement)
{
    ++iterator;
    auto program = statement->lowerLevelProgram();
    setCurrent(program, program->begin());
    return true;
}
    

bool ManipulatorControllerItemBase::Impl::interpretWhileStatement(WhileStatement* statement)
{
    auto condition = evalConditionalExpression(statement->condition());

    if(*condition){
        auto program = statement->lowerLevelProgram();
        setCurrent(program, program->begin());
    } else {
        ++iterator;
    }
    
    return true;
}


bool ManipulatorControllerItemBase::Impl::interpretCallStatement(CallStatement* statement)
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

    ++iterator;
    setCurrent(program, program->begin());

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


stdx::optional<bool> ManipulatorControllerItemBase::Impl::evalConditionalExpression(const string& expression)
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
    
    
stdx::optional<ExpressionTerm> ManipulatorControllerItemBase::Impl::getTermValue
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
            case ManipulatorVariable::Int:
                value = ExpressionTerm(variable->toInt());
                break;
            case ManipulatorVariable::Double:
                value = ExpressionTerm(variable->toDouble());
                break;
            case ManipulatorVariable::Bool:
                value = ExpressionTerm(variable->toBool());
                break;
            case ManipulatorVariable::String:
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


stdx::optional<string> ManipulatorControllerItemBase::Impl::getComparisonOperator
(string::const_iterator& iter, string::const_iterator& end)
{
    std::smatch match;
    if(regex_search(iter, end,  match, cmpOperatorPattern)){
        iter = match[0].second;
        return match.str(1);
    }
    return stdx::nullopt;
}


bool ManipulatorControllerItemBase::Impl::interpretAssignStatement(AssignStatement* statement)
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

    ManipulatorVariable::Value value = variable->variantValue();
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


bool ManipulatorControllerItemBase::Impl::applyExpressionTerm
(ManipulatorVariable::Value& value, ExpressionTerm term, char op)
{
    int termType = stdx::get_variant_index(term);

    if(termType == Variable){
        auto id = stdx::get<GeneralId>(term);
        auto variable = variables->findVariable(id);
        switch(variable->valueTypeId()){
        case ManipulatorVariable::Int:    term = variable->toInt();    break;
        case ManipulatorVariable::Double: term = variable->toDouble(); break;
        case ManipulatorVariable::Bool:   term = variable->toBool();   break;
        case ManipulatorVariable::String: term = variable->toString(); break;
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
        } else if(valueType == ManipulatorVariable::Int){
            value = applyNumericalOperation<int>(op, stdx::get<int>(value), rhs);
        } else if(valueType == ManipulatorVariable::Double){
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
        } else if(valueType == ManipulatorVariable::Int){
            value = applyNumericalOperation<double>(op, stdx::get<int>(value), rhs);
        } else if(valueType == ManipulatorVariable::Double){
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
        } else if(valueType == ManipulatorVariable::Bool && op == '+'){
            value = stdx::get<bool>(value) || rhs;
        } else {
            return false;
        }
        break;
    }
    case String:
        if(op == '='){
            value = stdx::get<string>(term);
        } else if(valueType == ManipulatorVariable::String && op == '+'){
            value = stdx::get<string>(value) + stdx::get<string>(term);
        } else {
            return false;
        }
        break;
    }

    return true;
}


bool ManipulatorControllerItemBase::Impl::interpretSetSignalStatement(SetSignalStatement* statement)
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

    
bool ManipulatorControllerItemBase::Impl::interpretDelayStatement(DelayStatement* statement)
{
    int remainingFrames = statement->time() / io->timeStep();
    self->pushControlFunctions([remainingFrames]() mutable { return (remainingFrames-- > 0); });
    ++iterator;
    return true;
}


void ManipulatorControllerItemBase::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("Speed ratio"), impl->speedRatio, changeProperty(impl->speedRatio));
}


bool ManipulatorControllerItemBase::store(Archive& archive)
{
    archive.write("speedRatio", impl->speedRatio);
    return true;
}
    

bool ManipulatorControllerItemBase::restore(const Archive& archive)
{
    archive.read("speedRatio", impl->speedRatio);
    return true;
}
