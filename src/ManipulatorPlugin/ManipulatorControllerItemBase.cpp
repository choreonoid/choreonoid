#include "ManipulatorControllerItemBase.h"
#include "ManipulatorProgramItemBase.h"
#include "BasicManipulatorStatements.h"
#include "ManipulatorVariableList.h"
#include "ManipulatorVariableListItemBase.h"
#include "ManipulatorVariableSetGroup.h"
#include <cnoid/LinkKinematicsKit>
#include <cnoid/LinkCoordinateFrameSet>
#include <cnoid/ManipulatorProgram>
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
    ManipulatorProgramPtr program;
    CloneMap cloneMap;
    LinkKinematicsKitPtr kinematicsKit;
    ManipulatorVariableSetPtr variables;
    vector<function<void()>> residentInputFunctions;
    unordered_map<type_index, function<bool(ManipulatorStatement* statement)>> interpreterMap;
    ManipulatorProgram::iterator iterator;
    vector<ref_ptr<Processor>> processorStack;
    DigitalIoDevicePtr ioDevice;
    double speedRatio;

    regex termPattern;
    regex operatorPattern;
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
    bool control();
    void clear();
    bool interpretCommentStatement(CommentStatement* statement);
    bool applyExpressionTerm(ManipulatorVariable::Value& value, ExpressionTerm term, char op);
    bool interpretDelayStatement(DelayStatement* statement);
    bool interpretAssignStatement(AssignStatement* statement);
    bool interpretSetSignalStatement(SetSignalStatement* statement);
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

    registerStatementInterpreter<DelayStatement>(
        [impl_](DelayStatement* statement){
            return impl_->interpretDelayStatement(statement); });

    registerStatementInterpreter<AssignStatement>(
        [impl_](AssignStatement* statement){
            return impl_->interpretAssignStatement(statement); });

    impl->termPattern.assign("^\\s*(.+)\\s*");
    impl->operatorPattern.assign("^\\s*([+-])\\s*");
    impl->intPattern.assign("^[+-]?\\d+");
    impl->floatPattern.assign("^[+-]?(\\d+\\.\\d*|\\.\\d+)");
    impl->boolPattern.assign("^([Tt][Rr][Uu][Ee]|[Ff][Aa][Ll][Ss][Ee])");
    impl->stringPattern.assign("^\"(.*)\"");
    impl->variablePattern.assign("^var\\[(\\d+)\\]");

    registerStatementInterpreter<SetSignalStatement>(
        [impl_](SetSignalStatement* statement){
            return impl_->interpretSetSignalStatement(statement); });
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

    programItem = nullptr;
    program = nullptr;
    
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

    if(!createKinematicsKitForControl()){
        return false;
    }
    
    cloneMap.clear();

    program = cloneMap.getClone(programItem->program());

    variables = createVariableSet(programItem);

    processorStack.clear();
    iterator = program->begin();

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


ManipulatorProgram* ManipulatorControllerItemBase::getProgram()
{
    return impl->program;
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
        if(isActive || iterator == program->end()){
            break;
        }
        auto statement = iterator->get();
        ++iterator;

        auto iter = interpreterMap.find(typeid(*statement));
        if(iter == interpreterMap.end()){
            io->os() << format(_("{0} cannot be executed because the interpreter for it is not found."),
                               statement->label(0)) << endl;
            continue;
        }
        auto& interpret = iter->second;
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
    program.reset();
    cloneMap.clear();
    kinematicsKit.reset();
}


double ManipulatorControllerItemBase::speedRatio() const
{
    return impl->speedRatio;
}


bool ManipulatorControllerItemBase::Impl::interpretCommentStatement(CommentStatement*)
{
    return true;
}


bool ManipulatorControllerItemBase::Impl::interpretDelayStatement(DelayStatement* statement)
{
    int remainingFrames = statement->time() / io->timeStep();
    self->pushControlFunctions([remainingFrames]() mutable { return (remainingFrames-- > 0); });
    return true;
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
