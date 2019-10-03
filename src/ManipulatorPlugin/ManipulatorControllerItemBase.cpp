#include "ManipulatorControllerItemBase.h"
#include "ManipulatorProgramItemBase.h"
#include <cnoid/BodyManipulatorManager>
#include <cnoid/ManipulatorProgram>
#include <cnoid/DigitalIoDevice>
#include <cnoid/ItemList>
#include <cnoid/ItemTreeView>
#include <cnoid/ControllerIO>
#include <cnoid/MessageView>
#include <cnoid/Archive>
#include <fmt/format.h>
#include <unordered_map>
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

class ManipulatorControllerItemBase::Impl
{
public:
    ManipulatorControllerItemBase* self;
    ControllerIO* io;
    ManipulatorProgramItemBasePtr programItem;
    ManipulatorProgramPtr program;
    ManipulatorProgramCloneMap manipulatorProgramCloneMap;
    BodyManipulatorManagerPtr manipulatorManager;
    vector<function<void()>> residentInputFunctions;
    unordered_map<type_index, function<bool(ManipulatorStatement* statement)>> interpreterMap;
    ManipulatorProgram::iterator iterator;
    vector<ref_ptr<Processor>> processorStack;
    DigitalIoDevicePtr ioDevice;
    double speedRatio;

    Impl(ManipulatorControllerItemBase* self);
    bool initialize(ControllerIO* io);
    bool control();
    void clear();
    bool interpretDelayStatement(DelayStatement* statement);
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
    manipulatorProgramCloneMap.setPositionSetIncluded(false);
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
    
    registerStatementInterpreter<DelayStatement>(
        [impl_](DelayStatement* statement){
            return impl_->interpretDelayStatement(statement); });

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

    manipulatorProgramCloneMap.clear();
    program = programItem->program()->clone(manipulatorProgramCloneMap);

    manipulatorManager = programItem->manipulatorManager()->clone();

    processorStack.clear();
    iterator = program->begin();

    auto body = io->body();
    ioDevice = body->findDevice<DigitalIoDevice>();
    
    return true;
}


bool ManipulatorControllerItemBase::onInitialize(ControllerIO* /* io */)
{
    return true;
}


ManipulatorProgramItemBase* ManipulatorControllerItemBase::getManipulatorProgramItem()
{
    return impl->programItem;
}


ManipulatorProgram* ManipulatorControllerItemBase::getManipulatorProgram()
{
    return impl->program;
}


BodyManipulatorManager* ManipulatorControllerItemBase::getBodyManipulatorManager()
{
    return impl->manipulatorManager;
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
    manipulatorProgramCloneMap.clear();
    manipulatorManager.reset();
}


double ManipulatorControllerItemBase::speedRatio() const
{
    return impl->speedRatio;
}


bool ManipulatorControllerItemBase::Impl::interpretDelayStatement(DelayStatement* statement)
{
    int remainingFrames = statement->time() / io->timeStep();
    self->pushControlFunctions([remainingFrames]() mutable { return (remainingFrames-- > 0); });
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
