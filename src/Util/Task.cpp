/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include "Task.h"
#include "AbstractTaskSequencer.h"
#include "ValueTree.h"
#include <algorithm>

using namespace std;
using namespace cnoid;


TaskProc::~TaskProc()
{

}


bool TaskProc::waitForSignal(SignalProxy<void()> signalProxy, double timeout)
{
    return waitForCommandToFinish(
        signalProxy.connect([this](){ notifyCommandFinish(true); }),
        timeout);
}


bool TaskProc::waitForBooleanSignal(SignalProxy<void(bool)> signalProxy, double timeout)
{
    return waitForCommandToFinish(
        signalProxy.connect([this](bool isCompleted){ notifyCommandFinish(isCompleted); }),
        timeout);
}


void TaskToggleState::setChecked(bool on)
{
    if(on != isChecked_){
        isChecked_ = on;
        sigToggled_(on);
    }
}


TaskCommand::TaskCommand()
{
    initialize();
}


TaskCommand::TaskCommand(const std::string& caption)
    : caption_(caption)
{
    initialize();
}


void TaskCommand::initialize()
{
    nextPhaseIndex_ = 0;
    nextCommandIndex_ = 0;
    level_ = 0;
    isNextPhaseRelative_ = true;
    isNextCommandRelative_ = true;
    isCommandLinkAutomatic_ = false;
    isDefault_ = false;
}


TaskCommand::~TaskCommand()
{

}


TaskCommand* TaskCommand::setCheckable(bool /* on */)
{
    toggleState();
    return this;
}


TaskCommand* TaskCommand::setToggleState(TaskToggleState* state)
{
    toggleState_ = state;
    return this;
}


TaskToggleState* TaskCommand::toggleState()
{
    if(!toggleState_){
        toggleState_ = new TaskToggleState();
    }
    return toggleState_;
}


TaskCommand* TaskCommand::setChecked(bool on)
{
    toggleState()->setChecked(on);
    return this;
}


bool TaskCommand::isChecked() const
{
    if(toggleState_){
        return toggleState_->isChecked();
    }
    return false;
}


int TaskCommand::nextPhaseIndex(int currentPhaseIndex) const
{
    return isNextPhaseRelative_ ? (currentPhaseIndex + nextPhaseIndex_) : nextPhaseIndex_;
}


TaskCommand* TaskCommand::setPhaseLink(int phaseIndex)
{
    nextPhaseIndex_ = phaseIndex;
    isNextPhaseRelative_ = false;
    return this;
}    


TaskCommand* TaskCommand::setPhaseLinkStep(int phaseIndexStep)
{
    nextPhaseIndex_ = phaseIndexStep;
    isNextPhaseRelative_ = true;
    return this;
}    


int TaskCommand::nextCommandIndex(int currentCommandIndex) const
{
    return isNextCommandRelative_ ? (currentCommandIndex + nextCommandIndex_) : nextCommandIndex_;
}


TaskCommand* TaskCommand::setCommandLink(int commandIndex)
{
    nextCommandIndex_ = commandIndex;
    isNextCommandRelative_ = false;
    return this;
}    


TaskCommand* TaskCommand::setCommandLinkStep(int commandIndexStep)
{
    nextCommandIndex_ = commandIndexStep;
    isNextCommandRelative_ = true;
    return this;
}


TaskCommand* TaskCommand::linkToNextTask()
{
    return setPhaseLink(std::numeric_limits<int>::max());
}


TaskPhase::TaskPhase(const std::string& caption)
    : caption_(caption)
{
    isSkipped_ = false;
}


TaskPhase::TaskPhase(const TaskPhase& org, bool doDeepCopy)
    : caption_(org.caption_),
      preCommand_(org.preCommand_),
      commands(org.commands),
      isSkipped_(org.isSkipped_)
{
    if(doDeepCopy){
        for(size_t i=0; i < commands.size(); ++i){
            commands[i] = new TaskCommand(*commands[i]);
        }
    }
}


TaskPhase::~TaskPhase()
{

}


TaskPhase* TaskPhase::clone(bool doDeepCopy)
{
    return new TaskPhase(*this, doDeepCopy);
}


void TaskPhase::setCaption(const std::string& str)
{
    caption_ = str;
}


void TaskPhase::setPreCommand(TaskFunc func)
{
    preCommand_ = func;
}


TaskCommand* TaskPhase::addCommand()
{
    TaskCommand* command = new TaskCommand();
    commands.push_back(command);
    return command;
}


TaskCommand* TaskPhase::addCommand(const std::string& caption)
{
    TaskCommand* command = new TaskCommand(caption);
    commands.push_back(command);
    return command;
}


TaskCommand* TaskPhase::addToggleCommand()
{
    return addCommand()->setCheckable();
}


TaskCommand* TaskPhase::addToggleCommand(const std::string& caption)
{
    return addCommand(caption)->setCheckable();
}


TaskCommand* TaskPhase::command(int index) const
{
    TaskCommand* command = 0;
    if(index >= 0 && index < static_cast<int>(commands.size())){
        command = commands[index];
    }
    return command;
}


TaskPhaseProxyPtr TaskPhase::commandLevel(int level)
{
    TaskPhaseProxyPtr proxy = new TaskPhaseProxy(this);
    proxy->setCommandLevel(level);
    return proxy;
}


int TaskPhase::maxCommandLevel() const
{
    int maxLevel = -1;
    for(size_t i=0; i < commands.size(); ++i){
        maxLevel = std::max(maxLevel, commands[i]->level());
    }
    return maxLevel;
}


TaskPhaseProxy::TaskPhaseProxy(TaskPhase* phase)
    : phase(phase)
{
    commandLevel_ = 0;
}
        

void TaskPhaseProxy::setCommandLevel(int level)
{
    commandLevel_ = level;
}
    

TaskCommand* TaskPhaseProxy::addCommand()
{
    TaskCommand* command = phase->addCommand();
    command->setLevel(commandLevel_);
    return command;
}


TaskCommand* TaskPhaseProxy::addCommand(const std::string& caption)
{
    TaskCommand* command = phase->addCommand(caption);
    command->setLevel(commandLevel_);
    return command;
}


TaskCommand* TaskPhaseProxy::addToggleCommand()
{
    return addCommand()->setCheckable();
}


TaskCommand* TaskPhaseProxy::addToggleCommand(const std::string& caption)
{
    return addCommand(caption)->setCheckable();
}


TaskMenu::~TaskMenu()
{

}


Task::Task()
{
    addPhase("");
    addCommand("Start")->linkToNextPhase();
}


Task::Task(const std::string& name, const std::string& caption)
    : name_(name),
      caption_(caption)
{
    addPhase(caption);
    addCommand("Start")->linkToNextPhase();
}


Task::Task(const Task& org, bool doDeepCopy)
    : name_(org.name_),
      caption_(org.caption_),
      phases_(org.phases_)
{
    if(doDeepCopy){
        for(size_t i=0; i < phases_.size(); ++i){
            phases_[i] = new TaskPhase(*phases_[i], true);
        }
    }
}


Task::~Task()
{

}


void Task::setName(const std::string& str)
{
    name_ = str;
}


void Task::setCaption(const std::string& str)
{
    caption_ = str;
    if(numPhases() > 0){
        phase(0)->setCaption(caption_);
    }
}
        
        
TaskPhase* Task::phase(int index)
{
    if(index >= 0 && index < static_cast<int>(phases_.size())){
        return phases_[index];
    }
    return 0;
}

TaskPhase* Task::addPhase(TaskPhase* phase)
{
    phases_.push_back(phase);
    return phase;
}


TaskPhase* Task::addPhase(const std::string& caption)
{
    TaskPhase* phase = new TaskPhase(caption);
    addPhase(phase);
    return phase;
}


TaskPhase* Task::lastPhase()
{
    if(!phases_.empty()){
        return phases_.back();
    }
    return 0;
}


void Task::setPreCommand(TaskFunc func)
{
    if(TaskPhase* last = lastPhase()){
        last->setPreCommand(func);
    }
}


TaskCommand* Task::addCommand()
{
    if(TaskPhase* last = lastPhase()){
        return last->addCommand();
    }
    return 0;
}


TaskCommand* Task::addCommand(const std::string& caption)
{
    if(TaskPhase* last = lastPhase()){
        return last->addCommand(caption);
    }
    return 0;
}


TaskCommand* Task::addToggleCommand()
{
    if(TaskCommand* command = addCommand()){
        return command->setCheckable();
    }
    return 0;
}


TaskCommand* Task::addToggleCommand(const std::string& caption)
{
    if(TaskCommand* command = addCommand(caption)){
        return command->setCheckable();
    }
    return 0;
}


TaskCommand* Task::lastCommand()
{
    if(TaskPhase* last = lastPhase()){
        return last->lastCommand();
    }
    return 0;
}


int Task::lastCommandIndex()
{
    if(TaskPhase* last = lastPhase()){
        return last->lastCommandIndex();
    }
    return -1;
}


TaskPhaseProxyPtr Task::commandLevel(int level)
{
    if(TaskPhase* last = lastPhase()){
        return last->commandLevel(level);
    }
    return 0;
}


int Task::maxCommandLevel() const
{
    if(!phases_.empty()){
        return phases_.back()->maxCommandLevel();
    }
    return -1;
}


namespace {
    struct FuncToSetCommandLink {
        int commandIndex;
        FuncToSetCommandLink(int commandIndex) : commandIndex(commandIndex) { }
        void operator()(TaskProc* proc) {
            proc->setNextCommand(commandIndex);
        }
    };
}


TaskFunc Task::funcToSetCommandLink(int commandIndex) const
{
    return FuncToSetCommandLink(commandIndex);
}


void Task::onMenuRequest(TaskMenu& /* menu */)
{

}


void Task::onActivated(AbstractTaskSequencer* /* sequencer */)
{

}


void Task::onDeactivated(AbstractTaskSequencer* /* sequencer */)
{

}


void Task::storeState(AbstractTaskSequencer* sequencer, Mapping& archive)
{
    archive.write("phaseIndex", sequencer->currentPhaseIndex());
}


void Task::restoreState(AbstractTaskSequencer* sequencer, const Mapping& archive)
{
    sequencer->setCurrentPhase(archive.get("phaseIndex", 0));
}
