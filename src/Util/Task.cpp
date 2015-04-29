/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include "Task.h"
#include "AbstractTaskSequencer.h"
#include "ValueTree.h"

using namespace cnoid;


TaskProc::~TaskProc()
{

}

TaskCommand::TaskCommand(const std::string& caption)
    : caption_(caption)
{
    nextPhaseIndex_ = 0;
    nextCommandIndex_ = 0;
    isNextPhaseRelative_ = true;
    isNextCommandRelative_ = true;
    isCommandLinkAutomatic_ = false;
    isDefault_ = false;
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


TaskCommand* TaskPhase::addCommand(const std::string& caption)
{
    TaskCommand* command = new TaskCommand(caption);
    commands.push_back(command);
    return command;
}


TaskCommand* TaskPhase::command(int index) const
{
    TaskCommand* command = 0;
    if(index >= 0 && index < commands.size()){
        command = commands[index];
    }
    return command;
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
    if(index >= 0 && index < phases_.size()){
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
    TaskPhase* last = lastPhase();
    if(last){
        last->setPreCommand(func);
    }
}


TaskCommand* Task::addCommand(const std::string& caption)
{
    TaskPhase* last = lastPhase();
    if(last){
        return last->addCommand(caption);
    }
    return 0;
}


TaskCommand* Task::lastCommand()
{
    TaskPhase* last = lastPhase();
    if(last){
        return last->lastCommand();
    }
    return 0;
}


int Task::lastCommandIndex()
{
    TaskPhase* last = lastPhase();
    if(last){
        return last->lastCommandIndex();
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


void Task::onMenuRequest(TaskMenu& menu)
{

}


bool Task::storeState(AbstractTaskSequencer* sequencer, Mapping& archive)
{
    archive.write("phaseIndex", sequencer->currentPhaseIndex());
    return true;
}


bool Task::restoreState(AbstractTaskSequencer* sequencer, const Mapping& archive)
{
    sequencer->setCurrentPhase(archive.get("phaseIndex", 0));
    return true;
}
