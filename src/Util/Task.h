/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_TASK_H
#define CNOID_UTIL_TASK_H

#include <cnoid/Referenced>
#include <cnoid/Signal>
#include <functional>
#include <vector>
#include <string>
#include "exportdecl.h"

namespace cnoid {

class Mapping;
class AbstractTaskSequencer;

class CNOID_EXPORT TaskProc
{
public:
    virtual ~TaskProc();
    virtual int currentPhaseIndex() const = 0;
    virtual bool isAutoMode() const = 0;
    virtual void breakSequence() = 0;
    virtual void setNextCommand(int commandIndex) = 0;
    virtual void setNextPhase(int phaseIndex) = 0;
    virtual void setCommandLinkAutomatic() = 0;    
    virtual bool executeCommand(int index) = 0;
    virtual bool wait(double sec) = 0;
    virtual bool waitForCommandToFinish(double timeout = 0.0) = 0;
    virtual bool waitForCommandToFinish(Connection connectionToDisconnect, double timeout) = 0;
    virtual void notifyCommandFinish(bool isCompleted = true) = 0;

    bool waitForSignal(SignalProxy<void()> signalProxy, double timeout = 0.0);
    bool waitForBooleanSignal(SignalProxy<void(bool)> signalProxy, double timeout = 0.0);
};


typedef std::function<void(TaskProc* proc)> TaskFunc;


class CNOID_EXPORT TaskToggleState : public Referenced
{
public:
    bool isChecked() const { return isChecked_; }
    void setChecked(bool on);

    SignalProxy<void(bool on)> sigToggled() { return sigToggled_; }

private:
    bool isChecked_;
    Signal<void(bool on)> sigToggled_;
};

typedef ref_ptr<TaskToggleState> TaskToggleStatePtr;


class CNOID_EXPORT TaskCommand : public Referenced
{
public:
    TaskCommand();
    TaskCommand(const std::string& caption);
    ~TaskCommand();
    
    const std::string& caption() const { return caption_; }
    TaskCommand* setCaption(const std::string& caption){
        caption_ = caption;
        return this;
    }

    const std::string& description() const { return description_; }
    TaskCommand* setDescription(const std::string& description) {
        description_ = description; return this; }
    
    TaskFunc function() const { return function_; }
    TaskCommand* setFunction(TaskFunc func) { function_ = func; return this; }

    TaskCommand* setDefault(bool on = true) { isDefault_ = on; return this; }
    bool isDefault() const { return isDefault_; }

    TaskCommand* setCheckable(bool on = true);
    TaskCommand* setToggleState(TaskToggleState* state);
    TaskToggleState* toggleState();
    TaskCommand* setChecked(bool on);
    bool isChecked() const;
    
    int nextPhaseIndex(int currentPhaseIndex) const;
    TaskCommand* setPhaseLink(int phaseIndex);
    TaskCommand* setPhaseLinkStep(int phaseIndexStep);
    TaskCommand* linkToNextPhase() { setPhaseLinkStep(1); return this; }

    int nextCommandIndex(int currentCommandIndex) const;
    TaskCommand* setCommandLink(int commandIndex);
    TaskCommand* setCommandLinkStep(int commandIndexStep);
    TaskCommand* linkToNextCommand() { setCommandLinkStep(1); return this; }
    bool isCommandLinkAutomatic() const { return isCommandLinkAutomatic_; }
    TaskCommand* setCommandLinkAutomatic(bool on = true) { isCommandLinkAutomatic_ = on; return this; }

    TaskCommand* setLevel(int level) { level_ = level; return this; }
    int level() const { return level_; }

    TaskCommand* linkToNextTask();

private:
    std::string caption_;
    std::string description_;
    TaskFunc function_;
    int nextPhaseIndex_;
    int nextCommandIndex_;
    int level_;
    TaskToggleStatePtr toggleState_;
    bool isNextPhaseRelative_;
    bool isNextCommandRelative_;
    bool isCommandLinkAutomatic_;
    bool isDefault_;

    void initialize();
};

typedef ref_ptr<TaskCommand> TaskCommandPtr;


class TaskPhaseProxy;
typedef ref_ptr<TaskPhaseProxy> TaskPhaseProxyPtr;


class CNOID_EXPORT TaskPhase : public Referenced
{
public:
    TaskPhase(const std::string& caption);
    TaskPhase(const TaskPhase& org, bool doDeepCopy = true);
    ~TaskPhase();

    virtual TaskPhase* clone(bool doDeepCopy = true);

    const std::string& caption() const { return caption_; }
    void setCaption(const std::string& str);

    bool isSkipped() const { return isSkipped_; }
    void setSkipped(bool on) { isSkipped_ = on; }
    
    void setPreCommand(TaskFunc func);
    TaskFunc preCommand() const { return preCommand_; }

    TaskCommand* addCommand();
    TaskCommand* addCommand(const std::string& caption);
    TaskCommand* addToggleCommand();
    TaskCommand* addToggleCommand(const std::string& caption);
    int numCommands() const { return commands.size(); }
    TaskCommand* command(int index) const;
    int lastCommandIndex() const { return commands.size() - 1; }
    TaskCommand* lastCommand() const { return command(commands.size() - 1); }

    TaskPhaseProxyPtr commandLevel(int level);
    int maxCommandLevel() const;

private:
    std::string caption_;
    TaskFunc preCommand_;
    std::vector<TaskCommandPtr> commands;
    bool isSkipped_;
};

typedef ref_ptr<TaskPhase> TaskPhasePtr;


class CNOID_EXPORT TaskPhaseProxy : public Referenced
{
public:
    TaskPhaseProxy(TaskPhase* phase);
        
    void setCommandLevel(int level);
    int commandLevel() const { return commandLevel_; }
    
    TaskCommand* addCommand();
    TaskCommand* addCommand(const std::string& caption);
    TaskCommand* addToggleCommand();
    TaskCommand* addToggleCommand(const std::string& caption);

private:
    TaskPhasePtr phase;
    int commandLevel_;
};


/**
   \todo The menu API should be moved to the actual implementation class of AbstractTaskSequencer
*/
class CNOID_EXPORT TaskMenu
{
public:
    virtual ~TaskMenu();
    virtual void addMenuItem(const std::string& caption, std::function<void()> func) = 0;
    virtual void addCheckMenuItem(const std::string& caption, bool isChecked, std::function<void(bool on)> func) = 0;
    virtual void addMenuSeparator() = 0;
};


class CNOID_EXPORT Task : public Referenced
{    
public:
    Task();
    Task(const std::string& name, const std::string& caption);
    Task(const Task& org, bool doDeepCopy = true);
    ~Task();

    const std::string& name() const { return name_; }
    void setName(const std::string& str);
    const std::string& caption() const { return caption_; }
    void setCaption(const std::string& str);

    int numPhases() const { return phases_.size(); }
    TaskPhase* phase(int index);

    TaskPhase* addPhase(TaskPhase* phase);
    TaskPhase* addPhase(const std::string& caption);
    TaskPhase* lastPhase();

    // The following functions do operations to the last added phase
    void setPreCommand(TaskFunc func);
    TaskCommand* addCommand();
    TaskCommand* addCommand(const std::string& caption);
    TaskCommand* addToggleCommand();
    TaskCommand* addToggleCommand(const std::string& caption);
    TaskCommand* lastCommand();
    int lastCommandIndex();
    TaskPhaseProxyPtr commandLevel(int level);
    int maxCommandLevel() const;

    TaskFunc funcToSetCommandLink(int commandIndex) const;

    virtual void onActivated(AbstractTaskSequencer* sequencer);
    virtual void onDeactivated(AbstractTaskSequencer* sequencer);
    virtual void storeState(AbstractTaskSequencer* sequencer, Mapping& archive);
    virtual void restoreState(AbstractTaskSequencer* sequencer, const Mapping& archive);
    
    //! \todo The menu API should be moved to the actual implementation class of AbstractTaskSequencer
    virtual void onMenuRequest(TaskMenu& menu);
    
private:
    std::string name_;
    std::string caption_;
    std::vector<TaskPhasePtr> phases_;
};

typedef ref_ptr<Task> TaskPtr;

}

#endif
