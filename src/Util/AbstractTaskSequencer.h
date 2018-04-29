/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_ABSTRACT_TASK_SEQUENCER_H
#define CNOID_UTIL_ABSTRACT_TASK_SEQUENCER_H

#include "Task.h"
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT AbstractTaskSequencer
{
public:
    virtual ~AbstractTaskSequencer();

    virtual void activate(bool on = true) = 0;
    virtual bool isActive() = 0;
    
    virtual void addTask(Task* task) = 0;
    virtual bool updateTask(Task* task) = 0;
    virtual bool removeTask(Task* task) = 0;
    virtual void clearTasks() = 0;
    virtual SignalProxy<void(Task* task)> sigTaskAdded() = 0;
    virtual SignalProxy<void(Task* task)> sigTaskRemoved() = 0;
    virtual int numTasks() const = 0;
    virtual Task* task(int index) = 0;
    virtual int currentTaskIndex() const = 0;
    virtual bool setCurrentTask(int taskIndex) = 0;
    virtual SignalProxy<void()> sigCurrentTaskChanged() = 0;
    virtual int currentPhaseIndex() const = 0;
    virtual void setCurrentPhase(int phaseIndex) = 0;
    virtual SignalProxy<void()> sigCurrentPhaseChanged() = 0;

    /*
      @return The -1 value is returned when the current command is an implicit pre-processing command.
    */
    virtual int currentCommandIndex() const = 0;
    virtual void executeCommand(int commandIndex) = 0;
    virtual SignalProxy<void()> sigCurrentCommandChanged() = 0;
    virtual bool isBusy() const = 0;
    virtual SignalProxy<void()> sigBusyStateChanged() = 0;
    virtual void cancelCurrentCommand() = 0;
    virtual SignalProxy<void()> sigCurrentCommandCanceled() = 0;
    virtual bool isAutoMode() const = 0;
    virtual void setAutoMode(bool on) = 0;
    virtual SignalProxy<void(bool isAutoMode)> sigAutoModeToggled() = 0;

    virtual void serializeTasks(const std::vector<std::string>& tasks) = 0;
};

}

#endif
