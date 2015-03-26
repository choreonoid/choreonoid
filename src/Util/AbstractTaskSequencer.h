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
    
    virtual void addTask(Task* task) = 0;
    virtual bool updateTask(Task* task) = 0;
    virtual int numTasks() const = 0;
    virtual Task* task(int index) = 0;
    virtual int currentTaskIndex() const = 0;
    virtual SignalProxy<void(int index)> sigCurrentTaskIndexChanged() = 0;
    virtual int currentPhaseIndex() const = 0;

    /**
       \note This signal is emitted when the current task is changed as well.
    */
    virtual SignalProxy<void(int index)> sigCurrentPhaseIndexChanged() = 0;

    /*
      @return The -1 value is returned when the current command is an implicit pre-processing command.
    */
    virtual int currentCommandIndex() const = 0;

    /**
       \note This signal is emitted when the current task or phase is changed as well.
    */       
    virtual SignalProxy<void(int index)> sigCurrentCommandIndexChanged() = 0;

    virtual bool isBusy() const = 0;
    virtual SignalProxy<void(bool isBusy)> sigBusyStateChanged() = 0;
    virtual bool isAutoMode() const = 0;
};

}

#endif
