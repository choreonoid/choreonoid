/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_ABSTRACT_TASK_PROCESSOR_H
#define CNOID_UTIL_ABSTRACT_TASK_PROCESSOR_H

#include "Task.h"
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT AbstractTaskProcessor
{
public:
    virtual void addTask(Task* task) = 0;
    virtual bool updateTask(Task* task) = 0;
    virtual int numTasks() const = 0;
    virtual Task* task(int index) = 0;
    virtual int currentTaskIndex() const = 0;
    virtual SignalProxy<void(int index)> sigCurrentTaskIndexChanged() = 0;
    virtual int currentPhaseIndex() const = 0;
    virtual SignalProxy<void(int index)> sigCurrentPhaseIndexChanged() = 0;
    virtual int currentCommandIndex() const = 0;
    virtual SignalProxy<void(int index)> sigCurrentCommandIndexChanged() = 0;

    enum CommandState { IDLE, BUSY };
    virtual int commandState() const = 0;
    virtual SignalProxy<void(int state)> sigCommandStateChanged() = 0;
};

}

#endif
