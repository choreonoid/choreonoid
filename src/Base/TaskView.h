/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_TASK_VIEW_H
#define CNOID_BASE_TASK_VIEW_H

#include <cnoid/View>
#include <cnoid/AbstractTaskSequencer>
#include "exportdecl.h"

namespace cnoid {

class TaskViewImpl;

class CNOID_EXPORT TaskView : public View, public AbstractTaskSequencer
{
public:
    static void initializeClass(ExtensionManager* ext);
    static TaskView* instance();
    
    TaskView();
    ~TaskView();

    virtual void activate(bool on = true);
    virtual bool isActive();
    
    virtual void addTask(Task* task);
    virtual bool updateTask(Task* task);
    virtual int numTasks() const;
    virtual Task* task(int index);
    virtual int currentTaskIndex() const;
    virtual bool setCurrentTask(int taskIndex);
    virtual SignalProxy<void()> sigCurrentTaskChanged();
    virtual int currentPhaseIndex() const;
    virtual void setCurrentPhase(int phaseIndex);
    virtual SignalProxy<void()> sigCurrentPhaseChanged();
    virtual int currentCommandIndex() const;
    virtual SignalProxy<void()> sigCurrentCommandChanged();
    virtual bool isBusy() const;
    virtual SignalProxy<void()> sigBusyStateChanged();
    virtual void cancelCurrentCommand();
    virtual SignalProxy<void()> sigCurrentCommandCanceled();
    virtual bool isAutoMode() const;
    virtual void setAutoMode(bool on);
    virtual SignalProxy<void(bool isAutoMode)> sigAutoModeToggled();
    
    void setNoExecutionMode(bool on);
    bool isNoExecutionMode() const;
    void setCurrentCommand(int commandIndex, bool doExecution);
    void setBusyState(bool on);

protected:
    virtual bool storeState(Archive& archive);
    virtual bool restoreState(const Archive& archive);
    
private:
    TaskViewImpl* impl;
};

}

#endif
