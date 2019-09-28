/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_TASK_VIEW_H
#define CNOID_BASE_TASK_VIEW_H

#include <cnoid/View>
#include <cnoid/AbstractTaskSequencer>
#include <vector>
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
    virtual bool removeTask(Task* task);
    virtual SignalProxy<void(Task* task)> sigTaskAdded();
    virtual SignalProxy<void(Task* task)> sigTaskRemoved();
    virtual void clearTasks();
    virtual int numTasks() const;
    virtual Task* task(int index);
    virtual int currentTaskIndex() const;
    virtual bool setCurrentTask(int taskIndex);
    virtual SignalProxy<void()> sigCurrentTaskChanged();
    virtual int currentPhaseIndex() const;
    virtual void setCurrentPhase(int phaseIndex);
    virtual SignalProxy<void()> sigCurrentPhaseChanged();
    virtual int currentCommandIndex() const;
    virtual void executeCommand(int commandIndex);
    virtual SignalProxy<void()> sigCurrentCommandChanged();
    virtual bool isBusy() const;
    virtual SignalProxy<void()> sigBusyStateChanged();
    virtual void cancelCurrentCommand();
    virtual SignalProxy<void()> sigCurrentCommandCanceled();
    virtual bool isAutoMode() const;
    virtual void setAutoMode(bool on);
    virtual SignalProxy<void(bool isAutoMode)> sigAutoModeToggled();
    virtual void serializeTasks(const std::vector<std::string>& tasks);
    
    void setNoExecutionMode(bool on);
    bool isNoExecutionMode() const;
    void setCurrentCommand(int commandIndex, bool doExecution);
    void setBusyState(bool on);

    void executeMenuItem(int index);
    void checkMenuItem(int index, bool on);
    std::vector<bool> menuItemCheckStates() const;

    // valid for the no execution mode
    SignalProxy<void()> sigMenuRequest();
    void showMenu(std::vector<bool> checkStates);
    SignalProxy<void(int index)> sigMenuItemTriggered();
    SignalProxy<void(int index, bool on)> sigMenuItemToggled();

protected:
    virtual bool storeState(Archive& archive);
    virtual bool restoreState(const Archive& archive);
    
private:
    TaskViewImpl* impl;
};

}

#endif
