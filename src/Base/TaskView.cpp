/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include "TaskView.h"
#include <cnoid/ViewManager>
#include <cnoid/MenuManager>
#include <cnoid/MessageView>
#include <cnoid/Button>
#include <cnoid/ComboBox>
#include <cnoid/Timer>
#include <cnoid/LazyCaller>
#include <QBoxLayout>
#include <QLabel>
#include <QEventLoop>
#include <QApplication>
#include <boost/bind.hpp>
#include <set>
#include <iostream>
#include "gettext.h"

using namespace std;
using namespace boost;
using namespace cnoid;

namespace cnoid {

class TaskViewImpl : public TaskProc, public TaskMenu
{
public:
    TaskView* self;
    MessageView* mv;
    ostream& os;
    ComboBox taskCombo;
    PushButton menuButton;
    PushButton prevButton;
    PushButton cancelButton;
    PushButton phaseIndexLabelButton;
    ToggleButton autoModeToggle;
    PushButton nextButton;
    QLabel phaseLabel;
    QHBoxLayout commandButtonBox;
    vector<PushButton*> commandButtons;
    MenuManager menuManager;

    std::vector<TaskPtr> tasks;
    TaskPtr currentTask;
    int currentTaskIndex;
    TaskPhasePtr currentPhase;
    boost::optional<int> currentCommandIndex;
    int currentPhaseIndex_;
    boost::optional<int> nextPhaseIndex;
    boost::optional<int> nextCommandIndex; // -1 means the pre-command
    bool forceCommandLinkAutomatic;    
    QEventLoop eventLoop;
    bool isPendingCommandCompleted;
    Connection commandConnection;
    Timer commandTimer;
    Timer waitTimer;

    TaskViewImpl(TaskView* self);
    ~TaskViewImpl();
    void addTask(Task* task);
    bool updateTask(Task* task);
    void setCurrentTask(int index);
    PushButton* getOrCreateCommandButton(int index);
    void enableCommandButtons(bool on);
    void setFocusToCommandButton(int commandIndex);
    int getClosestNextPhaseIndex(int phaseIndex);
    int getLoopBackPhaseIndex(int phaseIndex);
    void setPhaseIndex(int index, bool isSuccessivelyCalled);
    void executeCommandSuccessively(int commandIndex);
    void goToNextCommand();    
    void retry();
    void onNextOrPrevButtonClicked(int direction);

    virtual int currentPhaseIndex() const;
    virtual void setCurrentPhaseIndex(int index);
    virtual void breakSequence();
    virtual void setNextCommand(int commandIndex);
    virtual void setNextPhase(int phaseIndex);
    virtual void setCommandLinkAutomatic();
    virtual bool executeCommand(int index);
    virtual bool wait(double sec);
    virtual bool waitForCommandToFinish(double timeout);
    virtual bool waitForCommandToFinish(Connection connection, double timeout);
    virtual void notifyCommandFinish(bool isCompleted);

    bool isWaiting() const;
    bool cancelWaiting();
    bool stopWaiting(bool isCompleted);
    void onWaitTimeout();

    void onMenuButtonClicked();
    virtual void addMenuItem(const std::string& caption, boost::function<void()> func);
    virtual void addCheckMenuItem(const std::string& caption, bool isChecked, boost::function<void(bool on)> func);
    virtual void addMenuSeparator();
};

}

void TaskView::initializeClass(ExtensionManager* ext)
{
    ext->viewManager().registerClass<TaskView>(
        "TaskView", N_("Task"), ViewManager::SINGLE_OPTIONAL);
}


TaskView* TaskView::instance()
{
    static TaskView* instance_ = ViewManager::getOrCreateView<TaskView>();
    return instance_;
}


TaskView::TaskView()
{
    impl = new TaskViewImpl(this);
}


TaskViewImpl::TaskViewImpl(TaskView* self)
    : self(self),
      mv(MessageView::instance()),
      os(mv->cout())
{
    self->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
    self->setDefaultLayoutArea(View::CENTER);
    QVBoxLayout* vbox = new QVBoxLayout;
    QHBoxLayout* hbox = new QHBoxLayout;

    taskCombo.setToolTip(_("Select a task type"));
    taskCombo.addItem("  ----------  ");
    taskCombo.sigCurrentIndexChanged().connect(
        boost::bind(&TaskViewImpl::setCurrentTask, this, _1));
    hbox->addWidget(&taskCombo, 1);

    menuButton.setText("*");
    menuButton.setToolTip(_("Option Menu"));
    menuButton.sigClicked().connect(boost::bind(&TaskViewImpl::onMenuButtonClicked, this));
    hbox->addWidget(&menuButton);
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout;
    prevButton.setText("<");
    prevButton.setToolTip(_("Go back to the previous phase"));
    prevButton.sigClicked().connect(boost::bind(&TaskViewImpl::onNextOrPrevButtonClicked, this, -1));
    hbox->addWidget(&prevButton);

    cancelButton.setText(_("Cancel"));
    cancelButton.setToolTip(_("Cancel waiting for the command to finish"));
    cancelButton.setEnabled(false);
    cancelButton.sigClicked().connect(boost::bind(&TaskViewImpl::cancelWaiting, this));
    hbox->addWidget(&cancelButton);

    phaseIndexLabelButton.setToolTip(_("Execute the default command of the current phase"));
    phaseIndexLabelButton.sigClicked().connect(boost::bind(&TaskViewImpl::executeCommandSuccessively, this, -1));
    hbox->addWidget(&phaseIndexLabelButton);

    autoModeToggle.setText(_("Auto"));
    autoModeToggle.setToolTip(_("Automatic mode"));
    hbox->addWidget(&autoModeToggle);

    nextButton.setText(">");
    nextButton.setToolTip(_("Skip to the next phase"));
    nextButton.sigClicked().connect(boost::bind(&TaskViewImpl::onNextOrPrevButtonClicked, this, +1));
    hbox->addWidget(&nextButton);
    vbox->addLayout(hbox);

    vbox->addWidget(&phaseLabel, 0, Qt::AlignHCenter);
    vbox->addLayout(&commandButtonBox);
    vbox->addStretch();
    self->setLayout(vbox);

    currentTaskIndex = 0;
    currentPhaseIndex_ = 0;
    forceCommandLinkAutomatic = false;
    
    commandTimer.setSingleShot(true);
    commandTimer.sigTimeout().connect(bind(&TaskViewImpl::cancelWaiting, this));

    waitTimer.setSingleShot(true);
    waitTimer.sigTimeout().connect(bind(&TaskViewImpl::onWaitTimeout, this));

    setPhaseIndex(0, false);
}


TaskView::~TaskView()
{
    delete impl;
}


TaskViewImpl::~TaskViewImpl()
{

}


void TaskView::addTask(Task* task)
{
    impl->addTask(task);
}


void TaskViewImpl::addTask(Task* task)
{
    taskCombo.blockSignals(true);

    if(tasks.empty()){
        taskCombo.clear();
    }
    tasks.push_back(task);

    taskCombo.addItem(task->name().c_str());
    
    taskCombo.blockSignals(false);

    if(tasks.size() == 1){
        setCurrentTask(0);
    }

    os << format(_("Task \"%1%\" has been added.")) % task->name() << endl;
}


bool TaskView::updateTask(Task* task)
{
    return impl->updateTask(task);
}


bool TaskViewImpl::updateTask(Task* task)
{
    bool updated = false;
    
    int index = taskCombo.findText(task->name().c_str());

    if(index >= 0 && index < tasks.size()){
        if(isWaiting()){
            mv->putln(MessageView::WARNING,
                      format(_("Task \"%1%\" cannot be updated now because it is wating for a command to finish."))
                      % task->name());
        } else {
            MappingPtr state;
            if(index == currentTaskIndex){
                TaskPtr oldTask = tasks[index];
                state = new Mapping();
                oldTask->storeState(this, *state);
            }
            tasks[index] = task;
            if(state){
                setCurrentTask(index);
                task->restoreState(this, *state);
            }
            os << format(_("Task \"%1%\" has been updated with the new one.")) % task->name() << endl;
            updated = true;
        }
    } else {
        addTask(task);
    }

    return updated;
}


void TaskViewImpl::setCurrentTask(int index)
{
    if(index < 0 || index >= tasks.size()){
        return;
    }

    if(taskCombo.currentIndex() != index){
        taskCombo.blockSignals(true);
        taskCombo.setCurrentIndex(index);
        taskCombo.blockSignals(false);
    }

    currentTaskIndex = index;
    currentTask = tasks[index];
    setPhaseIndex(0, false);
}


PushButton* TaskViewImpl::getOrCreateCommandButton(int commandIndex)
{
    PushButton* button;
    if(commandIndex < commandButtons.size()){
        button = commandButtons[commandIndex];
    } else {
        button = new PushButton;
        button->sigClicked().connect(boost::bind(&TaskViewImpl::executeCommandSuccessively, this, commandIndex));
        commandButtonBox.addWidget(button);
        if(commandButtons.empty()){
            QWidget::setTabOrder(&menuButton, button);
        } else {
            QWidget::setTabOrder(commandButtons.back(), button);
        }
        commandButtons.push_back(button);
    }
    return button;
}


void TaskViewImpl::enableCommandButtons(bool on)
{
    for(size_t i=0; i < commandButtons.size(); ++i){
        commandButtons[i]->setDown(false);
        commandButtons[i]->setEnabled(on);
    }
    if(!on && currentCommandIndex){
        commandButtons[*currentCommandIndex]->setDown(true);
    }
    cancelButton.setEnabled(!on);
}


void TaskViewImpl::setFocusToCommandButton(int commandIndex)
{
    if(commandIndex >= 0 && commandIndex < commandButtons.size()){
        QWidget* widget = QApplication::focusWidget();
        while(widget){
            if(widget == self){
                commandButtons[commandIndex]->setFocus(Qt::OtherFocusReason);
                break;
            }
            widget = widget->parentWidget();
        }
    }
}

int TaskViewImpl::currentPhaseIndex() const
{
    return currentPhaseIndex_;
}


int TaskViewImpl::getClosestNextPhaseIndex(int phaseIndex)
{
    int closest = phaseIndex;
    int distance = std::numeric_limits<int>::max();
    if(currentTask){
        TaskPhase* phase = currentTask->phase(phaseIndex);
        if(phase){
            for(int i = 0; i < phase->numCommands(); ++i){
                TaskCommand* command = phase->command(i);
                if(command){
                    int nextIndex = command->nextPhaseIndex(phaseIndex);
                    if(nextIndex != phaseIndex && abs(nextIndex - phaseIndex) < distance){
                        closest = nextIndex;
                        distance = abs(nextIndex - phaseIndex);
                    }
                }
            }
        }
    }
    return closest;
}


int TaskViewImpl::getLoopBackPhaseIndex(int phaseIndex)
{
    int loopBackIndex = phaseIndex;
    if(currentTask){
        TaskPhase* phase = currentTask->phase(phaseIndex);
        if(phase){
            for(int i = phase->numCommands() - 1; i >=0; --i){
                TaskCommand* command = phase->command(i);
                if(command){
                    int nextIndex = command->nextPhaseIndex(phaseIndex);
                    if(nextIndex < phaseIndex){
                        loopBackIndex = nextIndex;
                        break;
                    }
                }
            }
        }
    }
    return loopBackIndex;
}


void TaskViewImpl::setCurrentPhaseIndex(int index)
{
    setPhaseIndex(index, false);
}


void TaskViewImpl::setPhaseIndex(int index, bool isSuccessivelyCalled)
{
    cancelWaiting();
    
    int numPhases = 0;
    if(currentTask){
        numPhases = currentTask->numPhases();
    }

    currentPhaseIndex_ = std::max(0, std::min(index, numPhases));

    // check if the phase should be skipped
    if(isSuccessivelyCalled){
        std::set<int> skippedIndices;
        while(true){
            if(skippedIndices.find(currentPhaseIndex_) == skippedIndices.end()){
                if(currentPhaseIndex_ >= 0 && currentPhaseIndex_ < numPhases){
                    TaskPhase* phase = currentTask->phase(currentPhaseIndex_);
                    if(phase->isSkipped()){
                        skippedIndices.insert(currentPhaseIndex_);
                        currentPhaseIndex_ = getClosestNextPhaseIndex(currentPhaseIndex_);
                        continue;
                    }
                }
            }
            break;
        }
    }

    prevButton.setEnabled(currentPhaseIndex_ > 0);
    nextButton.setEnabled(
        (currentPhaseIndex_ < numPhases - 1) ||
        (getLoopBackPhaseIndex(currentPhaseIndex_) < numPhases - 1));

    int numVisibleButtons = 0;
    if(!numPhases){
        phaseLabel.setText("No phase");
        PushButton* button = getOrCreateCommandButton(0);
        button->setText("-");
        button->setEnabled(false);
        button->show();
        currentPhase = 0;
        numVisibleButtons = 1;
    } else {
        currentPhase = currentTask->phase(currentPhaseIndex_);
        phaseLabel.setText(currentPhase->caption().c_str());
        numVisibleButtons = currentPhase->numCommands();
        for(int i=0; i < numVisibleButtons; ++i){
            PushButton* button = getOrCreateCommandButton(i);
            button->setText(currentPhase->command(i)->caption().c_str());
            button->setEnabled(true);
            button->show();
        }
    }

    for(size_t i=numVisibleButtons; i < commandButtons.size(); ++i){
        commandButtons[i]->hide();
    }
    
    phaseIndexLabelButton.setText(QString("%1 / %2").arg(currentPhaseIndex_).arg(numPhases - 1));

    currentCommandIndex = boost::none;

    if(isSuccessivelyCalled && currentPhase){
        if(nextCommandIndex){
            if(*nextCommandIndex < 0 || autoModeToggle.isChecked()){
                executeCommandSuccessively(*nextCommandIndex);
            }
        }
    }
}


void TaskViewImpl::executeCommandSuccessively(int commandIndex)
{
    cancelWaiting();
    
    currentCommandIndex = boost::none;
    nextCommandIndex = boost::none;
    
    if(currentTask && currentPhase){
        nextPhaseIndex = currentPhaseIndex_;
        TaskFunc commandFunc;
        if(commandIndex < 0){
            commandFunc = currentPhase->preCommand();
            if(!commandFunc){
                for(int i=0; i < currentPhase->numCommands(); ++i){
                    TaskCommand* command = currentPhase->command(i);
                    if(command->isDefault()){
                        commandIndex = i;
                        break;
                    }
                }
            }
        }
        if(commandIndex >= 0){
            currentCommandIndex = commandIndex;
            TaskCommand* command = currentPhase->command(commandIndex);
            if(command){
                commandFunc = command->function();
                nextPhaseIndex = command->nextPhaseIndex(currentPhaseIndex_);
                nextCommandIndex = command->nextCommandIndex(commandIndex);
            }
        }
        if(commandFunc){
            commandFunc(this);
        }
        setFocusToCommandButton(commandIndex < 0 ? 0 : commandIndex);

        goToNextCommand();
    }
}


void TaskViewImpl::goToNextCommand()
{
    if(!eventLoop.isRunning()){
        if(nextPhaseIndex && *nextPhaseIndex != currentPhaseIndex_){
            nextCommandIndex = -1;
            callLater(boost::bind(&TaskViewImpl::setPhaseIndex, this, *nextPhaseIndex, true));

        } else {
            if(nextCommandIndex &&
                  (!currentCommandIndex || *nextCommandIndex != *currentCommandIndex)){
                int index = *nextCommandIndex;
                nextCommandIndex = boost::none;
                setFocusToCommandButton(index);
                bool executeNext = autoModeToggle.isChecked();
                if(!executeNext){
                    if(currentCommandIndex){
                        TaskCommand* command = currentPhase->command(*currentCommandIndex);
                        if(command && command->isCommandLinkAutomatic()){
                            executeNext = true;
                        }
                    } else if(forceCommandLinkAutomatic){
                        executeNext = true;
                    }
                }
                if(executeNext){
                    callLater(boost::bind(&TaskViewImpl::executeCommandSuccessively, this, index));
                }
            }
        }
    }
    forceCommandLinkAutomatic = false;
}


void TaskViewImpl::retry()
{
    breakSequence();
    nextCommandIndex = -1;
    setPhaseIndex(0, false);
}


void TaskViewImpl::onNextOrPrevButtonClicked(int direction)
{
    int index = currentPhaseIndex_ + direction;
    if(currentPhase && index == currentTask->numPhases()){
        index = getLoopBackPhaseIndex(currentPhaseIndex_);
    }
    setPhaseIndex(index, false);
}


void TaskViewImpl::breakSequence()
{
    nextPhaseIndex = boost::none;
    nextCommandIndex = boost::none;

    mv->putln(MessageView::HIGHLIGHT,
              "Transition to the next task-command was interrupted.");
}


void TaskViewImpl::setNextCommand(int commandIndex)
{
    nextCommandIndex = commandIndex;
}


void TaskViewImpl::setNextPhase(int phaseIndex)
{
    nextPhaseIndex = phaseIndex;
}


void TaskViewImpl::setCommandLinkAutomatic()
{
    forceCommandLinkAutomatic = true;
}


bool TaskViewImpl::executeCommand(int commandIndex)
{
    bool completed = false;
    if(currentPhase){
        TaskCommand* command = currentPhase->command(commandIndex);
        if(command){
            TaskFunc func = command->function();
            if(func){
                boost::optional<int> callerCommandIndex = currentCommandIndex;
                currentCommandIndex = commandIndex;
                isPendingCommandCompleted = false;
                func(this);
                QCoreApplication::processEvents(QEventLoop::AllEvents, 10);
                currentCommandIndex = callerCommandIndex;
                completed = isPendingCommandCompleted;
            }
        }
    }
    return completed;
}


bool TaskViewImpl::wait(double sec)
{
    bool completed = false;
    if(!eventLoop.isRunning()){
        enableCommandButtons(false);
        waitTimer.start(sec * 1000.0);
        isPendingCommandCompleted = false;
        eventLoop.exec();
        completed = isPendingCommandCompleted;
        enableCommandButtons(true);
    }
    if(!completed){
        breakSequence();
    }
    return completed;
}


bool TaskViewImpl::waitForCommandToFinish(double timeout)
{
    bool completed = false;
    if(!eventLoop.isRunning()){
        enableCommandButtons(false);
        isPendingCommandCompleted = false;
        if(timeout > 0.0){
            commandTimer.start(timeout * 1000.0);
        } else {
            commandTimer.stop();
        }
        eventLoop.exec();
        completed = isPendingCommandCompleted;
        enableCommandButtons(true);
    }
    if(!completed){
        breakSequence();
    }
    return completed;
}


bool TaskViewImpl::waitForCommandToFinish(Connection connection, double timeout)
{
    bool completed = false;
    if(!eventLoop.isRunning()){
        commandConnection = connection;
        completed = waitForCommandToFinish(timeout);
    }
    commandConnection.disconnect();
    return completed;
}


void TaskViewImpl::notifyCommandFinish(bool isCompleted)
{
    stopWaiting(isCompleted);
}


bool TaskViewImpl::isWaiting() const
{
    return eventLoop.isRunning();
}


bool TaskViewImpl::cancelWaiting()
{
    if(eventLoop.isRunning()){
        autoModeToggle.setChecked(false); // stop the auto mode, too
        nextPhaseIndex = boost::none;
        nextCommandIndex = boost::none;
        return stopWaiting(false);
    }
    return false;
}


bool TaskViewImpl::stopWaiting(bool isCompleted)
{
    if(eventLoop.isRunning()){
        isPendingCommandCompleted = isCompleted;
        eventLoop.quit();
        waitTimer.stop();
        commandTimer.stop();
        return true;
    }
    return false;
}


void TaskViewImpl::onWaitTimeout()
{
    if(eventLoop.isRunning()){
        isPendingCommandCompleted = true;
        eventLoop.quit();
    }
}


void TaskViewImpl::onMenuButtonClicked()
{
    menuManager.setNewPopupMenu(self);
    if(currentTask){
        currentTask->onMenuRequest(*this);
    }
    if(menuManager.numItems() > 0){
        menuManager.addSeparator();
    }
    menuManager.addItem(_("Retry"))->sigTriggered().connect(bind(&TaskViewImpl::retry, this));
    
    menuManager.popupMenu()->popup(menuButton.mapToGlobal(QPoint(0,0)));
}


void TaskViewImpl::addMenuItem(const std::string& caption, boost::function<void()> func)
{
    Action* action = menuManager.addItem(caption.c_str());
    if(func){
        action->sigTriggered().connect(func);
    }
}


void TaskViewImpl::addCheckMenuItem(const std::string& caption, bool isChecked, boost::function<void(bool on)> func)
{
    Action* action = menuManager.addCheckItem(caption.c_str());
    action->setChecked(isChecked);
    if(func){
        action->sigToggled().connect(func);
    }
}


void TaskViewImpl::addMenuSeparator()
{
    menuManager.addSeparator();
}


bool TaskView::storeState(Archive& archive)
{
    archive.write("isAutoMode", impl->autoModeToggle.isChecked());
    return true;
}


bool TaskView::restoreState(const Archive& archive)
{
    impl->autoModeToggle.setChecked(archive.get("isAutoMode", false));
    return true;
}
