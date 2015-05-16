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
#include <cnoid/SpinBox>
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

namespace {

const bool TRACE_FUNCTIONS = false;

}

namespace cnoid {

class TaskViewImpl : public TaskProc, public TaskMenu
{
public:
    TaskView* self;
    MessageView* mv;
    ostream& os;
    bool isVerticalLayout;
    QVBoxLayout topVBox;
    QHBoxLayout hbox1;
    QHBoxLayout hbox2;
    QHBoxLayout hbox3;
    QVBoxLayout vspace;
    ComboBox taskCombo;
    PushButton menuButton;
    PushButton prevButton;
    PushButton cancelButton;
    SpinBox phaseIndexSpin;
    Connection phaseIndexSpinConnection;
    ToolButton defaultCommandButton;
    ToggleButton autoModeToggle;
    PushButton nextButton;
    QLabel phaseLabel;
    QWidget commandButtonBox;
    QBoxLayout* commandButtonBoxLayout;
    vector<PushButton*> commandButtons;
    MenuManager menuManager;

    bool isNoExecutionMode;
    bool isActive;

    bool isExecutionEnabled() const { return !isNoExecutionMode && isActive; }
    bool isExecutionDisabled() const { return !isExecutionEnabled(); }

    struct TaskInfo {
        TaskPtr task;
        MappingPtr state;
        TaskInfo(Task* task) : task(task) { }
    };
    std::vector<TaskInfo> tasks;
    
    TaskPtr currentTask;
    int currentTaskIndex;
    Signal<void()> sigCurrentTaskChanged;
    TaskPhasePtr currentPhase;
    int currentPhaseIndex_;
    Signal<void()> sigCurrentPhaseChanged;
    boost::optional<int> nextPhaseIndex;
    enum { NO_CURRENT_COMMAND = -2, PRE_COMMAND = -1 };
    int currentCommandIndex;
    boost::optional<int> nextCommandIndex; // -1 means the pre-command

    LazyCaller goToNextCommandLater;
    
    Signal<void()> sigCurrentCommandChanged;
    Signal<void()> sigCurrentCommandCanceled;
    bool forceCommandLinkAutomatic;    
    bool isBusy;
    QEventLoop eventLoop;
    bool isPendingCommandCompleted;
    Connection commandConnection;
    Timer commandTimer;
    Timer waitTimer;
    Signal<void()> sigBusyStateChanged;

    TaskViewImpl(TaskView* self);
    ~TaskViewImpl();
    void doLayout(bool on);
    void activate(bool on, bool forceUpdate);
    void addTask(Task* task);
    bool updateTask(Task* task);
    bool setCurrentTask(int index, bool forceUpdate);
    void setCurrentTaskByName(const std::string& name);
    PushButton* getOrCreateCommandButton(int index);
    bool setCurrentCommandIndex(int index);
    void setBusyState(bool on);
    void setFocusToCommandButton(int commandIndex);
    int getClosestNextPhaseIndex(int phaseIndex);
    int getLoopBackPhaseIndex(int phaseIndex);
    void setPhaseIndex(int index, bool isSuccessivelyCalled);
    void executeCommandSuccessively(int commandIndex);
    void setTransitionToNextCommand();    
    void retry();
    void onCommandButtonClicked(int commandIndex);
    void onNextOrPrevButtonClicked(int direction);

    virtual int currentPhaseIndex() const;
    virtual bool isAutoMode() const;    
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
    void cancelWaiting(bool doBreak = false);
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

    currentTaskIndex = -1;
    currentPhaseIndex_ = 0;
    currentCommandIndex = NO_CURRENT_COMMAND;
    forceCommandLinkAutomatic = false;
    isBusy = false;
    isNoExecutionMode = false;
    isActive = false;

    goToNextCommandLater.setPriority(LazyCaller::PRIORITY_NORMAL);
    
    commandTimer.setSingleShot(true);
    commandTimer.sigTimeout().connect(bind(&TaskViewImpl::cancelWaiting, this, true));

    waitTimer.setSingleShot(true);
    waitTimer.sigTimeout().connect(bind(&TaskViewImpl::onWaitTimeout, this));

    taskCombo.setToolTip(_("Select a task type"));
    taskCombo.addItem("  ----------  ");
    taskCombo.sigCurrentIndexChanged().connect(
        boost::bind(&TaskViewImpl::setCurrentTask, this, _1, true));

    menuButton.setText("*");
    menuButton.setToolTip(_("Option Menu"));
    menuButton.sigClicked().connect(boost::bind(&TaskViewImpl::onMenuButtonClicked, this));

    prevButton.setText("<");
    prevButton.setToolTip(_("Go back to the previous phase"));
    prevButton.sigClicked().connect(boost::bind(&TaskViewImpl::onNextOrPrevButtonClicked, this, -1));

    cancelButton.setText(_("Cancel"));
    cancelButton.setToolTip(_("Cancel waiting for the command to finish"));
    cancelButton.setEnabled(false);
    cancelButton.sigClicked().connect(boost::bind(&TaskViewImpl::cancelWaiting, this, true));

    phaseIndexSpin.setToolTip(_("Phase index"));
    phaseIndexSpin.setSuffix(" / 0");
    phaseIndexSpin.setAlignment(Qt::AlignCenter);
    phaseIndexSpin.setRange(0, 0);
    phaseIndexSpinConnection =
        phaseIndexSpin.sigValueChanged().connect(
            boost::bind(&TaskViewImpl::setPhaseIndex, this, _1, false));

    defaultCommandButton.setText(_("V"));
    defaultCommandButton.setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Preferred);
    defaultCommandButton.setToolTip(_("Execute the default command of the current phase"));
    defaultCommandButton.sigClicked().connect(boost::bind(&TaskViewImpl::onCommandButtonClicked, this, -1));

    autoModeToggle.setText(_("Auto"));
    autoModeToggle.setToolTip(_("Automatic mode"));

    nextButton.setText(">");
    nextButton.setToolTip(_("Skip to the next phase"));
    nextButton.sigClicked().connect(boost::bind(&TaskViewImpl::onNextOrPrevButtonClicked, this, +1));

    topVBox.addLayout(&hbox1);
    topVBox.addLayout(&hbox2);
    topVBox.addLayout(&hbox3);
    topVBox.addLayout(&vspace);
    topVBox.addWidget(&phaseLabel, 0, Qt::AlignHCenter);
    topVBox.addWidget(&commandButtonBox);
    topVBox.addStretch();
    self->setLayout(&topVBox);

    commandButtonBoxLayout = 0;
    isVerticalLayout = true;
    doLayout(false);
    
    setPhaseIndex(0, false);

    activate(false, true);
}


TaskView::~TaskView()
{
    delete impl;
}


TaskViewImpl::~TaskViewImpl()
{

}


static void removeWidgetsInLayout(QLayout* layout)
{
    QLayoutItem* child;
    while((child = layout->takeAt(0)) != 0){
        delete child;
    }
}


void TaskViewImpl::doLayout(bool isVertical)
{
    if(isVertical == this->isVerticalLayout){
        return;
    }
    this->isVerticalLayout = isVertical;

    removeWidgetsInLayout(&hbox1);
    removeWidgetsInLayout(&hbox2);
    removeWidgetsInLayout(&hbox3);
    removeWidgetsInLayout(&vspace);
    
    if(commandButtonBoxLayout){
        delete commandButtonBoxLayout;
    }

    if(isVertical){
        hbox1.addWidget(&taskCombo);

        hbox2.addWidget(&cancelButton);
        hbox2.addWidget(&autoModeToggle);
        hbox2.addWidget(&menuButton);

        hbox3.addWidget(&prevButton);
        hbox3.addWidget(&phaseIndexSpin);
        hbox3.addWidget(&defaultCommandButton);
        hbox3.addWidget(&nextButton);

        vspace.addSpacing(4);

        commandButtonBoxLayout = new QVBoxLayout;
        QMargins m = commandButtonBoxLayout->contentsMargins();
        m.setTop(m.top() + 8);
        m.setBottom(m.bottom() + 8);
        commandButtonBoxLayout->setContentsMargins(m);
        commandButtonBoxLayout->setSpacing(16);
        
    } else {
        hbox1.addWidget(&taskCombo, 1);
        hbox1.addWidget(&menuButton);

        hbox2.addWidget(&prevButton);
        hbox2.addWidget(&cancelButton);
        hbox2.addWidget(&phaseIndexSpin);
        hbox2.addWidget(&defaultCommandButton);
        hbox2.addWidget(&autoModeToggle);
        hbox2.addWidget(&nextButton);
        
        commandButtonBoxLayout = new QHBoxLayout;
        commandButtonBoxLayout->setContentsMargins(0, 0, 0, 0);
    }

    if(isVertical){
        for(size_t i=0; i < commandButtons.size(); ++i){
            PushButton* button = commandButtons[i];
            button->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Preferred);
            commandButtonBoxLayout->addWidget(button, 0, Qt::AlignHCenter);
        }
    } else {
        for(size_t i=0; i < commandButtons.size(); ++i){
            PushButton* button = commandButtons[i];
            button->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
            commandButtonBoxLayout->addWidget(button);
        }
    }

    commandButtonBox.setLayout(commandButtonBoxLayout);
}


void TaskView::activate(bool on)
{
    impl->activate(on, false);
}


void TaskViewImpl::activate(bool on, bool forceUpdate)
{
    if(on != isActive || forceUpdate){

        isActive = on;

        cancelButton.setEnabled(on);
        defaultCommandButton.setEnabled(on);
        autoModeToggle.setEnabled(on);
        phaseLabel.setEnabled(on);
        commandButtonBox.setEnabled(on);

        if(on){
            mv->notify(QString(_("Task sequencer '%1' has been activated.")).arg(self->windowTitle()));
        } else {
            mv->notify(QString(_("Task sequencer '%1' has been deactivated.")).arg(self->windowTitle()));
        }

        if(currentTask && !isNoExecutionMode){
            if(on){
                currentTask->onActivated(self);
            } else {
                currentTask->onDeactivated(self);
            }
        } 
    }
}


bool TaskView::isActive()
{
    return impl->isActive;
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
    tasks.push_back(TaskInfo(task));

    taskCombo.addItem(task->name().c_str());
    
    taskCombo.blockSignals(false);

    if(tasks.size() == 1){
        setCurrentTask(0, true);
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

    if(index < 0 || index >= tasks.size()){
        addTask(task);
    } else {
        if(isWaiting()){
            mv->putln(MessageView::WARNING,
                      format(_("Task \"%1%\" cannot be updated now because it is wating for a command to finish."))
                      % task->name());
        } else {
            TaskInfo& info = tasks[index];
            TaskPtr oldTask = info.task;
            info.task = task;

            if(index == currentTaskIndex){
                info.state = new Mapping();

                if(isExecutionEnabled()){
                    oldTask->storeState(self, *info.state);
                }
                
                setCurrentTask(index, true);

                if(isExecutionEnabled()){
                    task->restoreState(self, *info.state);
                }
            }
            os << format(_("Task \"%1%\" has been updated with the new one.")) % task->name() << endl;
            updated = true;
        }
    }

    return updated;
}


int TaskView::numTasks() const
{
    return impl->tasks.size();
}


Task* TaskView::task(int index)
{
    if(index >=0 && index < impl->tasks.size()){
        return impl->tasks[index].task;
    }
    return 0;
}


int TaskView::currentTaskIndex() const
{
    return impl->currentTaskIndex;
}


bool TaskView::setCurrentTask(int taskIndex)
{
    if(TRACE_FUNCTIONS){
        cout << "TaskView::setCurrentTask(" << taskIndex << ")" << endl;
    }
    return impl->setCurrentTask(taskIndex, false);
}


SignalProxy<void()> TaskView::sigCurrentTaskChanged()
{
    return impl->sigCurrentTaskChanged;
}


int TaskView::currentPhaseIndex() const
{
    return impl->currentPhaseIndex_;
}


void TaskView::setCurrentPhase(int phaseIndex)
{
    if(TRACE_FUNCTIONS){
        cout << "TaskView::setCurrentPhase(" << phaseIndex << ")" << endl;
    }
    impl->setPhaseIndex(phaseIndex, false);
}


SignalProxy<void()> TaskView::sigCurrentPhaseChanged()
{
    return impl->sigCurrentPhaseChanged;
}


int TaskView::currentCommandIndex() const
{
    return (impl->currentCommandIndex < 0) ? -1 : impl->currentCommandIndex;
}


SignalProxy<void()> TaskView::sigCurrentCommandChanged()
{
    return impl->sigCurrentCommandChanged;
}


bool TaskView::isBusy() const
{
    return impl->isBusy;
}


SignalProxy<void()> TaskView::sigBusyStateChanged()
{
    return impl->sigBusyStateChanged;
}


void TaskView::cancelCurrentCommand()
{
    impl->cancelWaiting(true);
}


SignalProxy<void()> TaskView::sigCurrentCommandCanceled()
{
    return impl->sigCurrentCommandCanceled;
}


bool TaskView::isAutoMode() const
{
    return impl->autoModeToggle.isChecked();
}


bool TaskViewImpl::isAutoMode() const
{
    return autoModeToggle.isChecked();
}


void TaskView::setAutoMode(bool on)
{
    if(TRACE_FUNCTIONS){
        cout << "TaskView::setAutoMode(" << on << ")" << endl;
    }
    impl->autoModeToggle.setChecked(on);
}


SignalProxy<void(bool isAutoMode)> TaskView::sigAutoModeToggled()
{
    return impl->autoModeToggle.sigToggled();
}


void TaskView::setNoExecutionMode(bool on)
{
    impl->isNoExecutionMode = on;
}


bool TaskView::isNoExecutionMode() const
{
    return impl->isNoExecutionMode;
}


void TaskView::setCurrentCommand(int commandIndex, bool doExecution)
{
    if(TRACE_FUNCTIONS){
        cout << "TaskView::setCurrentCommand(" << commandIndex << ", " << doExecution << ")" << endl;
    }
    
    if(impl->currentPhase){
        if(commandIndex < impl->currentPhase->numCommands()){
            impl->setCurrentCommandIndex(commandIndex);
            if(doExecution){
                impl->executeCommandSuccessively(commandIndex);
            }
        }
    }
}


bool TaskViewImpl::setCurrentTask(int index, bool forceUpdate)
{
    if(index < 0 || index >= tasks.size()){
        return false;
    }

    bool changed = index != currentTaskIndex;
    if(!changed && !forceUpdate){
        return false;
    }

    if(taskCombo.currentIndex() != index){
        taskCombo.blockSignals(true);
        taskCombo.setCurrentIndex(index);
        taskCombo.blockSignals(false);
    }

    if(changed && currentTaskIndex >= 0){
        TaskInfo& old = tasks[currentTaskIndex];
        old.state = new Mapping();
        if(isExecutionEnabled()){
            old.task->storeState(self, *old.state);
            old.task->onDeactivated(self);
        }
    }

    TaskInfo& info = tasks[index];
    currentTask = info.task;
    currentTaskIndex = index;

    currentPhaseIndex_ = -1;
    setPhaseIndex(0, false);

    if(isExecutionEnabled()){
        info.task->onActivated(self);
    }
    
    if(changed){
        sigCurrentTaskChanged();
    }

    return true;
}


void TaskViewImpl::setCurrentTaskByName(const std::string& name)
{
    if(currentTask && currentTask->name() == name){
        return;
    }
    for(size_t i=0; i < tasks.size(); ++i){
        Task* task = tasks[i].task;
        if(task->name() == name){
            setCurrentTask(i, false);
            break;
        }
    }
}


PushButton* TaskViewImpl::getOrCreateCommandButton(int commandIndex)
{
    PushButton* button;
    if(commandIndex < commandButtons.size()){
        button = commandButtons[commandIndex];
    } else {
        button = new PushButton;
        button->sigClicked().connect(boost::bind(&TaskViewImpl::onCommandButtonClicked, this, commandIndex));
        if(isVerticalLayout){
            button->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Preferred);
            commandButtonBoxLayout->addWidget(button, 0, Qt::AlignHCenter);
        } else {
            button->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
            commandButtonBoxLayout->addWidget(button);
        }
        if(commandButtons.empty()){
            QWidget::setTabOrder(&menuButton, button);
        } else {
            QWidget::setTabOrder(commandButtons.back(), button);
        }
        commandButtons.push_back(button);
    }
    return button;
}


bool TaskViewImpl::setCurrentCommandIndex(int index)
{
    for(size_t i=0; i < commandButtons.size(); ++i){
        commandButtons[i]->setDown(false);
    }
    if(isBusy && index >= 0 && index < commandButtons.size()){
        commandButtons[index]->setDown(true);
    }
    bool changed = (index != currentCommandIndex);
    currentCommandIndex = index;
    return changed;
}
    

void TaskView::setBusyState(bool on)
{
    impl->setBusyState(on);
}


void TaskViewImpl::setBusyState(bool on)
{
    if(TRACE_FUNCTIONS){
        cout << "TaskViewImpl::setBusyState(" << on << ")" << endl;
    }
    
    if(on != isBusy){
        if(TRACE_FUNCTIONS){
            cout << "on != isBusy" << endl;
        }
        isBusy = on;

        for(size_t i=0; i < commandButtons.size(); ++i){
            commandButtons[i]->setDown(false);
            commandButtons[i]->setEnabled(!isBusy);
        }
        if(isBusy && currentCommandIndex >= 0 && currentCommandIndex < commandButtons.size()){
            commandButtons[currentCommandIndex]->setDown(true);
        }
        cancelButton.setEnabled(isBusy);
        MessageView::instance()->flush();
        
        sigBusyStateChanged();
    }
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


void TaskViewImpl::setPhaseIndex(int index, bool isSuccessivelyCalled)
{
    if(TRACE_FUNCTIONS){
        cout << "TaskViewImpl::setPhaseIndex(" << index << ", " << isSuccessivelyCalled << ")" << endl;
    }
    
    cancelWaiting(false);
    
    int numPhases = 0;
    if(currentTask){
        numPhases = currentTask->numPhases();
    }

    int prevPhaseIndex = currentPhaseIndex_;

    currentPhaseIndex_ = std::max(0, std::min(index, numPhases - 1));

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
            button->setEnabled(!isBusy);
            button->setDown(false);
            button->show();
        }
    }

    for(size_t i=numVisibleButtons; i < commandButtons.size(); ++i){
        commandButtons[i]->hide();
    }

    phaseIndexSpinConnection.block();
    phaseIndexSpin.setRange(0, numPhases);
    phaseIndexSpin.setValue(currentPhaseIndex_);
    phaseIndexSpinConnection.unblock();
    phaseIndexSpin.setSuffix(QString(" / %1").arg(numPhases - 1));

    if(currentPhaseIndex_ != prevPhaseIndex){
        sigCurrentPhaseChanged();
    }

    if(isExecutionDisabled()){
        return;
    }
        
    bool nextCommandProcessed = false;
    if(isSuccessivelyCalled && currentPhase){
        if(nextCommandIndex){
            if(*nextCommandIndex < 0 || autoModeToggle.isChecked()){
                setCurrentCommandIndex(NO_CURRENT_COMMAND);
                executeCommandSuccessively(*nextCommandIndex);
                nextCommandProcessed = true;
            }
        }
    }
    if(!nextCommandProcessed){
        setBusyState(false);
        bool doEmit = currentCommandIndex >= 0;
        setCurrentCommandIndex(NO_CURRENT_COMMAND);
        if(doEmit){
            sigCurrentCommandChanged();
        }
    }
}


void TaskViewImpl::executeCommandSuccessively(int commandIndex)
{
    if(TRACE_FUNCTIONS){
        cout << "TaskViewImpl::executeCommandSuccessively(" << commandIndex << ")" << endl;
    }
    
    cancelWaiting(false);

    nextCommandIndex = boost::none;

    setBusyState(true);
    
    if(!currentTask || !currentPhase){
        setBusyState(false);
        
    } else {
        nextPhaseIndex = currentPhaseIndex_;
        TaskFunc commandFunc;
        if(commandIndex < 0){
            int defaultCommandIndex = -1;
            for(int i=0; i < currentPhase->numCommands(); ++i){
                TaskCommand* command = currentPhase->command(i);
                if(command->isDefault()){
                    defaultCommandIndex = i;
                    break;
                }
            }
            commandFunc = currentPhase->preCommand();

            if(defaultCommandIndex >= 0){
                if(commandFunc){
                    nextCommandIndex = defaultCommandIndex;
                    setCommandLinkAutomatic();
                } else {
                    commandIndex = defaultCommandIndex;
                }
            }
        }
        if(commandIndex >= 0){
            TaskCommand* command = currentPhase->command(commandIndex);
            if(command){
                commandFunc = command->function();
                nextPhaseIndex = command->nextPhaseIndex(currentPhaseIndex_);
                if(nextPhaseIndex >= currentTask->numPhases()){
                    nextPhaseIndex = boost::none;
                }
                nextCommandIndex = command->nextCommandIndex(commandIndex);
            }
        }

        setFocusToCommandButton(commandIndex < 0 ? 0 : commandIndex);
        bool doEmit = commandIndex != currentCommandIndex;
        
        if(setCurrentCommandIndex(commandIndex)){
            sigCurrentCommandChanged();
        }

        if(commandFunc){
            commandFunc(this);
        }

        setTransitionToNextCommand();
    }
}


void TaskViewImpl::setTransitionToNextCommand()
{
    if(TRACE_FUNCTIONS){
        cout << "TaskViewImpl::setTransitionToNextCommand()" << endl;
    }

    bool isNextDispatched = false;

    goToNextCommandLater.cancel();
    
    if(!eventLoop.isRunning()){
        if(nextPhaseIndex && *nextPhaseIndex != currentPhaseIndex_){
            nextCommandIndex = -1;
            goToNextCommandLater.setFunction(boost::bind(&TaskViewImpl::setPhaseIndex, this, *nextPhaseIndex, true));
            goToNextCommandLater();
            isNextDispatched = true;

        } else {
            if(nextCommandIndex && *nextCommandIndex != currentCommandIndex){
                int index = *nextCommandIndex;
                nextCommandIndex = boost::none;
                bool executeNext = autoModeToggle.isChecked();
                if(!executeNext){
                    if(currentCommandIndex >= 0){
                        TaskCommand* command = currentPhase->command(currentCommandIndex);
                        if(command && command->isCommandLinkAutomatic()){
                            executeNext = true;
                        }
                    } else if(forceCommandLinkAutomatic){
                        executeNext = true;
                    }
                }
                setFocusToCommandButton(index);
                if(setCurrentCommandIndex(index)){
                    sigCurrentCommandChanged();
                }
                if(executeNext){
                    goToNextCommandLater.setFunction(boost::bind(&TaskViewImpl::executeCommandSuccessively, this, index));
                    goToNextCommandLater();
                    isNextDispatched = true;
                }
            }
        }
    }
    forceCommandLinkAutomatic = false;

    if(!isNextDispatched){
        setBusyState(false);
    }
}


void TaskViewImpl::retry()
{
    breakSequence();
    nextCommandIndex = -1;
    setPhaseIndex(0, false);
}


void TaskViewImpl::onCommandButtonClicked(int commandIndex)
{
    if(TRACE_FUNCTIONS){
        cout << "TaskViewImpl::onCommandButtonClicked(" << commandIndex << ")" << endl;
    }

    if(isExecutionEnabled()){
        executeCommandSuccessively(commandIndex);
    } else {    
        setBusyState(true);
        setCurrentCommandIndex(commandIndex);
        sigCurrentCommandChanged();
    }
}


void TaskViewImpl::onNextOrPrevButtonClicked(int direction)
{
    if(TRACE_FUNCTIONS){
        cout << "TaskViewImpl::onNextOrPrevButtonClicked(" << direction << ")" << endl;
    }
    
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
                int callerCommandIndex = currentCommandIndex;
                setCurrentCommandIndex(commandIndex);
                isPendingCommandCompleted = false;
                func(this);
                QCoreApplication::processEvents(QEventLoop::AllEvents, 10);
                setCurrentCommandIndex(callerCommandIndex);
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
        waitTimer.start(sec * 1000.0);
        isPendingCommandCompleted = false;
        eventLoop.exec(QEventLoop::AllEvents);
        completed = isPendingCommandCompleted;
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
        isPendingCommandCompleted = false;
        if(timeout > 0.0){
            commandTimer.start(timeout * 1000.0);
        } else {
            commandTimer.stop();
        }
        eventLoop.exec(QEventLoop::AllEvents);
        completed = isPendingCommandCompleted;
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


void TaskViewImpl::cancelWaiting(bool doBreak)
{
    if(isExecutionDisabled()){
        if(doBreak){
            setBusyState(false);
            sigCurrentCommandCanceled();
        }
    } else {
        if(eventLoop.isRunning()){
            autoModeToggle.setChecked(false); // stop the auto mode, too
            nextPhaseIndex = boost::none;
            nextCommandIndex = boost::none;
            stopWaiting(false);
        }
        if(doBreak){
            goToNextCommandLater.cancel();
            setBusyState(false);
            sigCurrentCommandCanceled();
        }
    }
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
    Action* verticalCheck = menuManager.addCheckItem(_("Vertical Layout"));
    verticalCheck->setChecked(isVerticalLayout);
    verticalCheck->sigToggled().connect(boost::bind(&TaskViewImpl::doLayout, this, _1));
    
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
    archive.write("layoutMode", impl->isVerticalLayout ? "vertical" : "horizontal");
    archive.write("isAutoMode", impl->autoModeToggle.isChecked());
    if(impl->currentTask){
        archive.write("currentTask", impl->currentTask->name());
    }
    return true;
}


bool TaskView::restoreState(const Archive& archive)
{
    string layoutMode;
    if(archive.read("layoutMode", layoutMode)){
        if(layoutMode == "horizontal"){
            impl->doLayout(false);
        } else if(layoutMode == "vertical"){
            impl->doLayout(true);
        }
    }
    impl->autoModeToggle.setChecked(archive.get("isAutoMode", false));
    string name;
    if(archive.read("currentTask", name)){
        archive.addPostProcess(boost::bind(&TaskViewImpl::setCurrentTaskByName, impl, name));
    }
    return true;
}
