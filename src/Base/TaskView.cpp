/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include "TaskView.h"
#include <cnoid/ViewManager>
#include <cnoid/MenuManager>
#include <cnoid/MessageView>
#include <cnoid/Buttons>
#include <cnoid/ComboBox>
#include <cnoid/SpinBox>
#include <cnoid/Timer>
#include <cnoid/LazyCaller>
#include <cnoid/AppUtil>
#include <cnoid/ConnectionSet>
#include <cnoid/stdx/optional>
#include <QBoxLayout>
#include <QLabel>
#include <QEventLoop>
#include <QApplication>
#include <set>
#include <iostream>
#include <fmt/format.h>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

namespace {

const bool TRACE_FUNCTIONS = false;

TaskView* instance_ = nullptr;

typedef PushButton CommandButton;
//typedef ToolButton CommandButton;

/**
   This CombBox does not accept the wheel event to change the selection
   so that the current task is not changed by incorrect operations
*/
class TaskComboBox : public ComboBox
{
public:
    virtual void wheelEvent(QWheelEvent*){

    }
};

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
    TaskComboBox taskCombo;
    PushButton menuButton;
    ToolButton usualPhaseButton;
    ToolButton configPhaseButton;
    PushButton prevButton;
    PushButton cancelButton;
    SpinBox phaseIndexSpin;
    Connection phaseIndexSpinConnection;
    ToolButton defaultCommandButton;
    ToggleButton autoModeToggle;
    PushButton nextButton;
    QLabel phaseLabel;
    QWidget commandButtonBox;
    QBoxLayout* commandButtonBoxBaseLayout;
    vector<QBoxLayout*> commandButtonBoxLayouts;
    vector<CommandButton*> commandButtons;

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

    Signal<void(Task* task)> sigTaskAdded;
    Signal<void(Task* task)> sigTaskRemoved;
    
    TaskPtr currentTask;
    int currentTaskIndex;
    Signal<void()> sigCurrentTaskChanged;
    TaskPhasePtr currentPhase;
    int currentPhaseIndex_;
    int lastUsualPhaseIndex;
    int configPhaseIndex;
    Signal<void()> sigCurrentPhaseChanged;
    stdx::optional<int> nextPhaseIndex;
    enum { NO_CURRENT_COMMAND = -2, PRE_COMMAND = -1 };
    int currentCommandIndex;
    stdx::optional<int> nextCommandIndex; // -1 means the pre-command

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

    vector<string> serializedTasks;
    int currentIndexInSerializedTasks;

    ScopedConnectionSet menuConnections;
    MenuManager menuManager;
    struct MenuItem {
        std::function<void()> func;
        std::function<void(bool on)> checkFunc;
        Action* action;
        MenuItem() { action = 0; }
    };
    vector<MenuItem> menuItems;
    Signal<void()> sigMenuRequest;
    Signal<void(int index)> sigMenuItemTriggered;
    Signal<void(int index, bool on)> sigMenuItemToggled;

    TaskViewImpl(TaskView* self);
    ~TaskViewImpl();
    void doLayout(bool on);
    void activate(bool on, bool forceUpdate);
    void addTask(Task* task);
    bool updateTask(Task* task);
    void clearTasks();
    bool setCurrentTask(int index, bool forceUpdate, bool isTransition = false);
    bool setCurrentTaskByName(const std::string& name, bool forceUpdate, bool isTransition = false);
    CommandButton* getOrCreateCommandButton(int index);
    void layoutCommandButtons();
    bool setCurrentCommandIndex(int index);
    void setBusyState(bool on);
    void setFocusToCommandButton(int commandIndex);
    int getClosestNextPhaseIndex(int phaseIndex);
    int getLoopBackPhaseIndex(int phaseIndex);
    void setPhaseIndex(int index, bool isSuccessivelyCalled);
    void executeCommandSuccessively(int commandIndex);
    void setTransitionToNextCommand();
    void goToNextTask();
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

    void onUsualPhaseButtonClicked();
    void onConfigPhaseButtonClicked();
    
    void onMenuButtonClicked();
    void updateMenuItems(bool doPopup);
    virtual void addMenuItem(const std::string& caption, std::function<void()> func);
    virtual void addCheckMenuItem(const std::string& caption, bool isChecked, std::function<void(bool on)> func);
    virtual void addMenuSeparator();
    void onMenuItemTriggered(int index);
    void onMenuItemToggled(int index, bool on);
    void applyMenuItem(int index, bool on);
};

}


static void onAboutToQuit()
{
    if(instance_){
        instance_->clearTasks();
    }
}


void TaskView::initializeClass(ExtensionManager* ext)
{
    ext->viewManager().registerClass<TaskView>(
        "TaskView", N_("Task"), ViewManager::SINGLE_OPTIONAL);

    cnoid::sigAboutToQuit().connect([](){ onAboutToQuit(); });
}


TaskView* TaskView::instance()
{
    if(!instance_){
        instance_ = ViewManager::getOrCreateView<TaskView>();
    }
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
    self->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
    self->setDefaultLayoutArea(View::CENTER);

    currentTaskIndex = -1;
    currentPhaseIndex_ = 0;
    lastUsualPhaseIndex = 0;
    configPhaseIndex = 0;
    currentCommandIndex = NO_CURRENT_COMMAND;
    forceCommandLinkAutomatic = false;
    isBusy = false;
    isNoExecutionMode = false;
    isActive = false;

    goToNextCommandLater.setPriority(LazyCaller::PRIORITY_NORMAL);
    
    commandTimer.setSingleShot(true);
    commandTimer.sigTimeout().connect([&](){ cancelWaiting(true); });

    waitTimer.setSingleShot(true);
    waitTimer.sigTimeout().connect([&](){ onWaitTimeout(); });

    currentIndexInSerializedTasks = 0;

    taskCombo.setToolTip(_("Select a task type"));
    taskCombo.addItem("  ----------  ");
    taskCombo.sigCurrentIndexChanged().connect(
        [&](int index){
            setCurrentTask(index, true);
            currentIndexInSerializedTasks = 0;
        });

    menuButton.setText("*");
    menuButton.setToolTip(_("Option Menu"));
    menuButton.sigClicked().connect([&](){ onMenuButtonClicked(); });

    usualPhaseButton.setText("   {   ");
    usualPhaseButton.setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Preferred);
    usualPhaseButton.setToolTip(_("Return to the current phase"));
    usualPhaseButton.sigClicked().connect([&](){ onUsualPhaseButtonClicked(); });

    configPhaseButton.setText("   }   ");
    configPhaseButton.setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Preferred);
    configPhaseButton.setToolTip(_("Go to the config phase"));
    configPhaseButton.sigClicked().connect([&](){ onConfigPhaseButtonClicked(); });
    
    prevButton.setText("<");
    prevButton.setToolTip(_("Go back to the previous phase"));
    prevButton.sigClicked().connect([&](){ onNextOrPrevButtonClicked(-1); });

    cancelButton.setText(_("Cancel"));
    cancelButton.setToolTip(_("Cancel waiting for the command to finish"));
    cancelButton.setEnabled(false);
    cancelButton.sigClicked().connect([&](){ cancelWaiting(true); });

    phaseIndexSpin.setToolTip(_("Phase index"));
    phaseIndexSpin.setSuffix(" / 0");
    phaseIndexSpin.setAlignment(Qt::AlignCenter);
    phaseIndexSpin.setRange(0, 0);
    phaseIndexSpinConnection =
        phaseIndexSpin.sigValueChanged().connect(
            [&](int index){ setPhaseIndex(index, false); });

    defaultCommandButton.setText(_("V"));
    defaultCommandButton.setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Preferred);
    defaultCommandButton.setToolTip(_("Execute the default command of the current phase"));
    defaultCommandButton.sigClicked().connect([&](){ onCommandButtonClicked(-1); });

    autoModeToggle.setText(_("Auto"));
    autoModeToggle.setToolTip(_("Automatic mode"));

    nextButton.setText(">");
    nextButton.setToolTip(_("Skip to the next phase"));
    nextButton.sigClicked().connect([&](){ onNextOrPrevButtonClicked(+1); });

    topVBox.addLayout(&hbox1);
    topVBox.addLayout(&hbox2);
    topVBox.addLayout(&hbox3);
    topVBox.addLayout(&vspace);
    phaseLabel.setTextInteractionFlags(Qt::TextSelectableByMouse);
    topVBox.addWidget(&phaseLabel, 0, Qt::AlignHCenter);
    topVBox.addWidget(&commandButtonBox);
    topVBox.addStretch();
    self->setLayout(&topVBox);

    commandButtonBoxBaseLayout = 0;
    commandButtonBoxLayouts.resize(5, 0);
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
    for(size_t i=0; i < tasks.size(); ++i){
        sigTaskRemoved(tasks[i].task);
    }
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
    
    if(commandButtonBoxBaseLayout){
        delete commandButtonBoxBaseLayout;
    }

    if(isVertical){
        hbox1.addWidget(&taskCombo);

        hbox2.addWidget(&cancelButton);
        hbox2.addWidget(&autoModeToggle);
        hbox2.addWidget(&menuButton);
        hbox2.addWidget(&usualPhaseButton);
        hbox2.addWidget(&configPhaseButton);

        hbox3.addWidget(&prevButton);
        hbox3.addWidget(&phaseIndexSpin);
        hbox3.addWidget(&defaultCommandButton);
        hbox3.addWidget(&nextButton);

        vspace.addSpacing(4);

        commandButtonBoxBaseLayout = new QHBoxLayout();
        commandButtonBoxBaseLayout->setContentsMargins(0, 0, 0, 0);

        for(size_t i=0; i < commandButtonBoxLayouts.size(); ++i){
            commandButtonBoxLayouts[i] = new QVBoxLayout();
            QMargins m = commandButtonBoxLayouts[i]->contentsMargins();
            m.setTop(m.top() + 8);
            m.setBottom(m.bottom() + 8);
            commandButtonBoxLayouts[i]->setContentsMargins(m);
            commandButtonBoxLayouts[i]->setSpacing(16);
            commandButtonBoxBaseLayout->addLayout(commandButtonBoxLayouts[i]);
        }
        
    } else {
        hbox1.addWidget(&taskCombo, 1);
        hbox1.addWidget(&menuButton);
        hbox1.addWidget(&usualPhaseButton);
        hbox1.addWidget(&configPhaseButton);

        hbox2.addWidget(&prevButton);
        hbox2.addWidget(&cancelButton);
        hbox2.addWidget(&phaseIndexSpin);
        hbox2.addWidget(&defaultCommandButton);
        hbox2.addWidget(&autoModeToggle);
        hbox2.addWidget(&nextButton);
        
        commandButtonBoxBaseLayout = new QVBoxLayout();
        commandButtonBoxBaseLayout->setContentsMargins(0, 0, 0, 0);

        for(size_t i=0; i < commandButtonBoxLayouts.size(); ++i){
            commandButtonBoxLayouts[i] = new QHBoxLayout();
            commandButtonBoxLayouts[i]->setContentsMargins(0, 0, 0, 0);
            commandButtonBoxBaseLayout->addLayout(commandButtonBoxLayouts[i]);
        }
    }

    commandButtonBox.setLayout(commandButtonBoxBaseLayout);

    layoutCommandButtons();
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

    sigTaskAdded(task);

    if(tasks.size() == 1){
        setCurrentTask(0, true);
    }

    os << format(_("Task \"{}\" has been added."), task->name()) << endl;
}


bool TaskView::updateTask(Task* task)
{
    return impl->updateTask(task);
}


bool TaskViewImpl::updateTask(Task* task)
{
    bool updated = false;
    
    int index = taskCombo.findText(task->name().c_str());

    if(index < 0 || index >= static_cast<int>(tasks.size())){
        addTask(task);
    } else {
        if(isWaiting()){
            mv->putln(MessageView::WARNING,
                      format(_("Task \"{}\" cannot be updated now because it is wating for a command to finish."),
                      task->name()));
        } else {
            TaskInfo& info = tasks[index];
            TaskPtr oldTask = info.task;
            info.task = task;

            bool doEmitSignals = task != oldTask;

            if(index != currentTaskIndex){
                if(doEmitSignals){
                    sigTaskRemoved(oldTask);
                    sigTaskAdded(task);
                }
            } else {
                info.state = new Mapping();

                if(isExecutionEnabled()){
                    oldTask->storeState(self, *info.state);
                }
                if(doEmitSignals){
                    sigTaskRemoved(oldTask);
                }
                
                setCurrentTask(index, true);

                if(doEmitSignals){
                    sigTaskAdded(task);
                }

                if(isExecutionEnabled()){
                    task->restoreState(self, *info.state);
                }
            }
            os << format(_("Task \"{}\" has been updated with the new one."), task->name()) << endl;
            updated = true;
        }
    }

    return updated;
}


/**
   \note This function is not implemented yet
*/
bool TaskView::removeTask(Task* task)
{
    return false;
}


SignalProxy<void(Task* task)> TaskView::sigTaskAdded()
{
    return impl->sigTaskAdded;
}


void TaskView::clearTasks()
{
    impl->clearTasks();
}


void TaskViewImpl::clearTasks()
{
    while(!tasks.empty()){
        sigTaskRemoved(tasks.back().task);
        tasks.pop_back();
    }
}
            

SignalProxy<void(Task* task)> TaskView::sigTaskRemoved()
{
    return impl->sigTaskRemoved;
}


int TaskView::numTasks() const
{
    return impl->tasks.size();
}


Task* TaskView::task(int index)
{
    if(index >=0 && index < static_cast<int>(impl->tasks.size())){
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


void TaskView::serializeTasks(const std::vector<std::string>& tasks)
{
    impl->serializedTasks = tasks;
    impl->currentIndexInSerializedTasks = 0;
}


void TaskView::setNoExecutionMode(bool on)
{
    impl->isNoExecutionMode = on;
}


bool TaskView::isNoExecutionMode() const
{
    return impl->isNoExecutionMode;
}


void TaskView::executeCommand(int commandIndex)
{
    setCurrentCommand(commandIndex, true);
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


bool TaskViewImpl::setCurrentTask(int index, bool forceUpdate, bool isTransition)
{
    if(TRACE_FUNCTIONS){
        cout << "TaskViewImpl::setCurrentTask(" << index << ", " << forceUpdate << ")" << endl;
    }

    if(index < 0 || index >= static_cast<int>(tasks.size())){
        return false;
    }

    bool isTaskChanged = index != currentTaskIndex;
    if(!isTaskChanged && !forceUpdate){
        return false;
    }

    if(taskCombo.currentIndex() != index){
        taskCombo.blockSignals(true);
        taskCombo.setCurrentIndex(index);
        taskCombo.blockSignals(false);
    }

    if(isTaskChanged && currentTaskIndex >= 0){
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

    int prevPhaseIndex = currentPhaseIndex_;
    currentPhaseIndex_ = 0;
    currentPhase.reset();
    currentCommandIndex = NO_CURRENT_COMMAND;
    setPhaseIndex(0, false);

    lastUsualPhaseIndex = currentPhaseIndex_;
    configPhaseIndex = std::max(0, currentTask->numPhases() - 1);

    if(isExecutionEnabled()){
        info.task->onActivated(self);
    }
    
    if(isTaskChanged){
        sigCurrentTaskChanged();
    }
    if(currentPhaseIndex_ != prevPhaseIndex){
        sigCurrentPhaseChanged();
    }

    if(isTransition && isAutoMode()){
        // Proceed the start button
        executeCommandSuccessively(0);
    }

    return true;
}


bool TaskViewImpl::setCurrentTaskByName(const std::string& name, bool forceUpdate, bool isTransition)
{
    if(currentTask && currentTask->name() == name){
        return true;
    }
    for(size_t i=0; i < tasks.size(); ++i){
        Task* task = tasks[i].task;
        if(task->name() == name){
            setCurrentTask(i, forceUpdate, isTransition);
            return true;
        }
    }
    return false;
}


CommandButton* TaskViewImpl::getOrCreateCommandButton(int commandIndex)
{
    CommandButton* button;
    if(commandIndex < static_cast<int>(commandButtons.size())){
        button = commandButtons[commandIndex];
    } else {
        button = new CommandButton(&commandButtonBox);
        button->sigClicked().connect([this, commandIndex](){ onCommandButtonClicked(commandIndex); });
        commandButtons.push_back(button);
        /**
           \note the tab focus order should not be set to command buttons by the setTabOrder function
           because it causes unexpected focus changes after pushing a command button
        */
    }
    return button;
}


void TaskViewImpl::layoutCommandButtons()
{
    int numVisibleButtons = 0;

    for(size_t i=0; i < commandButtonBoxLayouts.size(); ++i){
        QBoxLayout* layout = commandButtonBoxLayouts[i];
        QLayoutItem* child;
        while((child = layout->takeAt(0)) != 0) {
            delete child;
        }
    }

    if(!currentPhase){
        CommandButton* button = getOrCreateCommandButton(0);
        button->setText("-");
        button->setEnabled(false);
        button->setToolTip(QString());
        commandButtonBoxLayouts[0]->addWidget(button);
        button->show();
        numVisibleButtons = 1;

    } else {
        numVisibleButtons = currentPhase->numCommands();

        for(int i=0; i < numVisibleButtons; ++i){
            CommandButton* button = getOrCreateCommandButton(i);
            TaskCommand* command = currentPhase->command(i);
            button->setText(command->caption().c_str());
            button->setToolTip(command->description().c_str());
            button->setEnabled(!isBusy);
            button->setDown(false);

            if(isVerticalLayout){
                button->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Preferred);
                commandButtonBoxLayouts[0]->addWidget(button, 0, Qt::AlignHCenter);
            } else {
                int level = std::min(command->level(), (int)commandButtonBoxLayouts.size() - 1);
                QBoxLayout* layout = commandButtonBoxLayouts[level];
                button->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
                layout->addWidget(button);
            }
            button->show();
        }
    }

    for(size_t i=1; i < commandButtonBoxLayouts.size(); ++i){
        commandButtonBoxLayouts[i]->insertStretch(0);
        commandButtonBoxLayouts[i]->addStretch();
    }

    for(size_t i = numVisibleButtons; i < commandButtons.size(); ++i){
        commandButtons[i]->hide();
    }
}


bool TaskViewImpl::setCurrentCommandIndex(int index)
{
    for(size_t i=0; i < commandButtons.size(); ++i){
        commandButtons[i]->setDown(false);
    }
    if(isBusy && index >= 0 && index < static_cast<int>(commandButtons.size())){
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
        if(isBusy && currentCommandIndex >= 0 && currentCommandIndex < static_cast<int>(commandButtons.size())){
            commandButtons[currentCommandIndex]->setDown(true);
        }
        cancelButton.setEnabled(isBusy);
        MessageView::instance()->flush();
        
        sigBusyStateChanged();
    }
}


void TaskViewImpl::setFocusToCommandButton(int commandIndex)
{
    if(commandIndex >= 0 && commandIndex < static_cast<int>(commandButtons.size())){
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

    if(numPhases == 0){
        currentPhase = 0;
        phaseLabel.setText("No phase");
        phaseLabel.show();
    } else {
        currentPhase = currentTask->phase(currentPhaseIndex_);

        if(currentPhase->caption().empty()){
            phaseLabel.setText("");
            phaseLabel.hide();
        } else {
            phaseLabel.setText(currentPhase->caption().c_str());
            phaseLabel.show();
        }
    }
    
    phaseIndexSpinConnection.block();
    phaseIndexSpin.setRange(0, numPhases);
    phaseIndexSpin.setValue(currentPhaseIndex_);
    phaseIndexSpinConnection.unblock();
    phaseIndexSpin.setSuffix(QString(" / %1").arg(numPhases - 1));

    layoutCommandButtons();

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

    nextCommandIndex = stdx::nullopt;

    setBusyState(true);
    
    if(!currentTask || !currentPhase){
        setBusyState(false);
        
    } else {
        bool isCommandToFinishTask = false;
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
                    nextPhaseIndex = stdx::nullopt;
                    isCommandToFinishTask = true;
                }
                nextCommandIndex = command->nextCommandIndex(commandIndex);
            }
        }

        setFocusToCommandButton(commandIndex < 0 ? 0 : commandIndex);
        
        if(setCurrentCommandIndex(commandIndex)){
            sigCurrentCommandChanged();
        }

        if(commandFunc){
            commandFunc(this);
        }

        if(!isCommandToFinishTask){
            setTransitionToNextCommand();
        } else {
            callLater([&](){ goToNextTask(); });
        }
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
            int npi = *nextPhaseIndex;
            goToNextCommandLater.setFunction([this, npi](){ setPhaseIndex(npi, true); });
            goToNextCommandLater();
            isNextDispatched = true;

        } else {
            if(nextCommandIndex && *nextCommandIndex != currentCommandIndex){
                int index = *nextCommandIndex;
                nextCommandIndex = stdx::nullopt;
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
                    goToNextCommandLater.setFunction([this, index](){ executeCommandSuccessively(index); });
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


void TaskViewImpl::goToNextTask()
{
    if(TRACE_FUNCTIONS){
        cout << "TaskViewImpl::goToNextTask)(" << endl;
    }
    
    if(!currentTask){
        return;
    }
    
    setBusyState(false);

    int n = serializedTasks.size();
    for(int i=currentIndexInSerializedTasks; i < n; ++i){
        if(serializedTasks[i] == currentTask->name()){
            int nextIndex = i + 1;
            if(nextIndex < n){
                const auto& nextTask = serializedTasks[nextIndex];
                if(setCurrentTaskByName(nextTask, true, true)){
                    ++currentIndexInSerializedTasks;
                } else {
                    mv->putln(MessageView::WARNING,
                              format(_("Next task \"{}\" is not found."),
                              nextTask));
                }
            }
            break;
        }
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
    nextPhaseIndex = stdx::nullopt;
    nextCommandIndex = stdx::nullopt;

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
            nextPhaseIndex = stdx::nullopt;
            nextCommandIndex = stdx::nullopt;
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


void TaskViewImpl::onUsualPhaseButtonClicked()
{
    setPhaseIndex(lastUsualPhaseIndex, false);
}


void TaskViewImpl::onConfigPhaseButtonClicked()
{
    if(currentPhaseIndex_ < configPhaseIndex){
        lastUsualPhaseIndex = currentPhaseIndex_;
    }
    setPhaseIndex(configPhaseIndex, false);
}


void TaskViewImpl::onMenuButtonClicked()
{
    if(isNoExecutionMode){
        sigMenuRequest();
    } else {
        updateMenuItems(true);
    }
}


void TaskViewImpl::updateMenuItems(bool doPopup)
{
    menuItems.clear();
    menuConnections.disconnect();
    
    menuManager.setNewPopupMenu(self);
    if(currentTask){
        currentTask->onMenuRequest(*this);
    }
    if(menuManager.numItems() > 0){
        menuManager.addSeparator();
    }

    addMenuItem(_("Retry"), [&](){ retry(); });
    
    Action* verticalCheck = menuManager.addCheckItem(_("Vertical Layout"));
    verticalCheck->setChecked(isVerticalLayout);
    verticalCheck->sigToggled().connect([&](bool on){ doLayout(on); });

    if(doPopup){
        menuManager.popupMenu()->popup(menuButton.mapToGlobal(QPoint(0,0)));
    }
}


void TaskViewImpl::addMenuItem(const std::string& caption, std::function<void()> func)
{
    int index = menuItems.size();
    MenuItem menuItem;
    menuItem.action = menuManager.addItem(caption.c_str());
    menuConnections.add(
        menuItem.action->sigTriggered().connect(
            [this, index](){ onMenuItemTriggered(index); }));
    if(func){
        menuItem.func = func;
    }
    menuItems.push_back(menuItem);
}


void TaskViewImpl::addCheckMenuItem(const std::string& caption, bool isChecked, std::function<void(bool on)> func)
{
    int index = menuItems.size();
    MenuItem menuItem;
    menuItem.action = menuManager.addCheckItem(caption.c_str());
    menuItem.action->setChecked(isChecked);
    menuConnections.add(
        menuItem.action->sigToggled().connect(
            [this, index](bool on){ onMenuItemToggled(index, on); }));
    if(func){
        menuItem.checkFunc = func;
    }
    menuItems.push_back(menuItem);
}


void TaskViewImpl::addMenuSeparator()
{
    menuManager.addSeparator();
}


void TaskViewImpl::onMenuItemTriggered(int index)
{
    MenuItem& item = menuItems[index];
    if(isNoExecutionMode){
        sigMenuItemTriggered(index);
    } else {
        if(item.func){
            item.func();
        }
    }
}


void TaskViewImpl::onMenuItemToggled(int index, bool on)
{
    MenuItem& item = menuItems[index];
    if(isNoExecutionMode){
        sigMenuItemToggled(index, on);
    } else {
        if(item.checkFunc){
            item.checkFunc(on);
        }
    }
}


void TaskView::executeMenuItem(int index)
{
    impl->applyMenuItem(index, true);
}


void TaskView::checkMenuItem(int index, bool on)
{
    impl->applyMenuItem(index, on);
}


void TaskViewImpl::applyMenuItem(int index, bool on)
{
    if(isExecutionEnabled()){
        updateMenuItems(false);
        if(index >= 0 && index < static_cast<int>(menuItems.size())){
            MenuItem& item = menuItems[index];
            if(item.func){
                item.func();
            } else if(item.checkFunc){
                item.checkFunc(on);
            }
        }
    }
}
                

std::vector<bool> TaskView::menuItemCheckStates() const
{
    std::vector<bool> states;
    impl->updateMenuItems(false);
    int n = impl->menuItems.size();
    states.resize(n, false);
    for(int i=0; i < n; ++i){
        Action* action = impl->menuItems[i].action;
        if(action){
            states[i] = action->isChecked();
        }
    }
    return states;
}


SignalProxy<void()> TaskView::sigMenuRequest()
{
    return impl->sigMenuRequest;
}


void TaskView::showMenu(std::vector<bool> checkStates)
{
    impl->updateMenuItems(true);

    impl->menuConnections.block();
    int n = impl->menuItems.size();
    for(int i=0; i < n; ++i){
        Action* action = impl->menuItems[i].action;
        if(action && action->isCheckable()){
            action->setChecked(checkStates[i]);
        }
    }
    impl->menuConnections.unblock();
}


SignalProxy<void(int index)> TaskView::sigMenuItemTriggered()
{
    return impl->sigMenuItemTriggered;
}


SignalProxy<void(int index, bool on)> TaskView::sigMenuItemToggled()
{
    return impl->sigMenuItemToggled;
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
        archive.addPostProcess([this, name](){ impl->setCurrentTaskByName(name, false); }, 1);
    }
    return true;
}
