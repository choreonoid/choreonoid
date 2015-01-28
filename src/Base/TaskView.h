/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_TASK_VIEW_H
#define CNOID_BASE_TASK_VIEW_H

#include <cnoid/Task>
#include <cnoid/View>
#include "exportdecl.h"

namespace cnoid {

class TaskViewImpl;

class CNOID_EXPORT TaskView : public View
{
public:
    static void initializeClass(ExtensionManager* ext);
    static TaskView* instance();
    
    TaskView();
    ~TaskView();

    void addTask(Task* task);
    bool updateTask(Task* task);

protected:
    virtual bool storeState(Archive& archive);
    virtual bool restoreState(const Archive& archive);
    
private:
    TaskViewImpl* impl;
};

}

#endif
