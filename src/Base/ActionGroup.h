/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_ACTION_GROUP_H
#define CNOID_BASE_ACTION_GROUP_H

#include <cnoid/Signal>
#include <QActionGroup>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT ActionGroup : public QActionGroup
{
    Q_OBJECT

public:
    ActionGroup(QObject* parent);
    ~ActionGroup();
                               
    SignalProxy<void(QAction* action)> sigHovered();
    SignalProxy<void(QAction* action)> sigTriggered();

private Q_SLOTS:
    void onHovered(QAction* action);
    void onTriggered(QAction* action);

private:
    Signal<void(QAction*)>* sigHovered_;
    Signal<void(QAction*)>* sigTriggered_;
};

}

#endif
