/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_ACTION_GROUP_H
#define CNOID_BASE_ACTION_GROUP_H

#include <cnoid/SignalProxy>
#include <QActionGroup>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT ActionGroup : public QActionGroup
{
    Q_OBJECT

public:
    ActionGroup(QObject* parent);
    ~ActionGroup();
                               
    SignalProxy< boost::signal<void(QAction* action)> > sigHovered();
    SignalProxy< boost::signal<void(QAction* action)> > sigTriggered();

private Q_SLOTS:
    void onHovered(QAction* action);
    void onTriggered(QAction* action);

private:
    boost::signal<void(QAction*)>* sigHovered_;
    boost::signal<void(QAction*)>* sigTriggered_;
};

}

#endif
