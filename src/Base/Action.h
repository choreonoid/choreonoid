/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_ACTION_H
#define CNOID_BASE_ACTION_H

#include <cnoid/Signal>
#include <QAction>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT Action : public QAction
{
    Q_OBJECT

public:
    Action(QObject* parent);
    Action(const QString& text, QObject* parent);
    Action(const QIcon& icon, QObject* parent);
    Action(const QIcon& icon, const QString& text, QObject* parent);
    ~Action();
                               
    SignalProxy<void()> sigTriggered();
    SignalProxy<void(bool)> sigToggled();

private Q_SLOTS:
    void onTriggered(bool checked);
    void onToggled(bool checked);

private:
    Signal<void(void)>* sigTriggered_;
    Signal<void(bool)>* sigToggled_;

    void initialize();
};

}

#endif
