/**
   @author Shin'ichiro NAKAOKA
*/

#ifndef CNOID_BASE_ACTION_H_INCLUDED
#define CNOID_BASE_ACTION_H_INCLUDED

#include <cnoid/SignalProxy>
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
                               
    SignalProxy< boost::signal<void(void)> > sigTriggered();
    SignalProxy< boost::signal<void(bool)> > sigToggled();

private Q_SLOTS:
    void onTriggered(bool checked);
    void onToggled(bool checked);

private:
    boost::signal<void(void)>* sigTriggered_;
    boost::signal<void(bool)>* sigToggled_;

    void initialize();
};
}

#endif

