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
    Action(QObject* parent = nullptr);
    Action(const QString& text, QObject* parent);
    Action(const QIcon& icon, QObject* parent);
    Action(const QIcon& icon, const QString& text, QObject* parent);
    ~Action();

    void setText(const QString& text){ QAction::setText(text); }
    void setText(const std::string& text){ QAction::setText(text.c_str()); }
    void setText(const char* text){ QAction::setText(text); }
                               
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
