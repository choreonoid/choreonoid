/**
   @author Shin'ichiro NAKAOKA
*/

#ifndef CNOID_BASE_MENU_H
#define CNOID_BASE_MENU_H

#include <cnoid/Signal>
#include <QMenu>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT Menu : public QMenu
{
    Q_OBJECT

public:
    Menu(QWidget* parent = 0);
    Menu(const QString& title, QWidget* parent = 0);
    ~Menu();

    SignalProxy<void(QAction*)> sigTriggered();
    SignalProxy<void()> sigAboutToShow();
    SignalProxy<void()> sigAboutToHide();

private Q_SLOTS:
    void onTriggered(QAction* action);
    void onAboutToShow();
    void onAboutToHide();

private:
    Signal<void(QAction*)>* sigTriggered_;
    Signal<void()>* sigAboutToShow_;
    Signal<void()>* sigAboutToHide_;

    void initialize();
};

}

#endif
