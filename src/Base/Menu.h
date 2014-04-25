/**
   @author Shin'ichiro NAKAOKA
*/

#ifndef CNOID_BASE_MENU_H_INCLUDED
#define CNOID_BASE_MENU_H_INCLUDED

#include <cnoid/SignalProxy>
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

    SignalProxy< boost::signal<void(QAction*)> > sigTriggered();
    SignalProxy< boost::signal<void()> > sigAboutToShow();
    SignalProxy< boost::signal<void()> > sigAboutToHide();

private Q_SLOTS:
    void onTriggered(QAction* action);
    void onAboutToShow();
    void onAboutToHide();

private:
    boost::signal<void(QAction*)>* sigTriggered_;
    boost::signal<void()>* sigAboutToShow_;
    boost::signal<void()>* sigAboutToHide_;

    void initialize();
};
}

#endif
