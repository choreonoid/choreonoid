#ifndef CNOID_BASE_WIDGET_H
#define CNOID_BASE_WIDGET_H

#include <cnoid/Signal>
#include <QWidget>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT Widget : public QWidget
{
    Q_OBJECT

public:
    Widget(QWidget* parent = 0);

    SignalProxy<void(std::string name)> sigObjectNameChanged() {
        return sigObjectNameChanged_;
    }
    SignalProxy<void(std::string title)> sigWindowTitleChanged() {
        return sigWindowTitleChanged_;
    }

private Q_SLOTS:
    void onObjectNameChanged(const QString& objectName);
    void onWindowTitleChanged(const QString& title);

private:
    Signal<void(std::string name)> sigObjectNameChanged_;
    Signal<void(std::string title)> sigWindowTitleChanged_;

    void initialize();
};

}

#endif
