#include "Widget.h"

using namespace cnoid;

Widget::Widget(QWidget* parent)
    : QWidget(parent)
{
    initialize();
}


void Widget::initialize()
{
    connect(this, SIGNAL(objectNameChanged(const QString&)), this, SLOT(onObjectNameChanged(const QString&)));
    connect(this, SIGNAL(windowTitleChanged(const QString&)), this, SLOT(onWindowTitleChanged(const QString&)));
}


void Widget::onObjectNameChanged(const QString& objectName)
{
    sigObjectNameChanged_(objectName.toStdString());
}


void Widget::onWindowTitleChanged(const QString& title)
{
    sigWindowTitleChanged_(title.toStdString());
}
