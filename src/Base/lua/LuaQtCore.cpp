/*!
  @author Shin'ichiro Nakaoka
*/

#include <cnoid/LuaUtil>
#include <QObject>
#include <QTimer>

namespace cnoid {

void exportLuaQtCoreTypes(sol::table& module)
{
    module.new_usertype<QObject>(
        "QObject",
        "new", sol::factories([]() { return new QObject; }),
        "blockSignals", &QObject::blockSignals,
        "inherits", &QObject::inherits,
        "isWidgetType", &QObject::isWidgetType,
        "killTimer", &QObject::killTimer,
        "objectName", [](QObject* self) { return self->objectName().toStdString(); },
        "parent", &QObject::parent,
        "setObjectName", [](QObject* self, const char* name) { return self->setObjectName(name); },
        "setParent", &QObject::setParent,
        "startTimer", &QObject::startTimer,
        "deleteLater", &QObject::deleteLater);

    module.new_usertype<QTimer>(
        "QTimer",
        sol::base_classes, sol::bases<QObject>(),
        "new", sol::factories([]() { return new QTimer; }),
        "interval", &QTimer::interval,
        "isActive", &QTimer::isActive,
        "isSingleShot", &QTimer::isSingleShot,
        "setInterval", &QTimer::setInterval,
        "setSingleShot", &QTimer::setSingleShot,
        "timerId", &QTimer::timerId,
        "start", sol::overload(
            [](QTimer* self) { self->start(); },
            [](QTimer* self, int msec) { self->start(msec); }),
        "stop", &QTimer::stop);
}

}


