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
        "blockSignals", [](QObject& self, bool b){ return self.blockSignals(b); },
        "inherits", &QObject::inherits,
        "isWidgetType", &QObject::isWidgetType,
        "killTimer", &QObject::killTimer,
        "objectName", [](QObject* self) { return self->objectName().toStdString(); },
        "parent", &QObject::parent,
        "setObjectName", [](QObject* self, const char* name) { return self->setObjectName(name); },
        "setParent", &QObject::setParent,
        "startTimer", [](QObject* self, int interval){ return self->startTimer(interval); },
        "deleteLater", &QObject::deleteLater
        );

    module.new_usertype<QTimer>(
        "QTimer",
        sol::base_classes, sol::bases<QObject>(),
        "new", sol::factories([]() { return new QTimer; }),
        "interval", &QTimer::interval,
        "isActive", &QTimer::isActive,
        "isSingleShot", &QTimer::isSingleShot,
        "setInterval", [](QTimer* self, int msec){ self->setInterval(msec); },
        "setSingleShot", &QTimer::setSingleShot,
        "timerId", &QTimer::timerId,
        "start", sol::overload(
            [](QTimer* self) { self->start(); },
            [](QTimer* self, int msec) { self->start(msec); }),
        "stop", &QTimer::stop);
}

}


