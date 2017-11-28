/*!
  @author Shin'ichiro Nakaoka
*/

#include <cnoid/LuaUtil>
#include <QWidget>
#include <QMainWindow>
#include <QToolButton>

namespace cnoid {

void exportLuaQtGuiTypes(sol::table& module)
{
    module.new_usertype<QWidget>(
        "QWidget",
        sol::base_classes, sol::bases<QObject>(),
        "new", sol::factories([]() { return new QWidget; }),
        "hasFocus", &QWidget::hasFocus,
        "isActiveWindow", &QWidget::isActiveWindow,
        "isAncestorOf", &QWidget::isAncestorOf,
        "isEnabled", &QWidget::isEnabled,
        "isEnabledTo", &QWidget::isEnabledTo,
        "isFullScreen", &QWidget::isFullScreen,
        "isHidden", &QWidget::isHidden,
        "isMaximized", &QWidget::isMaximized,
        "isMinimized", &QWidget::isMinimized,
        "isModal", &QWidget::isModal,
        "isVisible", &QWidget::isVisible,
        "isVisibleTo", &QWidget::isVisibleTo,
        "isWindow", &QWidget::isWindow,
        "isWindowModified", &QWidget::isWindowModified,
        "parentWidget", &QWidget::parentWidget,
        "setParent", [](QWidget* self, QWidget* parent) { self->setParent(parent); },
        "setToolTip", [](QWidget* self, const char* text) { self->setToolTip(text);},
        "setWhatsThis", [](QWidget* self, const char* text) { self->setWhatsThis(text);},
        "setWindowIconText", [](QWidget* self, const char* text) { self->setWindowIconText(text);},
        "toolTip", [](QWidget* self) { return self->toolTip().toStdString(); },
        "whatsThis", [](QWidget* self) { return self->whatsThis().toStdString(); },
        "window", &QWidget::window,
        "windowIconText", [](QWidget* self) { return self->windowIconText().toStdString(); },
        "windowRole", &QWidget::windowRole,
        "windowTitle",  [](QWidget* self) { return self->windowTitle().toStdString(); },
        "close", &QWidget::close,
        "hide", &QWidget::hide,
        "lower", &QWidget::lower,
        "raise", &QWidget::raise,
        "repaint", [](QWidget* self) { self->repaint(); },
        "setDisabled", &QWidget::setDisabled,
        "setEnabled", &QWidget::setEnabled,
        "setFocus", [](QWidget* self) { self->setFocus(); },
        "setHidden", &QWidget::setHidden,
        "setVisible", &QWidget::setVisible,
        "setWindowModified", &QWidget::setWindowModified,
        "setWindowTitle", [](QWidget* self, const char* title) { self->setWindowTitle(title);},
        "show", &QWidget::show,
        "showFullScreen", &QWidget::showFullScreen,
        "showMaximized", &QWidget::showMaximized,
        "showMinimized", &QWidget::showMinimized,
        "showNormal", &QWidget::showNormal,
        "update", [](QWidget* self) { self->update(); });

    module.new_usertype<QMainWindow>(
        "QMainWindow",
        sol::base_classes, sol::bases<QWidget, QObject>(),
        "new", sol::no_constructor);

    module.new_usertype<QAbstractButton>(
        "QAbstractButton",
        sol::base_classes, sol::bases<QWidget, QObject>(),
        "new", sol::no_constructor,
        "autoExclusive", &QAbstractButton::autoExclusive,
        "autoRepeat", &QAbstractButton::autoRepeat,
        "autoRepeatDelay", &QAbstractButton::autoRepeatDelay,
        "autoRepeatInterval", &QAbstractButton::autoRepeatInterval,
        "isCheckable", &QAbstractButton::isCheckable,
        "isChecked", &QAbstractButton::isChecked,
        "isDown", &QAbstractButton::isDown,
        "setAutoExclusive", &QAbstractButton::setAutoExclusive,
        "setAutoRepeat", &QAbstractButton::setAutoRepeat,
        "setAutoRepeatDelay", &QAbstractButton::setAutoRepeatDelay,
        "setAutoRepeatInterval", &QAbstractButton::setAutoRepeatInterval,
        "setCheckable", &QAbstractButton::setCheckable,
        "setDown", &QAbstractButton::setDown,
        "setText", [](QAbstractButton* self, const char* text) { self->setText(text);},
        "text", [](QAbstractButton* self) { return self->text().toStdString(); },
        "animateClick", [](QAbstractButton* self, int msec) { self->animateClick(msec); },
        "click", &QAbstractButton::click,
        "setChecked", &QAbstractButton::setChecked,
        "toggle", &QAbstractButton::toggle);

    module.new_usertype<QToolButton>(
        "QToolButton",
        sol::base_classes, sol::bases<QAbstractButton, QWidget, QObject>(),
        "new", sol::factories(
            []() { return new QToolButton; }),
        "autoRaise", &QToolButton::autoRaise);
}

}
