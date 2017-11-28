/*!
 * @author Shin'ichiro Nakaoka
*/

#include "../MessageView.h"
#include "../TaskView.h"
#include <cnoid/LuaUtil>

namespace cnoid {

void exportLuaViews(sol::table& module)
{
    module.new_usertype<View>(
        "View",
        sol::base_classes, sol::bases<QWidget, QObject>(),
        "new", sol::no_constructor,
        "name", &View::name,
        "isActive", &View::isActive,
        "bringToFront", &View::bringToFront,
        "lastFocusView", &View::lastFocusView
        );

    module.new_usertype<MessageView>(
        "MessageView",
        sol::base_classes, sol::bases<View, QWidget, QObject>(),
        "new", sol::no_constructor,
        "instance", &MessageView::instance,
        "put", [](MessageView* self, const char* msg){ self->put(msg); },
        "putln", [](MessageView* self, const char* msg){ self->putln(msg); },
        "notify", [](MessageView* self, const char* msg){ self->notify(msg); },
        "flush", &MessageView::flush,
        "clear", &MessageView::clear
        );

    module.new_usertype<TaskView>(
        "TaskView",
        sol::base_classes, sol::bases<View, QWidget, QObject, AbstractTaskSequencer>(),
        "new", sol::no_constructor,
        "instance", &TaskView::instance
        );
}

}
