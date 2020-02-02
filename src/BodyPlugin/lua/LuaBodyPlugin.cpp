/**
   @author Shin'ichiro Nakaoka
*/

#include "../BodyItem.h"
#include <cnoid/LuaUtil>
#include <cnoid/LuaSignal>
#include <cnoid/LuaItemList>

using namespace std;
using namespace cnoid;

extern "C" CNOID_EXPORT int luaopen_cnoid_BodyPlugin(lua_State* L)
{
    sol::state_view lua(L);

    lua["require"]("cnoid.Base");
    lua["require"]("cnoid.Body");

    sol::table module = lua.create_table();
    
    module.new_usertype<BodyItem>(
        "BodyItem",
        sol::base_classes, sol::bases<Item>(),
        "new", sol::factories([]() -> BodyItemPtr { return new BodyItem(); }),
        "cast", [](Item* item) -> BodyItemPtr { return dynamic_cast<BodyItem*>(item); },
        "setName", &BodyItem::setName,
        "body", [](BodyItem* self) -> BodyPtr { return self->body(); },
        "isEditable", &BodyItem::isEditable,
        "moveToOrigin", &BodyItem::moveToOrigin,
        "setPresetPose", &BodyItem::setPresetPose,
        "currentBaseLink", [](BodyItem* self) -> LinkPtr { return self->currentBaseLink(); },
        "setCurrentBaseLink", &BodyItem::setCurrentBaseLink,
        "calcForwardKinematics", sol::overload(
            [](BodyItem* self) { self->calcForwardKinematics(); },
            [](BodyItem* self, bool calcVelocity) { self->calcForwardKinematics(calcVelocity); },
            [](BodyItem* self, bool calcVelocity, bool calcAcceleration) { self->calcForwardKinematics(calcVelocity, calcAcceleration); }),
        "copyKinematicState", &BodyItem::copyKinematicState,
        "pasteKinematicState", &BodyItem::pasteKinematicState,
        //"storeKinematicState", &BodyItem::storeKinematicState,
        //"restoreKinematicState", &BodyItem::restoreKinematicState,
        "storeInitialState", &BodyItem::storeInitialState,
        "restoreInitialState", sol::overload(
            [](BodyItem* self) { self->restoreInitialState(); },
            [](BodyItem* self, bool doNotify) { self->restoreInitialState(doNotify); }),
        //"getInitialState", &BodyItem::getInitialState,
        "beginKinematicStateEdit", &BodyItem::beginKinematicStateEdit,
        "acceptKinematicStateEdit", &BodyItem::acceptKinematicStateEdit,
        "undoKinematicState", &BodyItem::undoKinematicState,
        "redoKinematicState", &BodyItem::redoKinematicState,
        //"sigKinematicStateChanged", &BodyItem::sigKinematicStateChanged,
        "notifyKinematicStateChange", sol::overload(
            [](BodyItem* self) { self->notifyKinematicStateChange(); },
            [](BodyItem* self, bool calcVelocity) { self->notifyKinematicStateChange(calcVelocity); },
            [](BodyItem* self, bool calcVelocity, bool calcAcceleration) { self->notifyKinematicStateChange(calcVelocity, calcAcceleration); }),
        "enableCollisionDetection", &BodyItem::enableCollisionDetection,
        "isCollisionDetectionEnabled", &BodyItem::isCollisionDetectionEnabled,
        "enableSelfCollisionDetection", &BodyItem::enableSelfCollisionDetection,
        "isSelfCollisionDetectionEnabled", &BodyItem::isSelfCollisionDetectionEnabled,
        "clearCollisions", &BodyItem::clearCollisions,
        "centerOfMass", &BodyItem::centerOfMass,
        "doLegIkToMoveCm", &BodyItem::doLegIkToMoveCm,
        "zmp", &BodyItem::zmp,
        "setZmp", &BodyItem::setZmp,
        "setStance", &BodyItem::setStance
        );

    LuaItemList<BodyItem>("BodyItem", module);

    sol::stack::push(L, module);
    
    return 1;
}
