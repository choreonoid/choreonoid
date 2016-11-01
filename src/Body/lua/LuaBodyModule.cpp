/**
   @author Shin'ichiro Nakaoka
*/

#include "../Body.h"
#include <cnoid/EigenUtil>
#include <cnoid/LuaUtil>

using namespace std;
using namespace cnoid;

extern "C" CNOID_EXPORT int luaopen_cnoid_Body(lua_State* L)
{
    sol::state_view lua(L);

    lua["require"]("cnoid.Util");

    sol::table module = lua.create_table();

    module.new_usertype<Link>(
        "Link",
        "new", sol::factories([]() -> LinkPtr { return new Link(); }),
        "index", &Link::index,
        "isValid", &Link::isValid,
        "parent", [](Link* self) -> LinkPtr { return self->parent(); },
        "sibling", [](Link* self) -> LinkPtr { return self->sibling(); },
        "child", [](Link* self) -> LinkPtr { return self->child(); },
        "isRoot", &Link::isRoot,


        //"T", Link_get_position,
        //"position", Link_get_position,
        //"setPosition", Link_set_position,
        //.add_property("p", Link_get_translation, Link_set_translation)
        "translation", [](Link* self) { return make_shared_aligned<Vector3>(self->translation()); },
        "setTranslation", [](Link* self, Vector3& p) { self->setTranslation(p); },

        //.add_property("R", Link_get_rotation, Link_set_rotation)
        //.def("rotation", Link_get_rotation)
        //.def("setRotation", Link_set_rotation)
        //.add_property("Tb", Link_get_Tb, Link_set_Tb)
        //.def("b", Link_get_offsetTranslation)
        //.def("offsetTranslation", Link_get_offsetTranslation)
        //.add_property("Rb", Link_get_offsetRotation)
        //.def("offsetRotation", Link_get_offsetRotation)
        "jointId", &Link::jointId,
        "jointType", &Link::jointType,
        "isFixedJoint", &Link::isFixedJoint,
        "isFreeJoint", &Link::isFreeJoint,
        "isRotationalJoint", &Link::isRotationalJoint,
        "isSlideJoint", &Link::isSlideJoint,
        "a", &Link::a,
        "jointAxis", &Link::jointAxis,
        "d", &Link::d,
        "q", [](Link* self) { return self->q(); },
        "dq", [](Link* self) { return self->dq(); },
        "ddq", [](Link* self) { return self->ddq(); },
        "u", [](Link* self) { return self->u(); },
        "q_upper", &Link::q_upper,
        "q_lower", &Link::q_lower,
        "dq_upper", &Link::dq_upper,
        "dq_lower", &Link::dq_lower,
        "v", [](Link* self) { return self->v(); },
        "w", [](Link* self) { return self->w(); },
        "dv", [](Link* self) { return self->dv(); },
        "dw", [](Link* self) { return self->dw(); },
        "c", &Link::c,
        "centerOfMass", &Link::centerOfMass,
        "wc", [](Link* self) { return self->wc(); },
        "centerOfMassGlobal", &Link::centerOfMassGlobal,
        "m", &Link::m,
        "mass", &Link::mass,
        //.add_property("I", make_function(&Link::I, return_value_policy<return_by_value>()))
        "Jm2", &Link::Jm2,
        //.add_property("F_ext", Link_get_F_ext, Link_set_F_ext)
        //.add_property("f_ext", Link_get_f_ext, Link_set_f_ext)
        //.add_property("tau_ext", Link_get_tau_ext, Link_set_tau_ext)
        "name", &Link::name,
        //.def("visualShape", Link_visualShape)
        //.def("collisionShape", Link_collisionShape)
        "setIndex", &Link::setIndex,
        "prependChild", &Link::prependChild,
        "appendChild", &Link::appendChild,
        "removeChild", &Link::removeChild,
        "setJointType", &Link::setJointType,
        "setJointId", &Link::setJointId,
        "setJointAxis", &Link::setJointAxis,
        "setJointRange", &Link::setJointRange,
        "setJointVelocityRange", &Link::setJointVelocityRange,
        "setMass", &Link::setMass,
        //"setInertia", &Link::setInertia,
        "setCenterOfMass", &Link::setCenterOfMass,
        "setEquivalentRotorInertia", &Link::setEquivalentRotorInertia,
        "setName", &Link::setName
        //"setVisualShape", &Link::setVisualShape,
        //"setCollisionShape", &Link::setCollisionShape,
        //"attitude", &Link::attitude,
        //"setAttitude", &Link::setAttitude,n
        //"calcRfromAttitude", &Link::calcRfromAttitude,
        //"info", Link_info,
        //"floatInfo", Link_floatInfo
        );
    
    module.new_usertype<Body>(
        "Body",
        "new", sol::factories([]() -> BodyPtr { return new Body(); }),
        "clone", [](Body* self) -> BodyPtr { return self->clone(); },
        "createLink", sol::overload(
            [](Body* self) -> LinkPtr { return self->createLink(); },
            [](Body* self, Link* org) -> LinkPtr { return self->createLink(org); }),
        "name", &Body::name,
        "setName", &Body::setName,
        "modelName", &Body::modelName,
        "setModelName", &Body::setModelName,
        "setRootLink", &Body::setRootLink,
        "updateLinkTree", &Body::updateLinkTree,
        "initializeState", &Body::initializeState,
        "numJoints", &Body::numJoints,
        "numVirtualJoints", &Body::numVirtualJoints,
        "numAllJoints", &Body::numAllJoints,
        "joint", [](Body* self, int id) -> LinkPtr { return self->joint(id); },
        "numLinks", &Body::numLinks,
        "link", sol::overload(
            [](Body* self, int index) -> LinkPtr { return self->link(index); },
            [](Body* self, const char* name) -> LinkPtr { return self->link(name); }),
        "rootLink", [](Body* self) -> LinkPtr { return self->rootLink(); },
        "numDevices", &Body::numDevices,
        //"device", [](Body* self, int index) -> DevicePtr { return self->device(index); },
        //"addDevice", &Body::addDevice)
        "initializeDeviceStates", &Body::initializeDeviceStates,
        "clearDevices", &Body::clearDevices,
        "isStaticModel", &Body::isStaticModel,
        "isFixedRootModel", &Body::isFixedRootModel,
        "resetDefaultPosition", &Body::resetDefaultPosition,
        //"defaultPosition", &Body::defaultPosition,
        "mass", &Body::mass,
        "calcCenterOfMass", &Body::calcCenterOfMass,
        "centerOfMass", &Body::centerOfMass,
        "calcForwardKinematics", sol::overload(
            [](Body* self) { self->calcForwardKinematics(); },
            [](Body* self, bool calcVelocity) { self->calcForwardKinematics(calcVelocity); },
            [](Body* self, bool calcVelocity, bool calcAcceleration) { self->calcForwardKinematics(calcVelocity, calcAcceleration); }),
        "clearExternalForces", &Body::clearExternalForces,
        "numExtraJoints", &Body::numExtraJoints,
        //"extraJoint", &Body::extraJoint,
        //"addExtraJoint", &Body::addExtraJoint)
        "clearExtraJoints", &Body::clearExtraJoints,
        //.def("installCustomizer", installCustomizer)
        "hasVirtualJointForces", &Body::hasVirtualJointForces,
        "setVirtualJointForces", &Body::setVirtualJointForces,
        "addCustomizerDirectory", &Body::addCustomizerDirectory,
        "calcTotalMomentum", [](Body* self) { Vector3 P, L; self->calcTotalMomentum(P, L); return std::make_tuple(P, L); }
        );
    
    sol::stack::push(L, module);
    
    return 1;
}
