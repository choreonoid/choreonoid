/*!
  @author Shin'ichiro Nakaoka
 */

#include "../Body.h"
#include "../BodyLoader.h"
#include <cnoid/PyUtil>

using namespace boost;
using namespace boost::python;
using namespace cnoid;

namespace
{

Vector3 Link_get_translation(Link& self) { return self.translation(); }
void Link_set_translation(Link& self, const Vector3& p) { self.translation() = p; }
double Link_get_q(Link& self) { return self.q(); }
void Link_set_q(Link& self, double q) { self.q() = q; }
double Link_get_dq(Link& self) { return self.dq(); }
void Link_set_dq(Link& self, double dq) { self.dq() = dq; }
double Link_get_ddq(Link& self) { return self.ddq(); }
void Link_set_ddq(Link& self, double ddq) { self.ddq() = ddq; }
double Link_get_u(Link& self) { return self.u(); }
void Link_set_u(Link& self, double u) { self.u() = u; }
Vector3 Link_get_v(Link& self) { return self.v(); }
void Link_set_v(Link& self, const Vector3& v) { self.v() = v; }
Vector3 Link_get_w(Link& self) { return self.w(); }
void Link_set_w(Link& self, const Vector3& w) { self.w() = w; }
Vector3 Link_get_dv(Link& self) { return self.dv(); }
void Link_set_dv(Link& self, const Vector3& dv) { self.dv() = dv; }
Vector3 Link_get_dw(Link& self) { return self.dw(); }
void Link_set_dw(Link& self, const Vector3& dw) { self.dw() = dw; }
Vector3 Link_get_wc(Link& self) { return self.wc(); }
void Link_set_wc(Link& self, const Vector3& wc) { self.wc() = wc; }
SgNodePtr Link_shape(const Link& self) { return self.shape(); }

BodyPtr Body_clone(Body& self) { return self.clone(); }
Link* (Body::*Body_link1)(int) const = &Body::link;
Link* (Body::*Body_link2)(const std::string&) const = &Body::link;
DevicePtr Body_device(Body& self, int index) { return self.device(index); }

PyObject* Body_calcTotalMomentum(Body& self) {
    Vector3 P, L;
    self.calcTotalMomentum(P, L);
    python::list p, l;
    for(int i=0; i < 3; ++i){
        p.append(P[i]);
        l.append(L[i]);
    }
    python::tuple ret = python::make_tuple(p, l);
    return python::incref(ret.ptr());
}

BodyPtr (BodyLoader::*BodyLoader_load2)(const std::string&) = &BodyLoader::load;
    
} // namespace

namespace cnoid 
{

BOOST_PYTHON_MODULE(Body)
{
    boost::python::import("cnoid.Util");

    {
        scope linkScope = 
            class_<Link, Link*>("Link")
            .def("index", &Link::index)
            .def("isValid", &Link::isValid)
            .def("parent", &Link::parent, return_value_policy<reference_existing_object>())
            .def("sibling", &Link::sibling, return_value_policy<reference_existing_object>())
            .def("child", &Link::child, return_value_policy<reference_existing_object>())
            .def("isRoot", &Link::isRoot)
            .add_property("p", Link_get_translation, Link_set_translation)
            .def("translation", Link_get_translation)
            .def("setTranslation", Link_set_translation)
            //.add_property("R", Link_get_rotation, Link_set_rotation)
            //.def("rotation", Link_get_rotation)
            //.def("setRotation", Link_set_rotation)
            .def("jointId", &Link::jointId)
            .def("jointType", &Link::jointType)
            .def("isFixedJoint", &Link::isFixedJoint)
            .def("isFreeJoint", &Link::isFreeJoint)
            .def("isRotationalJoint", &Link::isRotationalJoint)
            .def("isSlideJoint", &Link::isSlideJoint)
            .add_property("a", make_function(&Link::a, return_value_policy<return_by_value>()))
            .def("jointAxis", &Link::jointAxis, return_value_policy<return_by_value>())
            .add_property("d", make_function(&Link::d, return_value_policy<return_by_value>()))
            .add_property("q", Link_get_q, Link_set_q)
            .add_property("dq", Link_get_dq, Link_set_dq)
            .add_property("ddq", Link_get_ddq, Link_set_ddq)
            .add_property("u", Link_get_u, Link_set_u)
            .add_property("q_upper", &Link::q_upper)
            .add_property("q_lower", &Link::q_lower)
            .add_property("dq_upper", &Link::dq_upper)
            .add_property("dq_lower", &Link::dq_lower)
            .add_property("v", Link_get_v, Link_set_v)
            .add_property("w", Link_get_w, Link_set_w)
            .add_property("dv", Link_get_dv, Link_set_dv)
            .add_property("dw", Link_get_dw, Link_set_dw)
            .add_property("c", make_function(&Link::c, return_value_policy<return_by_value>()))
            .def("centerOfMass", &Link::centerOfMass, return_value_policy<return_by_value>())
            .add_property("wc", make_function(Link_get_wc, return_value_policy<return_by_value>()), Link_set_wc)
            .def("centerOfMassGlobal", &Link::centerOfMassGlobal, return_value_policy<return_by_value>())
            .add_property("m", &Link::m)
            .def("mass", &Link::mass)
            //.add_property("I", &Link::I, return_value_policy<return_by_value>())
            .add_property("Jm2", &Link::Jm2)
            //.add_property("F_ext", make_function(Link_get_F_ext, return_value_policy<return_by_value>()), Link_set_F_ext)
            //.add_property("f_ext", make_function(Link_get_f_ext, return_value_policy<return_by_value>()), Link_set_f_ext)
            //.add_property("tau_ext", make_function(Link_get_tau_ext, return_value_policy<return_by_value>()), Link_set_tau_ext)
            .def("name", &Link::name, return_value_policy<copy_const_reference>())
            .def("shape", Link_shape)
            .def("setIndex", &Link::setIndex)
            .def("prependChild", &Link::prependChild)
            .def("appendChild", &Link::appendChild)
            .def("removeChild", &Link::removeChild)
            .def("setJointType", &Link::setJointType)
            .def("setJointId", &Link::setJointId)
            .def("setJointAxis", &Link::setJointAxis)
            .def("setJointRange", &Link::setJointRange)
            .def("setJointVelocityRange", &Link::setJointVelocityRange)
            .def("setMass", &Link::setMass)
            //.def("setInertia", &Link::setInertia)
            .def("setCenterOfMass", &Link::setCenterOfMass)
            .def("setEquivalentRotorInertia", &Link::setEquivalentRotorInertia)
            .def("setName", &Link::setName)
            .def("setShape", &Link::setShape)
            .def("attitude", &Link::attitude)
            .def("setAttitude", &Link::setAttitude)
            .def("calcRfromAttitude", &Link::calcRfromAttitude)
            ;

        enum_<Link::JointType>("JointType")
            .value("ROTATIONAL_JOINT", Link::ROTATIONAL_JOINT) 
            .value("SLIDE_JOINT", Link::SLIDE_JOINT) 
            .value("FREE_JOINT", Link::FREE_JOINT) 
            .value("FIXED_JOINT", Link::FIXED_JOINT) 
            .value("CRAWLER_JOINT", Link::CRAWLER_JOINT);
    }        
    
    {
        scope bodyScope =
            class_< Body, BodyPtr, bases<Referenced> >("Body")
            .def("clone", Body_clone)
            .def("createLink", &Body::createLink, return_value_policy<reference_existing_object>())
            .def("name", &Body::name, return_value_policy<copy_const_reference>())
            .def("setName", &Body::setName)
            .def("modelName", &Body::modelName, return_value_policy<copy_const_reference>())
            .def("setModelName", &Body::setModelName)
            .def("setRootLink", &Body::setRootLink)
            .def("updateLinkTree", &Body::updateLinkTree)
            .def("initializeState", &Body::initializeState)
            .def("numJoints", &Body::numJoints)
            .def("numVirtualJoints", &Body::numVirtualJoints)
            .def("numAllJoints", &Body::numAllJoints)
            .def("joint", &Body::joint, return_value_policy<reference_existing_object>())
            .def("numLinks", &Body::numLinks)
            .def("link", Body_link1, return_value_policy<reference_existing_object>())
            .def("link", Body_link2, return_value_policy<reference_existing_object>())
            .def("rootLink", &Body::rootLink, return_value_policy<reference_existing_object>())
            .def("numDevices", &Body::numDevices)
            .def("device", Body_device)
            .def("addDevice", &Body::addDevice)
            .def("initializeDeviceStates", &Body::initializeDeviceStates)
            .def("clearDevices", &Body::clearDevices)
            .def("isStaticModel", &Body::isStaticModel)
            .def("isFixedRootModel", &Body::isFixedRootModel)
            .def("resetDefaultPosition", &Body::resetDefaultPosition)
            //.def("defaultPosition", &Body::defaultPosition)
            .def("mass", &Body::mass)
            .def("calcCenterOfMass", &Body::calcCenterOfMass, return_value_policy<return_by_value>())
            .def("centerOfMass", &Body::centerOfMass, return_value_policy<return_by_value>())
            .def("calcTotalMomentum", Body_calcTotalMomentum)
            .def("calcForwardKinematics", &Body::calcForwardKinematics)
            .def("clearExternalForces", &Body::clearExternalForces)
            .def("numExtraJoints", &Body::numExtraJoints)
            //.def("extraJoint", extraJoint, , return_value_policy<reference_existing_object>())
            //.def("addExtraJoint", &Body::addExtraJoint)
            .def("clearExtraJoints", &Body::clearExtraJoints)
            //.def("installCustomizer", installCustomizer)
            .def("hasVirtualJointForces", &Body::hasVirtualJointForces)
            .def("setVirtualJointForces", &Body::setVirtualJointForces)
            .def("addCustomizerDirectory", &Body::addCustomizerDirectory).staticmethod("addCustomizerDirectory")
            ;

        enum_<Body::ExtraJointType>("ExtraJointType")
            .value("EJ_PISTON", Body::EJ_PISTON) 
            .value("EJ_BALL", Body::EJ_BALL);
    }

    class_<AbstractBodyLoader, boost::noncopyable>("AbstractBodyLoader", no_init)
        .def("format", &AbstractBodyLoader::format)
        .def("setVerbose", &AbstractBodyLoader::setVerbose)
        .def("setShapeLoadingEnabled", &AbstractBodyLoader::setShapeLoadingEnabled)
        .def("setDefaultDivisionNumber", &AbstractBodyLoader::setDefaultDivisionNumber)
        .def("setDefaultCreaseAngle", &AbstractBodyLoader::setDefaultCreaseAngle)
        .def("load", &AbstractBodyLoader::load)
        ;

    class_<BodyLoader, bases<AbstractBodyLoader> >("BodyLoader")
        .def("load", BodyLoader_load2)
        .def("lastActualBodyLoader", &BodyLoader::lastActualBodyLoader)
        ;
}

}; // namespace cnoid
