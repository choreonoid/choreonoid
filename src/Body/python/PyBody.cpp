/*!
 * @author Hisashi Ikari
 */
#include <boost/python.hpp>
#include <cnoid/Body>
#include <cnoid/BodyState>
#include <cnoid/BodyLoader>
#include <cnoid/MultiValueSeq>
#include <cnoid/BodyMotion>
#include <cnoid/EigenTypes>
#include <cnoid/Link>
#include <cnoid/LinkTraverse>
#include <cnoid/Device>

#include "../ColladaBodyLoader.h"
#include "../VRMLBodyLoader.h"

// We recommend the use of minieigen.
using namespace boost::python;
using namespace cnoid;

namespace cnoid
{
/*
 * @brief This class is a wrapper to the Link.
 *        This wrapper will receive the arguments and self on python(def a(self)). 
 *        It is completely independent to Link for that.
 */
class PyLinkVisitor : public def_visitor<PyLinkVisitor>
{
    friend class def_visitor_access;
public:
    template <class T>
    void visit(T& self) const {
        // Visitor find to match signature on c++.
        self
            // For example, this interface is to match link.p(eigen.Vector3(0,0,0)).
            // And link assignment to self. Vector3 assignment in value.
            .def("p", &PyLinkVisitor::setTranslation)
            .def("R", &PyLinkVisitor::setRotation)
            .def("tb", &PyLinkVisitor::setTb)
            .def("Rs", &PyLinkVisitor::setRs)
            .def("q", &PyLinkVisitor::setQ)
            .def("dq", &PyLinkVisitor::setDq)
            .def("ddq", &PyLinkVisitor::setDdq)
            .def("u", &PyLinkVisitor::setU)
            .def("v", &PyLinkVisitor::setV)
            .def("w", &PyLinkVisitor::setW)
            .def("dv", &PyLinkVisitor::setDv)
            .def("dw", &PyLinkVisitor::setDw)
            .def("setCenterOfMassGlobal", &PyLinkVisitor::setCenterOfMassGlobal)

            // Please decide the correct name.
            .def("setPosition", &PyLinkVisitor::setPosition, return_value_policy<return_by_value>())
            .def("setTb", &PyLinkVisitor::setTb, return_value_policy<return_by_value>())

            // Convert Eigen::Block<Concrete> to concrete manually.
            // Because the miniEigen can not convert this(Part, block).
            // @note We can use method of same name, if signatures are different.
            .def("position", &PyLinkVisitor::position, return_value_policy<return_by_value>())
            .def("translation", &PyLinkVisitor::translation, return_value_policy<return_by_value>())	
            .def("rotation", &PyLinkVisitor::rotation, return_value_policy<return_by_value>())	
            .def("offsetTranslation", &PyLinkVisitor::offsetTranslation, return_value_policy<return_by_value>())	
            .def("offsetRotation", &PyLinkVisitor::offsetRotation, return_value_policy<return_by_value>())	
            ;
    }

    // For example, this interface is to match link.p(eigen.Vector3(0,0,0)).
    // And link assignment to self. Vector3 assignment in value.
    static void setTranslation(Link& self, const Vector3& value) {
        self.translation() = Eigen::Vector3d(value[0], value[1], value[2]);
    }
    static void setPosition(Link& self, const Eigen::MatrixXd& position) {
        Eigen::Matrix<double, 3, 4> result;
        result << position(0, 0), position(0, 1), position(0, 2), position(0, 3),
            position(1, 0), position(1, 1), position(1, 2), position(1, 3),
            position(2, 0), position(2, 1), position(2, 2), position(2, 3);
        self.T() = result;
    }
    static void setTb(Link& self, const Eigen::MatrixXd& position) {
        Eigen::Matrix<double, 3, 4> result;
        result << position(0, 0), position(0, 1), position(0, 2), position(0, 3),
            position(1, 0), position(1, 1), position(1, 2), position(1, 3),
            position(2, 0), position(2, 1), position(2, 2), position(2, 3);
        self.Tb() = result;
    }
    static Eigen::MatrixXd position(Link& self) { 
        Eigen::Matrix<double, 3, 4> result;
        result << self.T()(0, 0), self.T()(0, 1), self.T()(0, 2), self.T()(0, 3),
            self.T()(1, 0), self.T()(1, 1), self.T()(1, 2), self.T()(1, 3),
            self.T()(2, 0), self.T()(2, 1), self.T()(2, 2), self.T()(2, 3);
        return result;
    }
    static void setRotation(Link& self, const Matrix3& value) { self.T().linear() = value.cast<double>(); } 
    static void setRs(Link& self, const Matrix3& value) { self.Rs() = value.cast<double>(); } 
    static void setQ(Link& self, const double& value) { self.q() = value; }
    static void setDq(Link& self, const double& value) { self.dq() = value; }
    static void setDdq(Link& self, const double& value) { self.ddq() = value; }
    static void setU(Link& self, const double& value) { self.u() = value; }

    static void setV(Link& self, const Vector3& value) { self.v() = value; }
    static void setW(Link& self, const Vector3& value) { self.w() = value; }
    static void setDv(Link& self, const Vector3& value) { self.dv() = value; }
    static void setDw(Link& self, const Vector3& value) { self.dw() = value; }
    static void setCenterOfMassGlobal(Link& self, const Vector3& value) { self.wc() = value; }

    static Vector3 translation(Link& self) { return Vector3(self.translation()); }
    static Matrix3 rotation(Link& self) { return Matrix3(self.rotation()); }
    static Vector3 offsetTranslation(Link& self) { return Vector3(self.offsetTranslation()); }
    static Matrix3 offsetRotation(Link& self) { return Matrix3(self.offsetRotation()); }

};


/*
 * @brief This class is a wrapper to the Body.
 *        This wrapper will receive the arguments and self on python(def a(self)). 
 *        It is completely independent to Body for that.
 */
class PyBodyVisitor : public def_visitor<PyBodyVisitor>
{
    friend class def_visitor_access;
public:
    template <class T>
    void visit(T& self) const {
        // Visitor find to match signature on c++.
        self
            // For example, this interface is to match link.p(eigen.Vector3(0,0,0)).
            // And link assignment to self. Vector3 assignment in value.
            .def("defaultPosition", &PyBodyVisitor::defaultPosition)
            ;
    } 
    static Eigen::MatrixXd defaultPosition(Body& self) { 
        Eigen::Matrix<double, 3, 4> result;
        result << self.defaultPosition()(0, 0), self.defaultPosition()(0, 1), 
            self.defaultPosition()(0, 2), self.defaultPosition()(0, 3),
            self.defaultPosition()(1, 0), self.defaultPosition()(1, 1), 
            self.defaultPosition()(1, 2), self.defaultPosition()(1, 3),
            self.defaultPosition()(2, 0), self.defaultPosition()(2, 1), 
            self.defaultPosition()(2, 2), self.defaultPosition()(2, 3);
        return result;
    }
};


/*!
 * @brief Definition of concrete method of pure virtual for BodyLoader.
 */
struct BodyLoaderWrapper : BodyLoader, boost::python::wrapper<BodyLoader>
{
public:
    const char* format() { return this->get_override("format")(); }
    void setVerbose(bool on) { this->get_override("setVerbose")(on); } 
    void enableShapeLoading(bool on) { this->get_override("enableShapeLoading")(on); }
    void setDefaultDivisionNumber(int n) { this->get_override("setDefaultDivisionNumber")(n); }
    void setDefaultCreaseAngle(double theta) { this->get_override("setDefaultCreaseAngle")(theta); }
};

};


namespace cnoid 
{
/*!
 * @brief Reference(pointer) types are explicitly declare a function pointer. 
 *        Reference types share a reference in the original(in C++).
 *        But, it does not share the type in destination(in python).
 *        (Reason-1. Primitive types can not be a reference type.)
 *        (Reason-2. Minieigen create a new object "always" on convertion.)
 *        Therefore, we must use the setter if you want to use the python.
 *        We will define the settings of the following variables of setter in c++.       
 */
BOOST_PYTHON_MODULE(Body)
{
    /*!
     * @brief Definition of access the link-traverse.
     */
    class_<LinkTraverse, boost::noncopyable>("LinkTraverse", init<>())
        .def("find", &LinkTraverse::find)
        .def("numLinks", &LinkTraverse::numLinks, return_value_policy<return_by_value>())
        .def("empty", &LinkTraverse::empty, return_value_policy<return_by_value>())
        .def("size", &LinkTraverse::size, return_value_policy<return_by_value>())
        .def("rootLink", &LinkTraverse::rootLink, return_internal_reference<>())
        .def("link", &LinkTraverse::link, return_internal_reference<>())
        .def("isDownward", &LinkTraverse::isDownward, return_value_policy<return_by_value>())
        .def("calcForwardKinematics", &LinkTraverse::calcForwardKinematics);


    /*!
     * @brief Definition of access the function pointer of link.
     */
    const Vector3& (Link::*jointAxis)() const = &Link::jointAxis;
    Position& (Link::*position)() = & Link::position; 

    Position& (Link::*Tb)() = &Link::Tb;
    Matrix3& (Link::*Rs)() = &Link::Rs;

    double& (Link::*q)() = &Link::q;
    double& (Link::*dq)() = &Link::dq;
    double& (Link::*ddq)() = &Link::ddq;
    double& (Link::*u)() = &Link::u;

    Vector3& (Link::*v)() = &Link::v;
    Vector3& (Link::*w)() = &Link::w;
    Vector3& (Link::*dv)() = &Link::dv;
    Vector3& (Link::*dw)() = &Link::dw;
    Vector6& (Link::*F_ext)() = &Link::F_ext;


    /*!
     * @brief Definition of access the link.
     */
    class_<Link, boost::noncopyable>("Link", no_init) // Prohibition of constructor, reference object only.
        .def(PyLinkVisitor())
        .def("index", &Link::index, return_value_policy<return_by_value>())
        .def("isValid", &Link::isValid, return_value_policy<return_by_value>())
        .def("parent", &Link::parent, return_internal_reference<>())
        .def("sibling", &Link::sibling, return_internal_reference<>())
        .def("child", &Link::child, return_internal_reference<>())
        .def("isRoot", &Link::isRoot, return_value_policy<return_by_value>())

        //.def("position", position, return_value_policy<return_by_value>())
        .def("Tb", Tb, return_value_policy<return_by_value>())
        .def("Rs", Rs, return_value_policy<return_by_value>())

        .def("jointId", &Link::jointId, return_value_policy<return_by_value>())
        .def("jointType", &Link::jointType, return_value_policy<return_by_value>())
        .def("isFixedJoint", &Link::isFixedJoint, return_value_policy<return_by_value>())
        .def("isFreeJoint", &Link::isFreeJoint, return_value_policy<return_by_value>())
        .def("isRotationalJoint", &Link::isRotationalJoint, return_value_policy<return_by_value>())
        .def("isSlideJoint", &Link::isSlideJoint, return_value_policy<return_by_value>())
        .def("jointAxis", &Link::jointAxis, return_internal_reference<>())

        .def("q", q, return_value_policy<return_by_value>())
        .def("dq", dq, return_value_policy<return_by_value>())
        .def("ddq", ddq, return_value_policy<return_by_value>())
        .def("u", u, return_value_policy<return_by_value>())
        .def("q_upper", &Link::q_upper, return_value_policy<return_by_value>())
        .def("q_lower", &Link::q_lower, return_value_policy<return_by_value>())
        .def("dq_upper", &Link::dq_upper, return_value_policy<return_by_value>())
        .def("dq_lower", &Link::dq_lower, return_value_policy<return_by_value>())
        .def("v", v, return_internal_reference<>())
        .def("w", w, return_internal_reference<>())
        .def("dv", dv, return_internal_reference<>())
        .def("dw", dw, return_internal_reference<>())

        .def("centerOfMass", &Link::centerOfMass, return_value_policy<return_by_value>())
        .def("centerOfMassGlobal", &Link::centerOfMassGlobal, return_value_policy<return_by_value>())
        .def("mass", &Link::mass, return_value_policy<return_by_value>())
        .def("I", &Link::I, return_value_policy<return_by_value>())
        .def("Jm2", &Link::Jm2, return_value_policy<return_by_value>())
        .def("F_ext", F_ext, return_internal_reference<>())

        .def("name", &Link::name, return_value_policy<return_by_value>())
        .def("setIndex", &Link::setIndex)
        .def("prependChild", &Link::prependChild)
        .def("appendChild", &Link::appendChild)
        .def("removeChild", &Link::removeChild, return_value_policy<return_by_value>())

        .def("setName", &Link::setName)
        .def("setOffsetPosition", &Link::setOffsetPosition)
        .def("setJointType", &Link::setJointType)
        .def("setJointId", &Link::setJointId)
        .def("setJointAxis", &Link::setJointAxis)
        .def("setJointRange", &Link::setJointRange)
        .def("setJointVelocityRange", &Link::setJointVelocityRange)

        .def("setMass", &Link::setMass)
        .def("setInertia", &Link::setInertia)
        .def("setCenterOfMass", &Link::setCenterOfMass)
        .def("setEquivalentRotorInertia", &Link::setEquivalentRotorInertia)

        .def("attitude", &Link::attitude, return_value_policy<return_by_value>())
        .def("setAttitude", &Link::setAttitude)
        .def("calcRfromAttitude", &Link::calcRfromAttitude, return_value_policy<return_by_value>());

    // (*1)... This result is the same as return_by_value by minieigen. 
    //         We will return to the python pointer of Vector3(Eigen3 on C++). 
    //         But, minieigen converts with a new python_object it.
    // (*2)... It is not available to primitive types. 
    //         You can be found in the help function in python on this. e.g. help(variable)->output is c++ signature.


    /*!
     * @brief Definition for getting of variable link on the Device.
     */ 
    const Link* (Device::*link)() const = &Device::link;
    class_<Device, boost::noncopyable>("Device", no_init)
        .def("setIndex", &Device::setIndex)
        .def("setId", &Device::setId)
        .def("setName", &Device::setName)
        .def("setLink", &Device::setLink)
        .def("clearState", &Device::clearState)
        .def("hasStateOnly", &Device::hasStateOnly, return_value_policy<return_by_value>())
        .def("index", &Device::index, return_value_policy<return_by_value>())
        .def("id", &Device::id, return_value_policy<return_by_value>())
        .def("name", &Device::name, return_value_policy<return_by_value>())
        .def("link", link, return_internal_reference<>())
        .def("cycle", &Device::cycle, return_value_policy<return_by_value>())
        .def("setCycle", &Device::setCycle)
        .def("notifyStateChange", &Device::notifyStateChange);

     
    /*!
     * @brief Definition for getting of variable link on the body.
     */ 
    Link* (Body::*link1)(int) const = &Body::link;
    Link* (Body::*link2)(const std::string&) const = &Body::link;
    const Body::ExtraJoint& (Body::*extraJoint)(int index) const = &Body::extraJoint;
    const Device* (Body::*device)(int index) const = &Body::device;
    bool (Body::*installCustomizer)() = &Body::installCustomizer;


    /*!
     * @brief Definition of access the link.
     */ 
    {
        scope outer = class_<Body, boost::noncopyable>("Body", no_init) // Prohibition of constructor, reference object only.
            .def(PyBodyVisitor())
            .def("createLink", &Body::createLink, return_internal_reference<>(), (args("link") = 0))
            .def("name", &Body::name, return_value_policy<return_by_value>())
            .def("setName", &Body::setName)
            .def("modelName", &Body::modelName, return_value_policy<return_by_value>())
            .def("setModelName", &Body::setModelName)
            .def("setRootLink", &Body::setRootLink)
            .def("updateLinkTree", &Body::updateLinkTree)
            .def("initializeState", &Body::initializeState)
            .def("numJoints", &Body::numJoints, return_value_policy<return_by_value>())
            .def("numVirtualJoints", &Body::numVirtualJoints, return_value_policy<return_by_value>())
            .def("numLinks", &Body::numLinks, return_value_policy<return_by_value>())
            .def("joint", &Body::joint, return_internal_reference<>())        // (*1)
            .def("link", link1, return_internal_reference<>())                // (*1)
            .def("link", link2, return_internal_reference<>())          // (*1)
            .def("rootLink", &Body::rootLink, return_internal_reference<>())  // (*1)
            .def("numDevices", &Body::numDevices, return_value_policy<return_by_value>())
            .def("device", device, return_internal_reference<>())   
            .def("addDevice", &Body::addDevice)
            .def("initializeDeviceStates", &Body::initializeDeviceStates)
            .def("clearDevices", &Body::clearDevices)
            .def("isStaticModel", &Body::isStaticModel, return_value_policy<return_by_value>())
            .def("isFixedRootModel", &Body::isFixedRootModel, return_value_policy<return_by_value>())
            .def("resetDefaultPosition", &Body::resetDefaultPosition, return_value_policy<return_by_value>())
            .def("mass", &Body::mass, return_value_policy<return_by_value>())
            .def("calcCenterOfMass", &Body::calcCenterOfMass, return_value_policy<return_by_value>())
            .def("centerOfMass", &Body::centerOfMass, return_value_policy<return_by_value>())
            .def("calcTotalMomentum", &Body::calcTotalMomentum)
            .def("calcForwardKinematics", &Body::calcForwardKinematics)
            .def("clearExternalForces", &Body::clearExternalForces)
            .def("numExtraJoints", &Body::numExtraJoints, return_value_policy<return_by_value>())
            // I can not the define inner class interface on boost::python now...
            //.def("extraJoint", extraJoint, return_value_policy<return_by_value>())
            .def("addExtraJoint", &Body::addExtraJoint)
            .def("clearExtraJoints", &Body::clearExtraJoints)
            .def("installCustomizer", installCustomizer, return_value_policy<return_by_value>())
            .def("hasVirtualJointForces", &Body::hasVirtualJointForces, return_value_policy<return_by_value>())
            .def("setVirtualJointForces", &Body::setVirtualJointForces)
            .def("addCustomizerDirectory", &Body::addCustomizerDirectory).staticmethod("addCustomizerDirectory");
 
        // (*1)... Reference type is returned this. Because these classes we defined.

        /*!
         * @brief Definition for getting of variable extra-joint on the body.
         * @reference http://goo.gl/Wg29s4
         */
        /*
          class_<Body::ExtraJoint, boost::noncopyable>("ExtraJoint", init<>())
          .def("type", &Body::ExtraJoint::getType, return_value_policy<return_by_value>())
          .def("axis", &Body::ExtraJoint::getAxis, return_value_policy<return_by_value>())
          .def("link", &Body::ExtraJoint::getLink, return_value_policy<return_by_value>())
          .def("point", &Body::ExtraJoint::getPoint, return_value_policy<return_by_value>());
        */
    }

    /*!
     * @brief Definition of access the body.
     */
    void (BodyState::*setRootLinkPosition)(const Position& T) = &BodyState::setRootLinkPosition;
    bool (BodyState::*getRootLinkPosition)(Position& out_T) const = &BodyState::getRootLinkPosition;

    class_<BodyState, boost::noncopyable>("BodyState", no_init)
        .def("storePositions", &BodyState::storePositions)
        .def("restorePositions", &BodyState::restorePositions, return_value_policy<return_by_value>())
        .def("setRootLinkPosition", setRootLinkPosition)
        .def("getRootLinkPosition", getRootLinkPosition, return_value_policy<return_by_value>())
        .def("setZMP", &BodyState::setZMP)
        .def("getZMP", &BodyState::getZMP, return_value_policy<return_by_value>());


    /*!
     * @brief Definition of access the body motion.
     */ 
    void (BodyMotion::*setDimension)(int numFrames, int numJoints, int numLinks, bool clearNewArea) = &BodyMotion::setDimension;
    const MultiValueSeqPtr& (BodyMotion::*jointPosSeq)() const = &BodyMotion::jointPosSeq;
    const MultiSE3SeqPtr& (BodyMotion::*linkPosSeq)() const = &BodyMotion::linkPosSeq;

    class_<BodyMotion, boost::noncopyable>("BodyMotion", no_init)
        .def("setDimension", setDimension, (args("numFrames"), args("numJoints"), args("numLinks"), args("clearNewArea") = false))
        .def("setNumParts", &BodyMotion::setNumParts, (args("numParts"), args("clearNewElements") = false))
        .def("numParts", &BodyMotion::getNumParts, return_value_policy<return_by_value>())
        .def("numJoints", &BodyMotion::numJoints, return_value_policy<return_by_value>())
        .def("numLinks", &BodyMotion::numLinks, return_value_policy<return_by_value>())
        .def("frameRate", &BodyMotion::frameRate, return_value_policy<return_by_value>())
        .def("setFrameRate", &BodyMotion::setFrameRate)
        .def("numFrames", &BodyMotion::numFrames, return_value_policy<return_by_value>())
        .def("setNumFrames", &BodyMotion::setNumFrames)
        .def("jointPosSeq", jointPosSeq, return_value_policy<return_by_value>())
        .def("linkPosSeq", linkPosSeq, return_value_policy<return_by_value>())
        .def("loadStandardYAMLformat", &BodyMotion::loadStandardYAMLformat)
        .def("saveAsStandardYAMLformat", &BodyMotion::saveAsStandardYAMLformat)
        .def("setExtraSeq", &BodyMotion::setExtraSeq)
        .def("clearExtraSeq", &BodyMotion::clearExtraSeq);

    // (*1)... primitive types to copy the value. We can not refer raw-pointer of this value.


    /*!
     * @brief Definition of access the body loader.
     */
    class_<BodyLoaderWrapper, boost::noncopyable>("BodyLoader", no_init)
        .def("format", pure_virtual(&BodyLoaderWrapper::format), return_value_policy<return_by_value>())
        .def("setVerbose", pure_virtual(&BodyLoaderWrapper::setVerbose))
        .def("setShapeLoadingEnabled", pure_virtual(&BodyLoaderWrapper::setShapeLoadingEnabled))
        .def("setDefaultDivisionNumber", pure_virtual(&BodyLoaderWrapper::setDefaultDivisionNumber))
        .def("setDefaultCreaseAngle", pure_virtual(&BodyLoaderWrapper::setDefaultCreaseAngle));


    /*!
     * @brief Definition of access the child body loader.
     */
    class_<ColladaBodyLoader, bases<BodyLoader> , boost::noncopyable>("ColladaBodyLoader", init<>())
        .def("format", &ColladaBodyLoader::format)
        .def("setVerbose", &ColladaBodyLoader::setVerbose)
        .def("setShapeLoadingEnabled", &ColladaBodyLoader::setShapeLoadingEnabled)
        .def("setDefaultDivisionNumber", &ColladaBodyLoader::setDefaultDivisionNumber)
        .def("setDefaultCreaseAngle", &ColladaBodyLoader::setDefaultCreaseAngle);


    /*!
     * @brief Provides following enum value of the Body.
     */
    enum_ <Body::ExtraJointType>("ExtraJointType")
        .value("EJ_PISTON", Body::EJ_PISTON) 
        .value("EJ_BALL", Body::EJ_BALL);


    /*!
     * @brief Provides following enum value of the Link.
     */
    enum_ <Link::JointType>("JointType")
        .value("ROTATIONAL_JOINT", Link::ROTATIONAL_JOINT) 
        .value("SLIDE_JOINT", Link::SLIDE_JOINT) 
        .value("FREE_JOINT", Link::FREE_JOINT) 
        .value("FIXED_JOINT", Link::FIXED_JOINT) 
        .value("CRAWLER_JOINT", Link::CRAWLER_JOINT);


    /*!
     * @brief Provides following enum value of the body motion.
     */
    enum_ <BodyState::DataType>("BodyStateDataType")
        .value("JOINT_POSITIONS", BodyState::JOINT_POSITIONS) 
        .value("LINK_POSITIONS", BodyState::LINK_POSITIONS) 
        .value("JOINT_FORCE_OR_TORQUE", BodyState::JOINT_FORCE_OR_TORQUE) 
        .value("ZMP", BodyState::ZMP);


}

}; // end of namespace
