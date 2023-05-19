#include <cnoid/EigenUtil>
#include <cnoid/NullOut>
#include <cnoid/SceneDrawables>
#include <cnoid/MeshExtractor>
#include <cnoid/AssimpSceneWriter>
#include <cnoid/DeviceList>

#include "URDFBodyWriter.h"

#include <pugixml.hpp>

#include <iostream>
#include <sstream>
#include <iomanip>
// #include <set>
using namespace cnoid;
//using fmt::format;
//namespace filesystem = cnoid::stdx::filesystem;

#define _SET_PRECISION(strm,prec_)                                      \
    int prc;                                                            \
    prc = strm.precision();                                             \
    strm << std::scientific << std::setprecision(prec_);                \

#define _UNSET_PRECISION(strm)                                          \
    strm << std::fixed << std::setprecision(prc);                       \

//// inline functions for printing
inline void _print_vector(Vector3 &_vec, std::ostream &ostrm)
{
    _SET_PRECISION(ostrm,12);
    ostrm << _vec.x() << " ";
    ostrm << _vec.y() << " ";
    ostrm << _vec.z();
    _UNSET_PRECISION(ostrm);
}

namespace cnoid {

class URDFBodyWriter::Impl
{
public:
    Impl();

    pugi::xml_document createBodyNode(Body* _body);

    std::ostream* os_;
    std::ostream& os() { return *os_; }

    bool use_xacro;
    bool add_geometry;
    bool add_offset;
    bool verbose;
    bool export_devices;

    std::string robot_name;
    std::string mesh_file_prefix;
    std::string mesh_url_prefix;

    int material_counter;
protected:
    void addXacroScripts(pugi::xml_node &_node);
    void addTransmission(pugi::xml_node &_node, const std::string &_joint_name);

    void addGeometry(pugi::xml_node &_node, const cnoid::Link *_lk, bool isVisual);
    void addMassParam(pugi::xml_node &_node, const cnoid::Link *_lk);
    void addMaterial(pugi::xml_node &_node, SgMaterial *mat_);
    void extractPrimitiveGeometry(MeshExtractor &_me, pugi::xml_node &_node, bool isVisual);
};

};  // namespace cnoid
URDFBodyWriter::Impl::Impl()
{
    os_ = &nullout();
    use_xacro = true;
    add_geometry = true;
    add_offset = true;

    export_devices = false;

    material_counter = 0;
}
void URDFBodyWriter::Impl::addXacroScripts(pugi::xml_node &_root)
{
    _root.append_attribute("xmlns:xi"        ) = "http://www.w3.org/2001/Xinclude";
    _root.append_attribute("xmlns:gazebo"    ) = "http://playerstage.sourceforge.net/gazebo/xmlschema/#gz";
    _root.append_attribute("xmlns:model"     ) = "http://playerstage.sourceforge.net/gazebo/xmlschema/#model";
    _root.append_attribute("xmlns:sensor"    ) = "http://playerstage.sourceforge.net/gazebo/xmlschema/#sensor";
    _root.append_attribute("xmlns:body"      ) = "http://playerstage.sourceforge.net/gazebo/xmlschema/#body";
    _root.append_attribute("xmlns:geom"      ) = "http://playerstage.sourceforge.net/gazebo/xmlschema/#geom";
    _root.append_attribute("xmlns:joint"     ) = "http://playerstage.sourceforge.net/gazebo/xmlschema/#joint";
    _root.append_attribute("xmlns:interface" ) = "http://playerstage.sourceforge.net/gazebo/xmlschema/#interface";
    _root.append_attribute("xmlns:rendering" ) = "http://playerstage.sourceforge.net/gazebo/xmlschema/#rendering";
    _root.append_attribute("xmlns:renderable") = "http://playerstage.sourceforge.net/gazebo/xmlschema/#renderable";
    _root.append_attribute("xmlns:controller") = "http://playerstage.sourceforge.net/gazebo/xmlschema/#controller";
    _root.append_attribute("xmlns:physics"   ) = "http://playerstage.sourceforge.net/gazebo/xmlschema/#physics";
    _root.append_attribute("xmlns:xacro"     ) = "http://www.ros.org/wiki/xacro";

    {
    pugi::xml_node xac = _root.append_child("xacro:macro");
    xac.append_attribute("name") = "gz_trans_pos";
    xac.append_attribute("params") = "joint_name";
    pugi::xml_node trans = xac.append_child("transmission");
    trans.append_attribute("name") = "${joint_name}_trans";
    trans.append_child("type")
         .append_child(pugi::node_pcdata)
         .set_value("transmission_interface/SimpleTransmission");
    pugi::xml_node t_joint = trans.append_child("joint");
    t_joint.append_attribute("name") = "${joint_name}";
    t_joint.append_child("hardwareInterface")
           .append_child(pugi::node_pcdata)
           .set_value("hardware_interface/PositionJointInterface");
    pugi::xml_node t_act   = trans.append_child("actuator");
    t_act.append_attribute("name") = "${joint_name}_motor";
    t_act.append_child("hardwareInterface")
         .append_child(pugi::node_pcdata)
         .set_value("hardware_interface/PositionJointInterface");
    }

    {
    pugi::xml_node xac = _root.append_child("xacro:macro");
    xac.append_attribute("name") = "gz_trans_vel";
    xac.append_attribute("params") = "joint_name";
    pugi::xml_node trans = xac.append_child("transmission");
    trans.append_attribute("name") = "${joint_name}_trans";
    trans.append_child("type")
         .append_child(pugi::node_pcdata)
         .set_value("transmission_interface/SimpleTransmission");
    pugi::xml_node t_joint = trans.append_child("joint");
    t_joint.append_attribute("name") = "${joint_name}";
    t_joint.append_child("hardwareInterface")
           .append_child(pugi::node_pcdata)
           .set_value("hardware_interface/VelocityJointInterface");
    pugi::xml_node t_act   = trans.append_child("actuator");
    t_act.append_attribute("name") = "${joint_name}_motor";
    t_act.append_child("hardwareInterface")
         .append_child(pugi::node_pcdata)
         .set_value("hardware_interface/VelocityJointInterface");
    }

    {
    pugi::xml_node xac = _root.append_child("xacro:macro");
    xac.append_attribute("name") = "gz_trans_eff";
    xac.append_attribute("params") = "joint_name";
    pugi::xml_node trans = xac.append_child("transmission");
    trans.append_attribute("name") = "${joint_name}_trans";
    trans.append_child("type")
         .append_child(pugi::node_pcdata)
         .set_value("transmission_interface/SimpleTransmission");
    pugi::xml_node t_joint = trans.append_child("joint");
    t_joint.append_attribute("name") = "${joint_name}";
    t_joint.append_child("hardwareInterface")
           .append_child(pugi::node_pcdata)
           .set_value("hardware_interface/EffortJointInterface");
    pugi::xml_node t_act   = trans.append_child("actuator");
    t_act.append_attribute("name") = "${joint_name}_motor";
    t_act.append_child("hardwareInterface")
         .append_child(pugi::node_pcdata)
         .set_value("hardware_interface/EffortJointInterface");
    }
}
void URDFBodyWriter::Impl::addGeometry(pugi::xml_node &_node, const cnoid::Link *_lk, bool isVisual)
{
    if(isVisual) {
        //// for primitive
        MeshExtractor me_;
        me_.extract(_lk->visualShape(), [this, &me_, &_node](){ extractPrimitiveGeometry(me_, _node, true); });
        //// for mesh
        std::ostringstream oss_fname;
        std::ostringstream oss_url;
        oss_fname << mesh_file_prefix << robot_name << "_" << _lk->name() << ".dae";
        oss_url << mesh_url_prefix << robot_name << "_" << _lk->name() << ".dae";
        AssimpSceneWriter asw;
        if (verbose) {
            asw.setMessageSink(std::cerr);
            asw.setVerbose(true);
        }
        bool res_ =  asw.writeScene(oss_fname.str(), _lk->visualShape());
        if (res_) {
            _node.append_child("visual").append_child("geometry")
                 .append_child("mesh").append_attribute("filename") = oss_url.str().c_str();
        }
    } else {
        //// for primitive
        MeshExtractor me_;
        me_.extract(_lk->collisionShape(), [this, &me_, &_node](){ extractPrimitiveGeometry(me_, _node, false); });
        //// for mesh
        std::ostringstream oss_fname;
        std::ostringstream oss_url;
        oss_fname << mesh_file_prefix << robot_name << "_" << _lk->name() << ".stl";
        oss_url << mesh_url_prefix << robot_name << "_" << _lk->name() << ".stl";
        AssimpSceneWriter asw;
        if (verbose) {
            asw.setMessageSink(std::cerr);
            asw.setVerbose(true);
        }
        asw.setOutputType("stlb");
        bool res_ =  asw.writeScene(oss_fname.str(), _lk->collisionShape());
        if (res_) {
            _node.append_child("collision").append_child("geometry")
                 .append_child("mesh").append_attribute("filename") = oss_url.str().c_str();
        }
    }
}
void URDFBodyWriter::Impl::extractPrimitiveGeometry(MeshExtractor &_me, pugi::xml_node &_node, bool isVisual)
{
    SgMesh* mesh = _me.currentMesh();
    if(mesh->primitiveType() != SgMesh::BOX &&
       mesh->primitiveType() != SgMesh::CYLINDER &&
       mesh->primitiveType() != SgMesh::SPHERE ) {
        //// not primitive in URDF
        if (verbose) {
            std::cerr << "not primitive type in URDF" << std::endl;
        }
        return;
    }
    SgShape* shape = _me.currentShape();
    const Affine3& T = _me.currentTransform();
    const Isometry3 &Ti = _me.currentTransformWithoutScaling();

    pugi::xml_node cur_node;
    if (isVisual) {
        cur_node = _node.append_child("visual");
    } else {
        cur_node = _node.append_child("collision");
    }
    // origin
    {
        Vector3 trs_ = Ti.translation();
        Matrix3 mat_ = Ti.linear();
        if(mesh->primitiveType() == SgMesh::CYLINDER) {
            AngleAxis aa_(-PI_2, Vector3d::UnitX());
            mat_ *= aa_.toRotationMatrix();
        }
        Vector3 rpy_ = cnoid::rpyFromRot(mat_);
        pugi::xml_node o_node = cur_node.append_child("origin");
        {
            std::ostringstream oss;
            _print_vector(trs_, oss);
            o_node.append_attribute("xyz") = oss.str().c_str();
        }
        {
            std::ostringstream oss;
            _print_vector(rpy_, oss);
            o_node.append_attribute("rpy") = oss.str().c_str();
        }
    }
    // primitives
    switch(mesh->primitiveType()) {
    case SgMesh::BOX:
    {
        SgMesh::Box bx = mesh->primitive<SgMesh::Box>();
        pugi::xml_node g_node = cur_node.append_child("geometry").append_child("box");

        std::ostringstream oss;
        _print_vector(bx.size, oss);
        g_node.append_attribute("size") = oss.str().c_str();
    }
    break;
    case SgMesh::CYLINDER:
    {
        SgMesh::Cylinder cyl = mesh->primitive<SgMesh::Cylinder>();
        pugi::xml_node g_node = cur_node.append_child("geometry").append_child("cylinder");

        g_node.append_attribute("radius") = cyl.radius;
        g_node.append_attribute("length") = cyl.height;
    }
    break;
    case SgMesh::SPHERE:
    {
        SgMesh::Sphere sph = mesh->primitive<SgMesh::Sphere>();
        pugi::xml_node g_node = cur_node.append_child("geometry").append_child("sphere");

        g_node.append_attribute("radius") = sph.radius;
    }
    break;
    }
    // material
    SgMaterial *sgmat_ = shape->material();
    if (!!sgmat_) {
        addMaterial(cur_node, sgmat_);
    }
}
void URDFBodyWriter::Impl::addMaterial(pugi::xml_node &_node, SgMaterial *mat_)
{
    std::ostringstream oss;
    oss << "mat_" << mat_->name() << "_" << material_counter;
    pugi::xml_node m_node = _node.append_child("material");
    m_node.append_attribute("name") = oss.str().c_str();
    pugi::xml_node c_node = m_node.append_child("color");
    Vector3f col_ = mat_->diffuseColor();
    float trans_ = mat_->transparency();
    material_counter++;
    {
        std::ostringstream oss;
        oss << col_[0] << " " << col_[1] << " " << col_[2] << " " << 1.0 - trans_;
        c_node.append_attribute("rgba") = oss.str().c_str();
    }
}
void URDFBodyWriter::Impl::addMassParam(pugi::xml_node &_node, const cnoid::Link *_lk)
{
    Vector3 com_ = _lk->centerOfMass();
    Matrix3 Iner_ = _lk->I();
    pugi::xml_node i_node = _node.append_child("inertial");

    i_node.append_child("mass").append_attribute("value") = _lk->mass();

    std::ostringstream oss;
    _print_vector(com_, oss);
    i_node.append_child("origin").append_attribute("xyz") = oss.str().c_str();

    pugi::xml_node in_node = i_node.append_child("inertia");
    in_node.append_attribute("ixx") = Iner_(0,0);
    in_node.append_attribute("ixy") = Iner_(0,1);
    in_node.append_attribute("ixz") = Iner_(0,2);
    in_node.append_attribute("iyy") = Iner_(1,1);
    in_node.append_attribute("iyz") = Iner_(1,2);
    in_node.append_attribute("izz") = Iner_(2,2);
}
void URDFBodyWriter::Impl::addTransmission(pugi::xml_node &_root, const std::string &_joint_name)
{
    if(use_xacro) {
        _root.append_child("xacro:gz_trans_pos").append_attribute("joint_name") = _joint_name.c_str();
    } else {
        std::string jtrs_ = _joint_name + "_trans";
        std::string jmot_ = _joint_name + "_motor";
        pugi::xml_node trans = _root.append_child("transmission");
        trans.append_attribute("name") = jtrs_.c_str();
        trans.append_child("type")
             .append_child(pugi::node_pcdata)
             .set_value("transmission_interface/SimpleTransmission");
        pugi::xml_node t_joint = trans.append_child("joint");
        t_joint.append_attribute("name") = _joint_name.c_str();
        t_joint.append_child("hardwareInterface")
               .append_child(pugi::node_pcdata)
               .set_value("hardware_interface/PositionJointInterface");
        pugi::xml_node t_act   = trans.append_child("actuator");
        t_act.append_attribute("name") = jmot_.c_str();
        t_act.append_child("hardwareInterface")
             .append_child(pugi::node_pcdata)
             .set_value("hardware_interface/PositionJointInterface");
    }
}
pugi::xml_document URDFBodyWriter::Impl::createBodyNode(Body* _body)
{
    pugi::xml_document doc;
    pugi::xml_node root = doc.append_child("robot");

    if(robot_name.empty()) {
        robot_name = _body->modelName();
    }
    root.append_attribute("name") = robot_name.c_str();

    if (use_xacro) {
        addXacroScripts(root);
    }

    for(int i = 0; i < _body->numLinks(); i++) {
        cnoid::Link *cur_lk = _body->link(i);
        pugi::xml_node link_node =  root.append_child("link");
        link_node.append_attribute("name") = cur_lk->name().c_str();
        if (add_geometry) {
            addGeometry(link_node, cur_lk, true);
            addGeometry(link_node, cur_lk, false);
        }

        addMassParam(link_node, cur_lk);

        if(cur_lk->isRoot()) {
            continue;
        }

        std::string _jtype = "fixed";
        switch(cur_lk->jointType()) {
        case cnoid::Link::RevoluteJoint:
            _jtype = "revolute";
            if(cur_lk->q_upper() > 1e16) {
                _jtype = "continuous";
            }
            break;
        case cnoid::Link::PrismaticJoint:
            _jtype = "prismatic";
            break;
        case cnoid::Link::FreeJoint:
            _jtype = "floating";
            break;
        default:
            if (verbose) {
                std::cerr << "jointType : " << cur_lk->jointType() << " / joint : " << cur_lk->jointName() << std::endl;
                std::cerr << " is not a type in URDF, use fixed joint" << std::endl;
            }
        }

        pugi::xml_node joint_node =  root.append_child("joint");
        joint_node.append_attribute("name") = cur_lk->jointName().c_str();
        joint_node.append_attribute("type") = _jtype.c_str();

        cnoid::Link *plk_ = cur_lk->parent();
        if(!!plk_) {
            joint_node.append_child("parent").append_attribute("link") = plk_->name().c_str();
        }
        joint_node.append_child("child").append_attribute("link") = cur_lk->name().c_str();

        if(add_offset) {
            Isometry3::TranslationPart ltrans(cur_lk->translation());
            Isometry3::TranslationPart ptrans(plk_->translation());
            Matrix3 lrot(cur_lk->rotation());
            Matrix3 prot(plk_->rotation());

            Matrix3 prot_inv = prot.transpose();
            Matrix3 trot = prot_inv * lrot;
            Vector3 xyz_ = prot_inv * (ltrans - ptrans);
            Vector3 rpy_ = cnoid::rpyFromRot(trot);

            pugi::xml_node o_node = joint_node.append_child("origin");
            {
                std::ostringstream oss;
                _print_vector(xyz_, oss);
                o_node.append_attribute("xyz") = oss.str().c_str();
            }
            {
                std::ostringstream oss;
                _print_vector(rpy_, oss);
                o_node.append_attribute("rpy") = oss.str().c_str();
            }
        }

        if(!cur_lk->isFixedJoint()) {
            if(add_offset) {
                Vector3 ax_ = cur_lk->a();
                pugi::xml_node ax_node = joint_node.append_child("axis");
                std::ostringstream oss;
                _print_vector(ax_, oss);
                ax_node.append_attribute("xyz") = oss.str().c_str();
            }

            pugi::xml_node lm_node = joint_node.append_child("limit");
            lm_node.append_attribute("lower") = cur_lk->q_lower();
            lm_node.append_attribute("upper") = cur_lk->q_upper();
            lm_node.append_attribute("velocity") = cur_lk->dq_upper();
            lm_node.append_attribute("effort")   = cur_lk->u_upper();

            addTransmission(root, cur_lk->jointName());
        }
    }

    if (export_devices) {
        DeviceList<> devices = _body->devices();
        for(auto it = devices.begin(); it != devices.end(); it++) {
            //Device *dv = *it;
            Link *plk_ = (*it)->link();
            std::string dev_name = (*it)->name();
            std::string jname = "j_" + plk_->name() + "_to_" + (*it)->name();
            pugi::xml_node link_node =  root.append_child("link");
            link_node.append_attribute("name") = dev_name.c_str();

            pugi::xml_node joint_node =  root.append_child("joint");
            joint_node.append_attribute("name") = jname.c_str();
            joint_node.append_attribute("type") = "fixed";

            if(!!plk_) {
                joint_node.append_child("parent").append_attribute("link") = plk_->name().c_str();
            }
            joint_node.append_child("child").append_attribute("link") = dev_name.c_str();

            Isometry3::TranslationPart trans_((*it)->p_local());
            Matrix3 rot_((*it)->R_local());

            Vector3 xyz_ = trans_;
            Vector3 rpy_ = cnoid::rpyFromRot(rot_);

            pugi::xml_node o_node = joint_node.append_child("origin");
            {
                std::ostringstream oss;
                _print_vector(xyz_, oss);
                o_node.append_attribute("xyz") = oss.str().c_str();
            }
            {
                std::ostringstream oss;
                _print_vector(rpy_, oss);
                o_node.append_attribute("rpy") = oss.str().c_str();
            }
        }
    }

    return doc;
}
URDFBodyWriter::URDFBodyWriter()
{
    impl = new Impl();
}
URDFBodyWriter::~URDFBodyWriter()
{
    delete impl;
}
void URDFBodyWriter::setMessageSink(std::ostream& os)
{
    impl->os_ = &os;
}
void URDFBodyWriter::setMessageSinkStdErr()
{
    setMessageSink(std::cerr);
}
void URDFBodyWriter::setVerbose(bool _on)
{
    impl->verbose = _on;
}
bool URDFBodyWriter::writeBody(Body* body, const std::string& filename)
{
    pugi::xml_document  doc_ = impl->createBodyNode(body);
    return doc_.save_file(filename.c_str());
}
void URDFBodyWriter::writeBodyAsString(Body* body, std::string& outstr)
{
    pugi::xml_document  doc_ = impl->createBodyNode(body);
    std::ostringstream oss;
    doc_.save(oss, " ");
    outstr = oss.str();
}
void URDFBodyWriter::setUseXacro(bool _on) { impl->use_xacro = _on; }
void URDFBodyWriter::setAddGeometry(bool _on) { impl->add_geometry = _on; }
void URDFBodyWriter::setAddOffset(bool _on) { impl->add_offset = _on; }
void URDFBodyWriter::setExportDevices(bool _on) { impl->export_devices = _on; }
void URDFBodyWriter::setRobotName(const std::string &_name) { impl->robot_name = _name; }
void URDFBodyWriter::setMeshFilePrefix(const std::string &_pref) { impl->mesh_file_prefix = _pref; }
void URDFBodyWriter::setMeshURLPrefix(const std::string &_pref)  { impl->mesh_url_prefix  = _pref; }
