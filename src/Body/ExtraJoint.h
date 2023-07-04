#ifndef CNOID_BODY_EXTRA_JOINT_H
#define CNOID_BODY_EXTRA_JOINT_H

#include "Link.h"
#include <cnoid/Referenced>
#include <cnoid/ValueTree>
#include "exportdecl.h"
        
namespace cnoid {

class CNOID_EXPORT ExtraJoint : public Referenced
{
public:    
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    enum ExtraJointType {
        Fixed,
        Hinge,
        Ball,
        Piston,
        EJ_BALL [[deprecated]] = Ball,
        EJ_PISTON [[deprecated]] = Piston
    };
    
    ExtraJoint() {
        for(int i=0; i < 2; ++i){
            T[i].setIdentity();
            links[i] = nullptr;
        }
    };
    ExtraJoint(ExtraJointType type)
        : ExtraJoint() {
        type_ = type;
    };
    ExtraJoint(ExtraJointType type, const Vector3& axis)
        : ExtraJoint() {
        type_ = type;
        setAxis(axis);
    };

    ExtraJoint(const ExtraJoint& org) = delete;
    ExtraJoint& operator=(const ExtraJoint& rhs) = delete;

    ExtraJointType type() const { return type_; }
    void setType(const ExtraJointType type) { type_ = type; }
    
    Link* link(int which) const { return links[which]; }

    void setLink(int which, Link* link) {
        links[which] = link;
    }

    void setLocalPosition(int which, const Isometry3& T){ this->T[which] = T; }
    const Isometry3& localPosition(int which) const { return T[which]; }

    void setLocalRotation(int which, const Matrix3& R){ this->T[which].linear() = R; }
    Isometry3::ConstLinearPart localRotation(int which) const { return T[which].linear(); }

    void setLocalTranslation(int which, const Vector3& p){ this->T[which].translation() = p; }
    Isometry3::ConstTranslationPart localTranslation(int which) const { return T[which].translation(); }
    
    Vector3 axis(int which = 0) const { return T[which].linear().col(0); }
    void setAxis(const Vector3& axis) { setAxis(0, axis); }
    void setAxis(int which, const Vector3& axis) {
        auto rot = Quaternion::FromTwoVectors(Vector3::UnitX(), axis.normalized());
        T[which].linear() = rot * Matrix3::Identity();
    }
    
    Isometry3::ConstTranslationPart point(int which) const { return localTranslation(which); }
    void setPoint(int which, const Vector3& p) { setLocalTranslation(which, p); }
    
    std::string linkName(int which) const {
        if(auto link = links[which]){
            return link->name();
        }
        return std::string();
    }

    std::string bodyName(int which) const;

    bool isForLinksOfSameBody() const {
        if(links[0] && links[1]){
            return links[0]->body() == links[1]->body();
        }
        return false;
    }

    const Mapping* info() const { return info_; }
    Mapping* info() { return info_; }
    void resetInfo(Mapping* info){ info_ = info; }

private:
    ExtraJointType type_;
    LinkPtr links[2];

    /**
       Coordinate frames of the joint in link local coordinates.
       Note that T[x].linear().col(0) corresponds to the joint axis in each link.
    */
    Isometry3 T[2];
    
    MappingPtr info_;
};

typedef ref_ptr<ExtraJoint> ExtraJointPtr;

}

#endif
