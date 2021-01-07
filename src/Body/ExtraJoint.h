/**
   \file
   \author Shizuko Hattori
*/

#ifndef CNOID_BODY_EXTRA_JOINT_H
#define CNOID_BODY_EXTRA_JOINT_H

#include "Link.h"
        
namespace cnoid {

class ExtraJoint
{
public:    
    enum ExtraJointType { EJ_PISTON, EJ_BALL };
    
    ExtraJoint() {
        for(int i=0; i < 2; ++i){
            points[i].setZero();
            links[i] = nullptr;
        }
    };
    ExtraJoint(ExtraJointType type, const Vector3& axis)
        : ExtraJoint() {
        type_ = type;
        axis_ = axis;
    };

    ExtraJointType type() const { return type_; }
    void setType(const ExtraJointType type) { type_ = type; }
    
    const Vector3& axis() const { return axis_; }
    void setAxis(const Vector3& axis) { axis_ = axis; }
    
    Link* link(int which) const { return links[which]; }

    void setLink(int which, Link* link) {
        links[which] = link;
        if(link){
            linkNames[which] = link->name();
        }
    }
         
    const Vector3& point(int which) const { return points[which]; }
    void setPoint(int which, const Vector3& p) { points[which] = p; }
    
    const std::string& bodyName(int which) const { return bodyNames[which]; }
    const std::string& linkName(int which) const { return linkNames[which]; }

    bool isForLinksOfSameBody() const {
        if(links[0] && links[1]){
            return links[0]->body() == links[1]->body();
        }
        return false;
    }

private:
    ExtraJointType type_;
    Vector3 axis_;
    Link* links[2];
    Vector3 points[2];
    std::string bodyNames[2];
    std::string linkNames[2];
};

}

#endif
