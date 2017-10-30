/**
   \file
   \author Shizuko Hattori
*/

#ifndef CNOID_BODY_EXTRA_JOINT_H
#define CNOID_BODY_EXTRA_JOINT_H
        
namespace cnoid {

struct ExtraJoint {
    
    enum ExtraJointType { EJ_PISTON, EJ_BALL };
    
    ExtraJoint() {};
    ExtraJoint(ExtraJointType type, const Vector3& axis) : type(type), axis(axis) { };
    void setType(const ExtraJointType type_) { type = type_; }
    void setAxis(const Vector3& axis_) { axis = axis_; }
    void setPoint(int i, const std::string& body_, const std::string& link_, const Vector3& point_ ) {
        if(i>1) return;

        bodyName[i] = body_;
        linkName[i] = link_;
        point[i] = point_;
        body[i] = 0;
        link[i] = 0;
    };

    ExtraJointType type;
    Vector3 axis;
    Body* body[2];
    Link* link[2];
    Vector3 point[2];
    std::string bodyName[2];
    std::string linkName[2];
};

}

#endif
