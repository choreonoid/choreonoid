#ifndef CNOID_BODY_CUSTOM_JOINT_PATH_BASE_H
#define CNOID_BODY_CUSTOM_JOINT_PATH_BASE_H

#include "JointPath.h"
#include "Link.h"
#include "exportdecl.h"

namespace cnoid {

/**
   This class can be used as a base class of a custom joint path class whose instance is provided
   by CustomJointPathHandler.
   Using this class will make the implementation of a custom joint path class easier in most cases,
   but you don't necessarily have to use this class to implement a custom joint path class.
   See "sample/GRobotPlugin/GRobotHandler.cpp" as a sample of using this class.
*/
class CNOID_EXPORT CustomJointPathBase : public JointPath
{
    typedef std::function<bool(const Position& T_global, const Position& T_relative)> InverseKinematicsFunc;
    InverseKinematicsFunc calcCustomInverseKinematics;
    bool isReversed_;
    
public:
    CustomJointPathBase(Link* baseLink, Link* endLink);
    
    bool checkLinkPath(const std::string& from, const std::string& to, bool& out_isReversed) const;

    void setCustomInverseKinematics(InverseKinematicsFunc func, bool isReversed = false);
    InverseKinematicsFunc customInverseKinematics() const { return calcCustomInverseKinematics; }
    bool isReversed() const { return isReversed_; }

    virtual bool calcInverseKinematics(const Position& T) override;
    virtual bool hasCustomIK() const override;

    template<typename iterator> void copyJointDisplacements(iterator q_iter){
        if(!isReversed_){
            for(auto& joint : joints()){
                joint->q() = *q_iter++;
            }
        } else {
            for(auto j_iter = joints().rbegin(); j_iter != joints().rend(); ++j_iter){
                (*j_iter)->q() = *q_iter++;
            }
        }
    }
};

}

#endif
