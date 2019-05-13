#ifndef CNOID_BODY_CUSTOM_JOINT_PATH_HANDLER_H
#define CNOID_BODY_CUSTOM_JOINT_PATH_HANDLER_H

#include "BodyHandler.h"
#include "JointPath.h"
#include "Link.h"
#include "exportdecl.h"

namespace cnoid {

class Link;
class JointPath;

class CNOID_EXPORT CustomJointPathHandler : public virtual BodyHandler
{
public:
    virtual std::shared_ptr<JointPath> getCustomJointPath(Link* baseLink, Link* endLink) = 0;

    // Utility functions
    
    static bool checkPath(
        JointPath& jointPath, const std::string& link1Name, const std::string& link2Name, bool& out_isReversed);

    template<typename iterator>
    static void applyInverseKinematicsResults(JointPath& jointPath, iterator results, bool isReversed){
        if(!isReversed){
            for(auto& joint : jointPath.joints()){
                joint->q() = *results++;
            }
        } else {
            for(auto iter = jointPath.joints().rbegin(); iter != jointPath.joints().rend(); ++iter){
                (*iter)->q() = *results++;
            }
        }
        jointPath.calcForwardKinematics();
    }
};

typedef ref_ptr<CustomJointPathHandler> CustomJointPathHandlerPtr;

}

#endif
