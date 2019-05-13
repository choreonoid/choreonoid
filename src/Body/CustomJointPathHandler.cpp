#include "CustomJointPathHandler.h"

using namespace cnoid;

bool CustomJointPathHandler::checkPath
(JointPath& jointPath, const std::string& link1Name, const std::string& link2Name, bool& out_isReversed)
{
    const auto& baseLinkName = jointPath.baseLink()->name();
    const auto& endLinkName = jointPath.endLink()->name();
    if(baseLinkName == link1Name && endLinkName == link2Name){
        out_isReversed = false;
        return true;
    } else if(baseLinkName == link2Name && endLinkName == link1Name){
        out_isReversed = true;
        return true;
    }
    return false;
}


