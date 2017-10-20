/**
 @author Shin'ichiro Nakaoka
*/
#ifndef CNOID_UTIL_ABSTRACT_SCENE_LOADER_H
#define CNOID_UTIL_ABSTRACT_SCENE_LOADER_H

#include "SceneGraph.h"
#include <iosfwd>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT AbstractSceneLoader
{
public:
    virtual ~AbstractSceneLoader();
    virtual void setMessageSink(std::ostream& os);
    virtual void setDefaultDivisionNumber(int n);
    virtual void setDefaultCreaseAngle(double theta);
    virtual SgNode* load(const std::string& filename) = 0;
};

}

#endif
