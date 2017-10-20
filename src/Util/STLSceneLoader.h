#ifndef CNOID_UTIL_STL_SCENE_LOADER_H
#define CNOID_UTIL_STL_SCENE_LOADER_H

#include "AbstractSceneLoader.h"
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT STLSceneLoader : public AbstractSceneLoader
{
    std::ostream* os_;
    std::ostream& os() { return *os_; }
public:
    STLSceneLoader();
    virtual void setMessageSink(std::ostream& os);
    virtual SgNode* load(const std::string& filename) override;
};

}

#endif
