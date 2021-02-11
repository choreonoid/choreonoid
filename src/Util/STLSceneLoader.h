/*!
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_STL_SCENE_LOADER_H
#define CNOID_UTIL_STL_SCENE_LOADER_H

#include "AbstractSceneLoader.h"
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT STLSceneLoader : public AbstractSceneLoader
{
public:
    STLSceneLoader();
    ~STLSceneLoader();
    virtual void setMessageSink(std::ostream& os) override;
    virtual SgNode* load(const std::string& filename) override;

private:
    class Impl;
    Impl* impl;
};

}

#endif
