/*!
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_STL_SCENE_LOADER_H
#define CNOID_UTIL_STL_SCENE_LOADER_H

#include "AbstractSceneLoader.h"
#include "exportdecl.h"

namespace cnoid {

class STLSceneLoaderImpl;

class CNOID_EXPORT STLSceneLoader : public AbstractSceneLoader
{
public:
    STLSceneLoader();
    ~STLSceneLoader();
    virtual void setMessageSink(std::ostream& os);
    virtual SgNode* load(const std::string& filename) override;

private:
    STLSceneLoaderImpl* impl;
};

}

#endif
