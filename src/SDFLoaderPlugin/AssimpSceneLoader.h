#ifndef CNOID_UTIL_ASSIMP_SCENE_LOADER_H
#define CNOID_UTIL_ASSIMP_SCENE_LOADER_H

#include <cnoid/AbstractSceneLoader>
#include "exportdecl.h"

namespace cnoid {

class AssimpSceneLoaderImpl;

class AssimpSceneLoader : public AbstractSceneLoader
{
public:
    AssimpSceneLoader();
    ~AssimpSceneLoader();
    void setMessageSink(std::ostream& os);
    virtual const char* format() const;
    virtual SgNode* load(const std::string& fileName);

private:
    AssimpSceneLoaderImpl* impl;
};

};

#endif
