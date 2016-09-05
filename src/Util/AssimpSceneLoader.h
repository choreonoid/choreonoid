#ifndef CNOID_UTIL_ASSIMP_SCENE_LOADER_H
#define CNOID_UTIL_ASSIMP_SCENE_LOADER_H

#include "AbstractSceneLoader.h"
#include "exportdecl.h"

namespace cnoid {

class AssimpSceneLoaderImpl;

class CNOID_EXPORT AssimpSceneLoader : public AbstractSceneLoader
{
public:
    AssimpSceneLoader();
    ~AssimpSceneLoader();
    virtual const char* format() const;
    virtual SgNode* load(const std::string& fileName);

private:
    AssimpSceneLoaderImpl* impl;
};

};

#endif
