/*!
  @file
  @author Shizuko Hattori, Shin'ichiro Nakaoka
*/

#ifndef CNOID_ASSIMP_SCENE_LOADER_H
#define CNOID_ASSIMP_SCENE_LOADER_H

#include <cnoid/AbstractSceneLoader>
#include "exportdecl.h"

namespace cnoid {

class AssimpSceneLoaderImpl;

class CNOID_EXPORT AssimpSceneLoader : public AbstractSceneLoader
{
public:
    static void initializeClass();
    
    AssimpSceneLoader();
    ~AssimpSceneLoader();
    void setMessageSink(std::ostream& os) override;
    virtual SgNode* load(const std::string& filename) override;

private:
    AssimpSceneLoaderImpl* impl;
    AssimpSceneLoaderImpl* getOrCreateImpl();
};

};

#endif
