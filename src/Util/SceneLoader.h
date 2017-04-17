/**
 @author Shin'ichiro Nakaoka
*/
#ifndef CNOID_UTIL_SCENE_LOADER_H
#define CNOID_UTIL_SCENE_LOADER_H

#include "AbstractSceneLoader.h"

namespace cnoid {

class SceneLoaderImpl;

class SceneLoader : public AbstractSceneLoader
{
public:
    static void registerLoader(const std::string& extension, std::function<AbstractSceneLoaderPtr()> factory);
    
    SceneLoader();
    virtual ~SceneLoader();
    virtual SgNode* load(const std::string& filename);

private:
    SceneLoaderImpl* impl;
};

};

#endif
