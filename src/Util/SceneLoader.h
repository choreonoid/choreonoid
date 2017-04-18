/**
 @author Shin'ichiro Nakaoka
*/
#ifndef CNOID_UTIL_SCENE_LOADER_H
#define CNOID_UTIL_SCENE_LOADER_H

#include "AbstractSceneLoader.h"
#include <memory>
#include "exportdecl.h"

namespace cnoid {

class SceneLoaderImpl;

class CNOID_EXPORT SceneLoader : public AbstractSceneLoader
{
public:
    static void registerLoader(const char* extensions, std::function<std::shared_ptr<AbstractSceneLoader>()> factory);

    //! This function returns a SceneLoader instance. The instance can only be used in the main thread.
    static SceneLoader* instance();
    
    SceneLoader();
    virtual ~SceneLoader();
    virtual SgNodePtr load(const std::string& filename) override;

private:
    SceneLoaderImpl* impl;
};

};

#endif
