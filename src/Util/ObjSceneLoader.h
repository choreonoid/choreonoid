#ifndef CNOID_UTIL_OBJ_SCENE_LOADER_H
#define CNOID_UTIL_OBJ_SCENE_LOADER_H

#include "AbstractSceneLoader.h"
#include <string>
#include "exportdecl.h"

namespace cnoid {

/**
   Loads Wavefront OBJ (.obj) scene files.

   Thread safety: a single instance must be used by only one thread at a time. Different
   instances on different threads do not interact and are safe to use concurrently.
*/
class CNOID_EXPORT ObjSceneLoader : public AbstractSceneLoader
{
public:
    ObjSceneLoader();
    ~ObjSceneLoader();
    virtual void setMessageSink(std::ostream& os) override;
    virtual SgNode* load(const std::string& filename) override;

    /**
       Adds a directory that is searched when a texture image referenced from the OBJ's
       companion MTL file cannot be found next to the OBJ file itself. See
       AssimpSceneLoader::addImageSearchDirectory for the lookup rules and motivation.
    */
    void addImageSearchDirectory(const std::string& directory) override;

    void clearImageSearchDirectories() override;

private:
    class Impl;
    Impl* impl;
};

}

#endif
