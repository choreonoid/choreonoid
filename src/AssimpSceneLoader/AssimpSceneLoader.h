#ifndef CNOID_ASSIMP_SCENE_LOADER_H
#define CNOID_ASSIMP_SCENE_LOADER_H

#include <cnoid/AbstractSceneLoader>
#include <string>
#include "exportdecl.h"

namespace cnoid {

/**
   Loads scene files (Collada, STL, OBJ, etc.) via Assimp.

   Thread safety: an instance of this class is not thread-safe. A single instance must be
   used by only one thread at a time, and all mutating calls (load, addImageSearchDirectory,
   clearImageSearchDirectories, setMessageSink) must be made from that same thread. Different
   instances on different threads do not interact and are safe to use concurrently. When the
   loader is obtained indirectly through cnoid::SceneLoader, that container already gives each
   SceneLoader instance its own AssimpSceneLoader, so this contract holds naturally.
*/
class CNOID_EXPORT AssimpSceneLoader : public AbstractSceneLoader
{
public:
    static void initializeClass();

    AssimpSceneLoader();
    ~AssimpSceneLoader();
    void setMessageSink(std::ostream& os) override;
    virtual SgNode* load(const std::string& filename) override;

    /**
       Adds a directory that is searched when a texture image referenced by a loaded scene
       file cannot be found next to the scene file itself. Useful for formats whose
       distribution convention places textures in a directory separate from the meshes
       (for example, Gazebo SDF models keep meshes under "<model>/meshes/" and textures
       under "<model>/materials/textures/"). The search is tried for relative texture paths
       only; absolute paths are honored as written. Multiple directories may be added and
       are tried in the order they were registered.

       The added directories persist across calls to load() on the same instance. Call
       clearImageSearchDirectories() to discard them when their context (e.g. one model file
       load) is finished.
    */
    void addImageSearchDirectory(const std::string& directory) override;

    /**
       Removes all directories previously added with addImageSearchDirectory(). After this
       call, texture image lookup falls back to the directory of the scene file alone, just
       as it does on a freshly constructed loader.
    */
    void clearImageSearchDirectories() override;

private:
    class Impl;
    Impl* impl;
    Impl* getOrCreateImpl();
};

};

#endif
