#ifndef CNOID_UTIL_SCENE_LOADER_H
#define CNOID_UTIL_SCENE_LOADER_H

#include "AbstractSceneLoader.h"
#include "Signal.h"
#include <memory>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT SceneLoader : public AbstractSceneLoader
{
public:
    //! \note The registration must be done in the main thread.
    static void registerLoader(
        const std::vector<std::string>& extensions,
        std::function<std::shared_ptr<AbstractSceneLoader>()> factory);
    
    //! \note The registration must be done in the main thread.
    static void registerLoader(const char* extension, std::function<std::shared_ptr<AbstractSceneLoader>()> factory);

    static std::vector<std::string> availableFileExtensions();
    static SignalProxy<void(const std::vector<std::string>& extensions)> sigAvailableFileExtensionsAdded();

    SceneLoader();
    virtual ~SceneLoader();
    virtual void setMessageSink(std::ostream& os) override;
    virtual void setDefaultDivisionNumber(int n) override;
    virtual void setDefaultCreaseAngle(double theta) override;
    virtual SgNode* load(const std::string& filename) override;

    SgNode* load(const std::string& filename, bool& out_isSupportedFormat);

    std::shared_ptr<AbstractSceneLoader> actualSceneLoaderOnLastLoading();

    /**
       Registers an image search directory and propagates it to every concrete loader this
       SceneLoader manages. The directory is remembered, so loaders created lazily by later
       calls to load() will also receive it. This is the right place to express "textures
       may also live in this directory" hints that are independent of the mesh file format
       (e.g. the SDF convention of placing textures under "<model>/materials/textures/").
    */
    virtual void addImageSearchDirectory(const std::string& directory) override;

    virtual void clearImageSearchDirectories() override;

private:
    class Impl;
    Impl* impl;
};

}

#endif
