/**
 @author Shin'ichiro Nakaoka
*/

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

private:
    class Impl;
    Impl* impl;
};

}

#endif
