#ifndef CNOID_ASSIMP_SCENE_LOADER_H
#define CNOID_ASSIMP_SCENE_LOADER_H

#include <cnoid/AbstractSceneLoader>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT AssimpSceneLoader : public AbstractSceneLoader
{
public:
    static void initializeClass();
    
    AssimpSceneLoader();
    ~AssimpSceneLoader();
    void setMessageSink(std::ostream& os) override;
    virtual SgNode* load(const std::string& filename) override;

    void setMessageSinkStdErr();
private:
    class Impl;
    Impl* impl;
    Impl* getOrCreateImpl();
};

};

#endif
