#ifndef CNOID_BASE_SCENE_ITEM_FILE_IO_H
#define CNOID_BASE_SCENE_ITEM_FILE_IO_H

#include "SceneItem.h"
#include <cnoid/ItemFileIO>

namespace cnoid {

class StdSceneWriter;

class CNOID_EXPORT SceneItemStdSceneFileExporter : public ItemFileIoBase<SceneItem>
{
public:
    SceneItemStdSceneFileExporter();
    
protected:
    virtual bool save(SceneItem* item, const std::string& filename) override;
    
private:
    StdSceneWriter* ensureSceneWriter();
    
    std::unique_ptr<StdSceneWriter> sceneWriter_;
};

}

#endif
