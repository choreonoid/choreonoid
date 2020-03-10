#ifndef CNOID_BASE_SCENE_ITEM_FILE_IO_H
#define CNOID_BASE_SCENE_ITEM_FILE_IO_H

#include "ItemFileIO.h"
#include "exportdecl.h"

namespace cnoid {

class SgNode;

class CNOID_EXPORT SceneItemFileIO : public ItemFileIO
{
public:
    SceneItemFileIO();
    SceneItemFileIO(int api);
    ~SceneItemFileIO();

protected:
    SgNode* loadScene(const std::string& filename);
    
    virtual void resetOptions() override;
    virtual void storeOptions(Mapping* archive) override;
    virtual bool restoreOptions(const Mapping* archive) override;
    virtual QWidget* getOptionPanelForLoading() override;
    virtual void fetchOptionPanelForLoading() override;

private:
    class Impl;
    Impl* impl;
};

}

#endif
