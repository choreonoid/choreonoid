#ifndef CNOID_BASE_ITEM_GENERAL_SCENE_FILE_IMPORTER_BASE_H
#define CNOID_BASE_ITEM_GENERAL_SCENE_FILE_IMPORTER_BASE_H

#include "ItemFileIO.h"
#include "exportdecl.h"

namespace cnoid {

class SgNode;

class CNOID_EXPORT GeneralSceneFileImporterBase : public ItemFileIO
{
public:
    GeneralSceneFileImporterBase();
    GeneralSceneFileImporterBase(int api);
    ~GeneralSceneFileImporterBase();

protected:
    SgNode* loadScene(const std::string& filename);

    //! This function has not been implemented yet
    bool saveScene(SgNode* scene, const std::string& filename);
    
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
