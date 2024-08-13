#ifndef CNOID_BASE_GENERAL_SCENE_FILE_LOAD_DIALOG_H
#define CNOID_BASE_GENERAL_SCENE_FILE_LOAD_DIALOG_H

#include "FileDialog.h"
#include <cnoid/SceneLoader>
#include <string>
#include <memory>
#include "exportdecl.h"

namespace cnoid {

class Mapping;

class CNOID_EXPORT GeneralSceneFileLoadDialog : public FileDialog
{
public:
    // For the application customization
    static void setDefaultLengthUnitHint(SceneLoader::LengthUnitType hint);
    
    GeneralSceneFileLoadDialog();
    GeneralSceneFileLoadDialog(QWidget* parent, Qt::WindowFlags f = Qt::WindowFlags());
    void setFileTypeCaption(const std::string& caption);
    virtual int exec() override;
    SgNode* loadedScene() const { return loadedScene_; }
    void clearLoadedScene();

    class OptionSet
    {
    public:
        OptionSet();
        virtual ~OptionSet();
        void resetOptions();

        void setLengthUnitHint(SceneLoader::LengthUnitType hint);
        SceneLoader::LengthUnitType lengthUnitHint() const { return lengthUnitHint_; }

        void setUpperAxisHint(SceneLoader::UpperAxisType hint);
        SceneLoader::UpperAxisType upperAxisHint() const { return upperAxisHint_; }
        
        void storeOptions(Mapping* archive);
        bool restoreOptions(const Mapping* archive);

        QWidget* panel();

    private:
        SceneLoader::LengthUnitType lengthUnitHint_;
        SceneLoader::UpperAxisType upperAxisHint_;

        class Panel;
        Panel* panel_;
    };

private:
    std::string fileTypeCaption_;
    SgNodePtr loadedScene_;
    std::unique_ptr<SceneLoader> sceneLoader;
    OptionSet optionSet;

    void initialize();
};

}

#endif
