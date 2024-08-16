#include "GeneralSceneFileImporterBase.h"
#include "GeneralSceneFileLoadDialog.h"
#include <cnoid/SceneLoader>
#include <cnoid/UTF8>
#include <cnoid/Format>
#include <cnoid/stdx/filesystem>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace cnoid {

//! \todo Share the instance of the following impl class
class GeneralSceneFileImporterBase::Impl
{
public:
    static Impl* sharedImpl;
    static int sharedImplCounter;
    
    unique_ptr<SceneLoader> sceneLoader;
    GeneralSceneFileLoadDialog::OptionSet optionSet;

    SgNode* loadScene(GeneralSceneFileImporterBase* self, const std::string& filename);
};

GeneralSceneFileImporterBase::Impl* GeneralSceneFileImporterBase::Impl::sharedImpl = nullptr;
int GeneralSceneFileImporterBase::Impl::sharedImplCounter = 0;

}


GeneralSceneFileImporterBase::GeneralSceneFileImporterBase(int api)
    : ItemFileIO("GENERAL-3D-MODEL", api)
{
    setExtensionsForLoading(SceneLoader::availableFileExtensions());

    SceneLoader::sigAvailableFileExtensionsAdded().connect(
        [this](const std::vector<std::string>& extensions){
            addExtensionsForLoading(extensions);
        });

    if(!Impl::sharedImpl){
        Impl::sharedImpl = new Impl;
    }
    impl = Impl::sharedImpl;
    ++Impl::sharedImplCounter;
}


GeneralSceneFileImporterBase::GeneralSceneFileImporterBase()
    : GeneralSceneFileImporterBase(Load | Options | OptionPanelForLoading)
{

}


GeneralSceneFileImporterBase::~GeneralSceneFileImporterBase()
{
    if(--Impl::sharedImplCounter == 0){
        delete Impl::sharedImpl;
        Impl::sharedImpl = nullptr;
    }
}


SgNode* GeneralSceneFileImporterBase::loadScene(const std::string& filename)
{
    return impl->loadScene(this, filename);
}


SgNode* GeneralSceneFileImporterBase::Impl::loadScene(GeneralSceneFileImporterBase* self, const std::string& filename)
{
    if(!sceneLoader){
        sceneLoader.reset(new SceneLoader);
        sceneLoader->setMessageSink(self->os());
    }

    sceneLoader->setLengthUnitHint(optionSet.lengthUnitHint());
    sceneLoader->setUpperAxisHint(optionSet.upperAxisHint());
    
    bool isSupported;
    SgNode* scene = sceneLoader->load(filename, isSupported);

    if(!scene){
        if(!isSupported){
            auto fname = toUTF8(stdx::filesystem::path(fromUTF8(filename)).filename().string());
            self->putError(formatR(_("The file format of \"{}\" is not supported.\n"), fname));
        }
        return nullptr;
    }

    return scene;
}


std::shared_ptr<AbstractSceneLoader> GeneralSceneFileImporterBase::sceneLoaderOnLastLoading()
{
    return impl->sceneLoader->actualSceneLoaderOnLastLoading();
}


bool GeneralSceneFileImporterBase::saveScene(SgNode* /* scene */, const std::string& /* filename */)
{
    return false;
}


void GeneralSceneFileImporterBase::resetOptions()
{
    impl->optionSet.resetOptions();
}


void GeneralSceneFileImporterBase::storeOptions(Mapping* archive)
{
    impl->optionSet.storeOptions(archive);
}


bool GeneralSceneFileImporterBase::restoreOptions(const Mapping* archive)
{
    return impl->optionSet.restoreOptions(archive);
}


void GeneralSceneFileImporterBase::setCurrentLengthUnitHint(LengthUnitType unitType)
{
    switch(unitType){
    case Meter:      impl->optionSet.setLengthUnitHint(SceneLoader::Meter);      break;
    case Millimeter: impl->optionSet.setLengthUnitHint(SceneLoader::Millimeter); break;
    case Inch:       impl->optionSet.setLengthUnitHint(SceneLoader::Inch);       break;
    default: break;
    }
}


QWidget* GeneralSceneFileImporterBase::getOptionPanelForLoading()
{
    return impl->optionSet.panel();
}
