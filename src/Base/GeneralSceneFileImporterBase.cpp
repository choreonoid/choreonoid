#include "GeneralSceneFileImporterBase.h"
#include <cnoid/SceneLoader>
#include <cnoid/Selection>
#include <cnoid/ValueTree>
#include <cnoid/ComboBox>
#include <cnoid/UTF8>
#include <cnoid/stdx/filesystem>
#include <QBoxLayout>
#include <QLabel>
#include <fmt/format.h>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

namespace cnoid {

//! \todo Share the instance of the following impl class
class GeneralSceneFileImporterBase::Impl
{
public:
    static Impl* sharedImpl;
    static int sharedImplCounter;
    
    unique_ptr<SceneLoader> sceneLoader;
    QWidget* optionPanel;

    Selection lengthUnitHint;
    ComboBox* unitCombo;

    Selection upperAxisHint;
    ComboBox* axisCombo;
    
    Impl();
    ~Impl();
    SgNode* loadScene(GeneralSceneFileImporterBase* self, const std::string& filename);
    void createOptionPanel();    
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


GeneralSceneFileImporterBase::Impl::Impl()
    : lengthUnitHint(SceneLoader::NumLengthUnitTypes),
      upperAxisHint(SceneLoader::NumUpperAxisTypes, CNOID_GETTEXT_DOMAIN_NAME)
{
    lengthUnitHint.setSymbol(SceneLoader::Meter, "meter");
    lengthUnitHint.setSymbol(SceneLoader::Millimeter, "millimeter");
    lengthUnitHint.setSymbol(SceneLoader::Inch, "inch");

    upperAxisHint.setSymbol(SceneLoader::Z_Upper, "Z");
    upperAxisHint.setSymbol(SceneLoader::Y_Upper, "Y");

    optionPanel = nullptr;
    unitCombo = nullptr;
    axisCombo = nullptr;
}


GeneralSceneFileImporterBase::~GeneralSceneFileImporterBase()
{
    if(--Impl::sharedImplCounter == 0){
        delete Impl::sharedImpl;
        Impl::sharedImpl = nullptr;
    }
}


GeneralSceneFileImporterBase::Impl::~Impl()
{
    if(optionPanel){
        delete optionPanel;
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

    sceneLoader->setLengthUnitHint(static_cast<SceneLoader::LengthUnitType>(lengthUnitHint.which()));
    sceneLoader->setUpperAxisHint(static_cast<SceneLoader::UpperAxisType>(upperAxisHint.which()));
    
    bool isSupported;
    SgNode* scene = sceneLoader->load(filename, isSupported);

    if(!scene){
        if(!isSupported){
            auto fname = toUTF8(stdx::filesystem::path(fromUTF8(filename)).filename().string());
            self->putError(format(_("The file format of \"{}\" is not supported.\n"), fname));
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
    impl->lengthUnitHint.select(SceneLoader::Meter);
    impl->upperAxisHint.select(SceneLoader::Z_Upper);
}


void GeneralSceneFileImporterBase::storeOptions(Mapping* archive)
{
    if(!impl->lengthUnitHint.is(SceneLoader::Meter)){
        archive->write("meshLengthUnitHint", impl->lengthUnitHint.selectedSymbol());
    }
    if(!impl->upperAxisHint.is(SceneLoader::Z_Upper)){
        archive->write("meshUpperAxisHint", impl->upperAxisHint.selectedSymbol());
    }
}


bool GeneralSceneFileImporterBase::restoreOptions(const Mapping* archive)
{
    string value;
    if(archive->read("meshLengthUnitHint", value)){
        impl->lengthUnitHint.select(value);
    }
    if(archive->read("meshUpperAxisHint", value)){
        impl->upperAxisHint.select(value);
    }
    return true;
}


void GeneralSceneFileImporterBase::setCurrentLengthUnitHint(LengthUnitType unitType)
{
    switch(unitType){
    case Meter:      impl->lengthUnitHint.select(SceneLoader::Meter);      break;
    case Millimeter: impl->lengthUnitHint.select(SceneLoader::Millimeter); break;
    case Inch:       impl->lengthUnitHint.select(SceneLoader::Inch);       break;
    default: break;
    }
}


QWidget* GeneralSceneFileImporterBase::getOptionPanelForLoading()
{
    if(!impl->optionPanel){
        impl->createOptionPanel();
    }

    impl->unitCombo->blockSignals(true);
    impl->unitCombo->setCurrentIndex(impl->lengthUnitHint.which());
    impl->unitCombo->blockSignals(false);
    
    impl->axisCombo->blockSignals(true);
    impl->axisCombo->setCurrentIndex(impl->upperAxisHint.which());
    impl->axisCombo->blockSignals(false);
    
    return impl->optionPanel;
}


void GeneralSceneFileImporterBase::Impl::createOptionPanel()
{
    optionPanel = new QWidget;
    auto hbox = new QHBoxLayout;
    hbox->setContentsMargins(0, 0, 0, 0);
    optionPanel->setLayout(hbox);

    hbox->addWidget(new QLabel(_("[ Mesh import hints ]")));

    hbox->addWidget(new QLabel(_("Unit:")));
    unitCombo = new ComboBox;
    unitCombo->addItem(_("Meter"));
    unitCombo->addItem(_("Millimeter"));
    unitCombo->addItem(_("Inch"));
    unitCombo->sigCurrentIndexChanged().connect(
        [this](int index){ lengthUnitHint.select(index); });
    hbox->addWidget(unitCombo);
    
    hbox->addWidget(new QLabel(_("Upper axis:")));
    axisCombo = new ComboBox;
    for(int i=0; i < SceneLoader::NumUpperAxisTypes; ++i){
        axisCombo->addItem(upperAxisHint.label(i));
    }
    axisCombo->sigCurrentIndexChanged().connect(
        [this](int index){ upperAxisHint.select(index); });
    hbox->addWidget(axisCombo);
}


void GeneralSceneFileImporterBase::fetchOptionPanelForLoading()
{
    impl->lengthUnitHint.select(impl->unitCombo->currentIndex());
    impl->upperAxisHint.select(impl->axisCombo->currentIndex());
}
