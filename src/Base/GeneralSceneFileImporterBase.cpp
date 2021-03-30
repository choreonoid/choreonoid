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

namespace {

enum LengthUnit { Meter, Millimeter, Inch, NumLengthUnitIds };
enum UpperAxis { Z_Upper, Y_Upper, NumUpperAxisIds };

}

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
    setExtensionFunction(SceneLoader::availableFileExtensions);

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
    : lengthUnitHint(NumLengthUnitIds),
      upperAxisHint(NumUpperAxisIds, CNOID_GETTEXT_DOMAIN_NAME)
{
    lengthUnitHint.setSymbol(Meter, "meter");
    lengthUnitHint.setSymbol(Millimeter, "millimeter");
    lengthUnitHint.setSymbol(Inch, "inch");

    upperAxisHint.setSymbol(Z_Upper, "Z");
    upperAxisHint.setSymbol(Y_Upper, "Y");

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

    bool isSupported;
    SgNode* scene = sceneLoader->load(filename, isSupported);

    if(!scene){
        if(!isSupported){
            auto fname = toUTF8(stdx::filesystem::path(fromUTF8(filename)).filename().string());
            self->putError(format(_("The file format of \"{}\" is not supported.\n"), fname));
        }
        return nullptr;
    }

    SgNodePtr topNode = scene;
    /**
       \note Modifying the vertex positions might be better than
       inserting the transform nodes.
       \note This should be implemented in SceneLoader.
    */
    if(!lengthUnitHint.is(Meter)){
        auto scale = new SgScaleTransform;
        if(lengthUnitHint.is(Millimeter)){
            scale->setScale(1.0 / 1000.0);
        } else if(lengthUnitHint.is(Inch)){
            scale->setScale(0.0254);
        }
        scale->addChild(topNode);
        topNode = scale;
    }
    if(upperAxisHint.is(Y_Upper)){
        auto transform = new SgPosTransform;
        Matrix3 R;
        R << 0, 0, 1,
             1, 0, 0,
             0, 1, 0;
        transform->setRotation(R);
        transform->addChild(topNode);
        topNode = transform;
    }
    
    return topNode.retn();
}


bool GeneralSceneFileImporterBase::saveScene(SgNode* scene, const std::string& filename)
{
    return false;
}


void GeneralSceneFileImporterBase::resetOptions()
{
    impl->lengthUnitHint.select(Meter);
    impl->upperAxisHint.select(Z_Upper);
}


void GeneralSceneFileImporterBase::storeOptions(Mapping* archive)
{
    if(!impl->lengthUnitHint.is(Meter)){
        archive->write("meshLengthUnitHint", impl->lengthUnitHint.selectedSymbol());
    }
    if(!impl->upperAxisHint.is(Z_Upper)){
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


QWidget* GeneralSceneFileImporterBase::getOptionPanelForLoading()
{
    if(!impl->optionPanel){
        impl->createOptionPanel();
    }
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
    hbox->addWidget(unitCombo);
    hbox->addWidget(new QLabel(_("Upper axis:")));
    axisCombo = new ComboBox;
    for(int i=0; i < NumUpperAxisIds; ++i){
        axisCombo->addItem(upperAxisHint.label(i));
    }
    hbox->addWidget(axisCombo);
}


void GeneralSceneFileImporterBase::fetchOptionPanelForLoading()
{
    impl->lengthUnitHint.select(impl->unitCombo->currentIndex());
    impl->upperAxisHint.select(impl->axisCombo->currentIndex());
}
