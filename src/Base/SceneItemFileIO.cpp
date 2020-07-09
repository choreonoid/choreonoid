#include "SceneItemFileIO.h"
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
class SceneItemFileIO::Impl
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
    SgNode* loadScene(SceneItemFileIO* self, const std::string& filename);
    void createOptionPanel();    
};

SceneItemFileIO::Impl* SceneItemFileIO::Impl::sharedImpl = nullptr;
int SceneItemFileIO::Impl::sharedImplCounter = 0;

}


SceneItemFileIO::SceneItemFileIO(int api)
    : ItemFileIO("GENERAL-3D-MODEL", api)
{
    setExtensions({ "scen", "wrl", "dae", "stl" });

    if(!Impl::sharedImpl){
        Impl::sharedImpl = new Impl;
    }
    impl = Impl::sharedImpl;
    ++Impl::sharedImplCounter;
}


SceneItemFileIO::SceneItemFileIO()
    : SceneItemFileIO(Load | Options | OptionPanelForLoading)
{

}


SceneItemFileIO::Impl::Impl()
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


SceneItemFileIO::~SceneItemFileIO()
{
    if(--Impl::sharedImplCounter == 0){
        delete Impl::sharedImpl;
        Impl::sharedImpl = nullptr;
    }
}


SceneItemFileIO::Impl::~Impl()
{
    if(optionPanel){
        delete optionPanel;
    }
}


SgNode* SceneItemFileIO::loadScene(const std::string& filename)
{
    return impl->loadScene(this, filename);
}


SgNode* SceneItemFileIO::Impl::loadScene(SceneItemFileIO* self, const std::string& filename)
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
 

void SceneItemFileIO::resetOptions()
{
    impl->lengthUnitHint.select(Meter);
    impl->upperAxisHint.select(Z_Upper);
}


void SceneItemFileIO::storeOptions(Mapping* archive)
{
    if(!impl->lengthUnitHint.is(Meter)){
        archive->write("meshLengthUnitHint", impl->lengthUnitHint.selectedSymbol());
    }
    if(!impl->upperAxisHint.is(Z_Upper)){
        archive->write("meshUpperAxisHint", impl->upperAxisHint.selectedSymbol());
    }

}


bool SceneItemFileIO::restoreOptions(const Mapping* archive)
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


QWidget* SceneItemFileIO::getOptionPanelForLoading()
{
    if(!impl->optionPanel){
        impl->createOptionPanel();
    }
    return impl->optionPanel;
}


void SceneItemFileIO::Impl::createOptionPanel()
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


void SceneItemFileIO::fetchOptionPanelForLoading()
{
    impl->lengthUnitHint.select(impl->unitCombo->currentIndex());
    impl->upperAxisHint.select(impl->axisCombo->currentIndex());
}
