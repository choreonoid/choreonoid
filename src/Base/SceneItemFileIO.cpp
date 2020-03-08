#include "SceneItemFileIO.h"
#include <cnoid/SceneLoader>
#include <cnoid/Selection>
#include <cnoid/ValueTree>
#include <cnoid/FileUtil>
#include <cnoid/ComboBox>
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

unique_ptr<SceneLoader> sceneLoader;

}

namespace cnoid {

//! \todo Share the instance of the following impl class
class SceneItemFileIO::Impl
{
public:
    SceneItemFileIO* self;
    QWidget* optionPanel;

    Selection lengthUnitHint;
    ComboBox* unitCombo;

    Selection upperAxisHint;
    ComboBox* axisCombo;
    
    Impl(SceneItemFileIO* self);
    ~Impl();
    SgNode* loadScene(const std::string& filename);
    void createOptionPanel();    
};

}


SceneItemFileIO::SceneItemFileIO()
    : SceneItemFileIO("GENERAL-3D-MODEL", Load | Options | OptionPanelForLoading)
{

}


SceneItemFileIO::SceneItemFileIO(const std::string& formatId, int api)
    : ItemFileIO(formatId, api)
{
    setExtensions({ "scen", "wrl", "dae", "stl" });

    impl = new Impl(this);
}


SceneItemFileIO::Impl::Impl(SceneItemFileIO* self)
    : self(self),
      lengthUnitHint(NumLengthUnitIds),
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
    delete impl;
}


SceneItemFileIO::Impl::~Impl()
{
    if(optionPanel){
        delete optionPanel;
    }
}


SgNode* SceneItemFileIO::loadScene(const std::string& filename)
{
    return impl->loadScene(filename);
}


SgNode* SceneItemFileIO::Impl::loadScene(const std::string& filename)
{
    if(!sceneLoader){
        sceneLoader.reset(new SceneLoader);
        sceneLoader->setMessageSink(self->os());
    }

    bool isSupported;
    SgNode* scene = sceneLoader->load(filename, isSupported);

    if(!scene){
        if(!isSupported){
            self->putError(format(_("The file format of \"{}\" is not supported.\n"),
                            stdx::filesystem::path(filename).filename().string()));
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
