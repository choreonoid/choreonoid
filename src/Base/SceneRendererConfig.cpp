#include "SceneRendererConfig.h"
#include "MainWindow.h"
#include "Dialog.h"
#include "Buttons.h"
#include "ButtonGroup.h"
#include "CheckBox.h"
#include "SpinBox.h"
#include "DoubleSpinBox.h"
#include <cnoid/GLSceneRenderer>
#include <cnoid/SceneLights>
#include <cnoid/Selection>
#include <cnoid/ValueTree>
#include <cnoid/EigenArchive>
#include <QBoxLayout>
#include <QLabel>
#include <QColorDialog>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

constexpr int MaxNumShadows = 2;

enum ConfigCategory
{
    Lighting = 1 << 1,
    Shadow = 1 << 2,
    Drawing = 1 << 3,
    Effect = 1 << 4,
    AllCategories = Lighting | Shadow | Drawing | Effect
};

class ConfigWidgetSet
{
public:
    SceneRendererConfig::Impl* config;

    // Used for deleting all the widgets when the ConfigWidgetSet instance is deleted.
    QWidget* ownerWidget;

    QWidget* lightingPanel;
    
    ButtonGroup* lightingModeGroup;
    RadioButton* lightingModeRadios[GLSceneRenderer::NumLightingModes];
    ButtonGroup* cullingModeGroup;
    RadioButton* cullingModeRadios[GLSceneRenderer::NumCullingModes];
    CheckBox* smoothShadingCheck;
    CheckBox* headLightCheck;
    DoubleSpinBox* headLightIntensitySpin;
    CheckBox* worldLightCheck;
    DoubleSpinBox* worldLightIntensitySpin;
    CheckBox* ambientLightCheck;
    DoubleSpinBox* ambientIntensitySpin;
    CheckBox* worldLightShadowCheck;
    CheckBox* additionalLightSetCheck;
    CheckBox* textureCheck;
    CheckBox* fogCheck;
    struct ShadowWidgetSet {
        CheckBox* check;
        SpinBox* lightSpin;
    };
    ShadowWidgetSet shadowWidgetSets[MaxNumShadows];
    CheckBox* shadowAntiAliasingCheck;
    PushButton* backgroundColorButton;
    PushButton* defaultColorButton;
    DoubleSpinBox* pointSizeSpin;
    DoubleSpinBox* lineWidthSpin;
    CheckBox* upsideDownCheck;

    vector<QObject*> signalObjects;

    ConfigWidgetSet(SceneRendererConfig::Impl* config);
    ~ConfigWidgetSet();
    QWidget* createLightingPanel();
    void updateWidgets();
    void updateShadowWidgets();
};

class ConfigDialog : public Dialog
{
public:
    SceneRendererConfig::Impl* config;
    
    ConfigDialog(SceneRendererConfig::Impl* config);
};

}

namespace cnoid {

class SceneRendererConfig::Impl
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    SceneRendererConfig* self;
    vector<GLSceneRenderer*> renderers;
    
    Selection lightingMode;
    Selection cullingMode;
    
    bool isSmoothShadingEnabled;
    bool isHeadLightEnabled;
    bool isWorldLightEnabled;
    bool isAmbientLightEnabled;
    bool isAdditionalLightSetEnabled;
    bool isShadowCastingAvailable;
    bool isShadowAntiAliasingEnabled;
    bool isWorldLightShadowEnabled;
    bool isTextureEnabled;
    bool isFogEnabled;
    bool isUpsideDownEnabled;
    
    double headLightIntensity;
    double worldLightIntensity;
    double ambientIntensity;
    
    struct ShadowInfo {
        int lightIndex;
        bool isEnabled;
    };
    ShadowInfo shadowInfos[MaxNumShadows];
    
    Vector3f backgroundColor;
    Vector3f defaultColor;
    double pointSize;
    double lineWidth;

    ConfigWidgetSet* widgetSet;
    ConfigDialog* dialog;
    
    void doCommonInitialization();    
    Impl(SceneRendererConfig* self);
    Impl(const Impl& org, SceneRendererConfig* self);
    ~Impl();
    void addRenderer(SceneRenderer* renderer, bool doUpdateRenderer);
    void updateRenderer(GLSceneRenderer* renderer, unsigned int categories);
    void updateRenderers(unsigned int categories, bool isInteractive);
    bool store(Mapping* archive);
    bool restore(const Mapping* archive);
    ConfigWidgetSet* getOrCreateConfigWidgetSet();
};

}


SceneRendererConfig::SceneRendererConfig()
{
    impl = new Impl(this);
}


void SceneRendererConfig::Impl::doCommonInitialization()
{
    lightingMode.setDomain(CNOID_GETTEXT_DOMAIN_NAME);
    lightingMode.resize(GLSceneRenderer::NumLightingModes);
    lightingMode.setSymbol(GLSceneRenderer::NormalLighting, "normal");
    lightingMode.setSymbol(GLSceneRenderer::MinimumLighting, "minimum");
    lightingMode.setSymbol(GLSceneRenderer::SolidColorLighting, "solid_color");
    
    cullingMode.setDomain(CNOID_GETTEXT_DOMAIN_NAME);
    cullingMode.resize(GLSceneRenderer::NumCullingModes);
    cullingMode.setSymbol(GLSceneRenderer::ENABLE_BACK_FACE_CULLING, "enabled");
    cullingMode.setSymbol(GLSceneRenderer::DISABLE_BACK_FACE_CULLING, "disabled");
    cullingMode.setSymbol(GLSceneRenderer::FORCE_BACK_FACE_CULLING, "forced");

    widgetSet = nullptr;
    dialog = nullptr;
}


SceneRendererConfig::Impl::Impl(SceneRendererConfig* self)
    : self(self)
{
    doCommonInitialization();
    
    lightingMode.select(GLSceneRenderer::NormalLighting);
    cullingMode.select(GLSceneRenderer::ENABLE_BACK_FACE_CULLING);
    
    isSmoothShadingEnabled = true;
    isHeadLightEnabled = true;
    isWorldLightEnabled = true;
    isAmbientLightEnabled = true;
    isAdditionalLightSetEnabled = true;
    isShadowCastingAvailable = false;
    isShadowAntiAliasingEnabled = true;
    isWorldLightShadowEnabled = true;
    isTextureEnabled = true;
    isFogEnabled = true;
    isUpsideDownEnabled = false;

    headLightIntensity = 0.5;
    worldLightIntensity = 0.5;
    ambientIntensity = 0.5;

    for(int i=0; i < MaxNumShadows; ++i){
        shadowInfos[i].lightIndex = i;
        shadowInfos[i].isEnabled = false;
    }
    
    backgroundColor << 0.1f, 0.1f, 0.3f; // Dark blue
    defaultColor << 1.0f, 1.0f, 1.0f; // White
    pointSize = 1.0;
    lineWidth = 1.0;
}


SceneRendererConfig::SceneRendererConfig(const SceneRendererConfig& org)
{
    impl = new Impl(*org.impl, this);
}


SceneRendererConfig::Impl::Impl(const Impl& org, SceneRendererConfig* self)
    : self(self)
{
    doCommonInitialization();

    lightingMode.select(org.lightingMode.which());
    cullingMode.select(org.cullingMode.which());
    
    isSmoothShadingEnabled = org.isSmoothShadingEnabled;
    isHeadLightEnabled = org.isHeadLightEnabled;
    isWorldLightEnabled = org.isWorldLightEnabled;
    isAmbientLightEnabled = org.isAmbientLightEnabled;
    isAdditionalLightSetEnabled = org.isAdditionalLightSetEnabled;
    isShadowCastingAvailable = org.isShadowCastingAvailable;
    isShadowAntiAliasingEnabled = org.isShadowAntiAliasingEnabled;
    isWorldLightShadowEnabled = org.isWorldLightShadowEnabled;
    isTextureEnabled = org.isTextureEnabled;
    isFogEnabled = org.isFogEnabled;
    isUpsideDownEnabled = org.isUpsideDownEnabled;

    headLightIntensity = org.headLightIntensity;
    worldLightIntensity = org.worldLightIntensity;
    ambientIntensity = org.ambientIntensity;

    for(int i=0; i < MaxNumShadows; ++i){
        shadowInfos[i] = org.shadowInfos[i];
    }
    
    backgroundColor << org.backgroundColor;
    defaultColor << org.defaultColor;
    pointSize = org.pointSize;
    lineWidth = org.lineWidth;
}    


SceneRendererConfig::~SceneRendererConfig()
{
    delete impl;
}


SceneRendererConfig::Impl::~Impl()
{
    if(widgetSet){
        delete widgetSet;
    }
    if(dialog){
        delete dialog;
    }
}


void SceneRendererConfig::addRenderer(SceneRenderer* renderer, bool doUpdateRenderer)
{
    impl->addRenderer(renderer, doUpdateRenderer);
}


void SceneRendererConfig::Impl::addRenderer(SceneRenderer* renderer, bool doUpdateRenderer)
{
    if(auto glRenderer = dynamic_cast<GLSceneRenderer*>(renderer)){
        renderers.push_back(glRenderer);
        if(doUpdateRenderer){
            updateRenderer(glRenderer, AllCategories);
        }
        if(!isShadowCastingAvailable){
            if(glRenderer->isShadowCastingAvailable()){
                isShadowCastingAvailable = true;
                if(widgetSet){
                    widgetSet->updateShadowWidgets();
                }
            }
        }
    }
}


void SceneRendererConfig::removeRenderer(SceneRenderer* renderer)
{
    auto& renderers = impl->renderers;
    auto it = std::find(renderers.begin(), renderers.end(), renderer);
    if(it != renderers.end()){
        renderers.erase(it);
    }
}


void SceneRendererConfig::clearRenderers()
{
    impl->renderers.clear();

    if(impl->isShadowCastingAvailable){
        impl->isShadowCastingAvailable = false;
        if(impl->widgetSet){
            impl->widgetSet->updateShadowWidgets();
        }
    }
}


void SceneRendererConfig::Impl::updateRenderer(GLSceneRenderer* renderer, unsigned int categories)
{
    if(categories & Lighting){
        renderer->setLightingMode((GLSceneRenderer::LightingMode)lightingMode.which());
        renderer->setDefaultSmoothShading(isSmoothShadingEnabled);
        renderer->setBackFaceCullingMode(cullingMode.which());

        auto headLight = renderer->headLight();
        headLight->on(isHeadLightEnabled);
        headLight->setIntensity(headLightIntensity);
        headLight->setAmbientIntensity(0.0);

        auto worldLight = renderer->worldLight();
        worldLight->on(isWorldLightEnabled || isAmbientLightEnabled);
        worldLight->setIntensity(isWorldLightEnabled ? worldLightIntensity : 0.0);
        worldLight->setAmbientIntensity(isAmbientLightEnabled ? ambientIntensity : 0.0);
        
        renderer->enableAdditionalLights(isAdditionalLightSetEnabled);
    }

    if(categories & Shadow){
        renderer->setWorldLightShadowEnabled(isWorldLightShadowEnabled);
        renderer->clearAdditionalLightShadows();
        for(int i=0; i < MaxNumShadows; ++i){
            auto& shadow = shadowInfos[i];
            if(shadow.isEnabled){
                renderer->setAdditionalLightShadowEnabled(shadow.lightIndex, true);
            }
        }
        renderer->setShadowAntiAliasingEnabled(isShadowAntiAliasingEnabled);
    }

    if(categories & Drawing){
        renderer->enableTexture(isTextureEnabled);
        renderer->setBackgroundColor(backgroundColor);
        renderer->setDefaultColor(defaultColor);
        renderer->setDefaultPointSize(pointSize);
        renderer->setDefaultLineWidth(lineWidth);
    }

    if(categories & Effect){
        renderer->enableFog(isFogEnabled);
        renderer->setUpsideDown(isUpsideDownEnabled);
    }
}


void SceneRendererConfig::Impl::updateRenderers(unsigned int categories, bool isInteractive)
{
    for(auto& renderer : renderers){
        updateRenderer(renderer, categories);
    }
    self->onRendererConfigUpdated(isInteractive);
}


void SceneRendererConfig::updateRenderers()
{
    impl->updateRenderers(AllCategories, false);
}


void SceneRendererConfig::onRendererConfigUpdated(bool /* isInteractive */)
{

}


bool SceneRendererConfig::store(Mapping* archive)
{
    return impl->store(archive);
}


bool SceneRendererConfig::Impl::store(Mapping* archive)
{
    archive->write("lighting_mode", lightingMode.selectedSymbol());
    archive->write("culling_mode", cullingMode.selectedSymbol());
    archive->write("shading_mode", isSmoothShadingEnabled ? "smooth" : "flat");

    archive->write("world_light", isWorldLightEnabled);
    archive->write("world_light_intensity", worldLightIntensity);
    archive->write("ambient_light", isAmbientLightEnabled);
    archive->write("ambient_light_intensity", ambientIntensity);
    archive->write("head_light", isHeadLightEnabled);
    archive->write("head_light_intensity", headLightIntensity);

    if(!isAdditionalLightSetEnabled){
        archive->write("enable_additional_lights", false);
    }

    archive->write("world_light_shadow", isWorldLightShadowEnabled);

    ListingPtr shadowLights = new Listing;
    for(int i=0; i < MaxNumShadows; ++i){
        auto& info = shadowInfos[i];
        MappingPtr node = new Mapping;
        node->setFlowStyle(true);
        node->write("index", info.lightIndex);
        node->write("enabled", info.isEnabled);
        shadowLights->append(node);
    }
    archive->insert("shadow_lights", shadowLights);

    if(!isShadowAntiAliasingEnabled){
        archive->write("shadow_antialiasing", false);
    }
    if(!isTextureEnabled){
        archive->write("texture", false);
    }
    if(!isFogEnabled){
        archive->write("fog", false);
    }
    write(archive, "background_color", backgroundColor);

    if(!defaultColor.isApprox(Vector3f::Ones())){
        write(archive, "default_color", defaultColor);
    }
    archive->write("line_width", lineWidth);
    archive->write("point_size", pointSize);

    if(isUpsideDownEnabled){
        archive->write("upside_down", true);
    }

    return true;
}
 

bool SceneRendererConfig::restore(const Mapping* archive)
{
    return impl->restore(archive);
}


bool SceneRendererConfig::Impl::restore(const Mapping* archive)
{
    string symbol;
    
    if(archive->read({ "lighting_mode", "lightingMode" }, symbol)){
        lightingMode.select(symbol);
    }
    if(archive->read({ "culling_mode", "cullingMode" }, symbol)){
        cullingMode.select(symbol);
    }
    if(archive->read("shading_mode", symbol)){
        isSmoothShadingEnabled = (symbol != "flat");
    }

    archive->read({ "world_light", "worldLight"}, isWorldLightEnabled);
    archive->read({ "world_light_intensity", "worldLightIntensity" }, worldLightIntensity);

    bool hasAmbientLightParameters = false;
    if(archive->read("ambient_light", isAmbientLightEnabled)){
        hasAmbientLightParameters = true;
    }
    if(archive->read("ambient_light_intensity", ambientIntensity)){
        hasAmbientLightParameters = true;
    }
    if(!hasAmbientLightParameters){
        if(archive->read("worldLightAmbient", ambientIntensity)){
            isAmbientLightEnabled = true;
        }
    }
    
    archive->read({ "head_light", "defaultHeadLight" }, isHeadLightEnabled);
    archive->read({ "head_light_intensity", "defaultHeadLightIntensity" }, headLightIntensity);

    if(!archive->read({ "enable_additional_lights", "additionalLights" }, isAdditionalLightSetEnabled)){
        isAdditionalLightSetEnabled = true;
    }

    for(int i=0; i < MaxNumShadows; ++i){
        shadowInfos[i].isEnabled = false;
    }
    bool isOldShadowIndexFormat = false;
    ListingPtr shadowLights = archive->findListing("shadow_lights");
    if(!shadowLights->isValid()){
        shadowLights = archive->findListing("shadowLights"); // Old format
        if(shadowLights->isValid()){
            isOldShadowIndexFormat = true;
            isWorldLightShadowEnabled = false;
        }
    }
    if(shadowLights->isValid()){
        int configIndex = 0;
        int n = std::min(shadowLights->size(), MaxNumShadows);
        for(int i=0; i < n; ++i){
            auto node = shadowLights->at(i);
            int lightIndex = 0;
            bool enabled = true;
            if(node->isScalar()){ // Old format
                lightIndex = node->toInt();
            } else if(node->isMapping()){
                auto info = node->toMapping();
                lightIndex = info->get("index", 0);
                enabled = info->get("enabled", false);
            }
            if(isOldShadowIndexFormat){
                if(lightIndex == 0){
                    isWorldLightShadowEnabled = true;
                    continue;
                }
                --lightIndex;
            }
            auto& shadow = shadowInfos[configIndex++];
            shadow.lightIndex = lightIndex;
            shadow.isEnabled = enabled;
        }
        for(int i = configIndex; i < MaxNumShadows; ++i){
            shadowInfos[configIndex].isEnabled = false;
        }
    }
    if(!isOldShadowIndexFormat){
        archive->read("world_light_shadow", isWorldLightShadowEnabled);
    }
    isShadowAntiAliasingEnabled = archive->get("shadow_antialiasing", true);

    isTextureEnabled = archive->get("texture", true);
    isFogEnabled = archive->get("fog", true);
    read(archive, { "background_color", "backgroundColor" }, backgroundColor);
    if(!read(archive, { "default_color", "defaultColor" }, defaultColor)){
        defaultColor << 1.0f, 1.0f, 1.0f;
    }
    archive->read({ "line_width", "lineWidth" }, lineWidth);
    archive->read({ "point_size", "pointSize" }, pointSize);
    isUpsideDownEnabled = archive->get({ "upside_down", "upsideDown" }, false);

    if(widgetSet){
        widgetSet->updateWidgets();
    }
    updateRenderers(AllCategories, false);

    return true;
}


void SceneRendererConfig::showConfigDialog()
{
    if(!impl->dialog){
        impl->dialog = new ConfigDialog(impl);
    }
    impl->dialog->show();
}


void SceneRendererConfig::createConfigWidgets()
{
    impl->getOrCreateConfigWidgetSet();
}


ConfigWidgetSet* SceneRendererConfig::Impl::getOrCreateConfigWidgetSet()
{
    if(!widgetSet){
        widgetSet = new ConfigWidgetSet(this);
    }
    return widgetSet;
}


QWidget* SceneRendererConfig::getOrCreateLightingPanel()
{
    auto ws = impl->getOrCreateConfigWidgetSet();
    if(!ws->lightingPanel){
        ws->createLightingPanel();
    }
    return ws->lightingPanel;
}


PushButton* SceneRendererConfig::backgroundColorButton()
{
    return impl->widgetSet->backgroundColorButton;
}


PushButton* SceneRendererConfig::defaultColorButton()
{
    return impl->widgetSet->defaultColorButton;
}


DoubleSpinBox* SceneRendererConfig::pointSizeSpin()
{
    return impl->widgetSet->pointSizeSpin;
}


DoubleSpinBox* SceneRendererConfig::lineWidthSpin()
{
    return impl->widgetSet->lineWidthSpin;
}


CheckBox* SceneRendererConfig::upsideDownCheck()
{
    return impl->widgetSet->upsideDownCheck;
}


void SceneRendererConfig::updateConfigWidgets()
{
    if(impl->widgetSet){
        impl->widgetSet->updateWidgets();
    }
}


bool SceneRendererConfig::inputColorWithColorDialog(const std::string& title, Vector3f& io_color)
{
    auto& c = io_color;
    QColor newColor =
        QColorDialog::getColor(
            QColor::fromRgbF(c[0], c[1], c[2], 1.0f),
            MainWindow::instance(), title.c_str());
    
    if(newColor.isValid()){
        io_color << newColor.redF(), newColor.greenF(), newColor.blueF();
        return true;
    }
    return false;
}


ConfigWidgetSet::ConfigWidgetSet(SceneRendererConfig::Impl* config_)
    : config(config_)
{
    ownerWidget = new QWidget;
    lightingPanel = nullptr;
    signalObjects.reserve(20);

    for(auto& radio : lightingModeRadios){
        radio = nullptr;
    }
    lightingModeRadios[GLSceneRenderer::NormalLighting] =
        new RadioButton(_("Normal"), ownerWidget);
    lightingModeRadios[GLSceneRenderer::MinimumLighting] =
        new RadioButton(_("Minimum"), ownerWidget);
    lightingModeRadios[GLSceneRenderer::SolidColorLighting] =
        new RadioButton(_("Solid color"), ownerWidget);

    lightingModeGroup = new ButtonGroup(ownerWidget);
    for(int i=0; i < config->lightingMode.size(); ++i){
        if(auto radio = lightingModeRadios[i]){
            lightingModeGroup->addButton(radio, i);
        }
    }
    lightingModeGroup->sigButtonToggled().connect(
        [this](int mode, bool on){
            if(on){
                config->lightingMode.select(mode);
                config->updateRenderers(Lighting, true);
            }
        });
    signalObjects.push_back(lightingModeGroup);

    smoothShadingCheck = new CheckBox(_("Smooth shading"), ownerWidget);
    smoothShadingCheck->sigToggled().connect(
        [this](bool on){
            config->isSmoothShadingEnabled = on;
            config->updateRenderers(Lighting, true);
        });
    signalObjects.push_back(smoothShadingCheck);

    for(auto& radio : cullingModeRadios){
        radio = nullptr;
    }
    cullingModeRadios[GLSceneRenderer::ENABLE_BACK_FACE_CULLING] =
        new RadioButton(_("Enabled"), ownerWidget);
    cullingModeRadios[GLSceneRenderer::DISABLE_BACK_FACE_CULLING] =
        new RadioButton(_("Disabled"), ownerWidget);
    cullingModeRadios[GLSceneRenderer::FORCE_BACK_FACE_CULLING] =
        new RadioButton(_("Forced"), ownerWidget);

    cullingModeGroup = new ButtonGroup(ownerWidget);
    for(int i=0; i < config->cullingMode.size(); ++i){
        if(auto radio = cullingModeRadios[i]){
            cullingModeGroup->addButton(radio, i);
        }
    }
    cullingModeGroup->sigButtonToggled().connect(
        [this](int mode, bool on){
            if(on){
                config->cullingMode.select(mode);
                config->updateRenderers(Lighting, true);
            }
        });
    signalObjects.push_back(cullingModeGroup);

    headLightCheck = new CheckBox(_("Head light"), ownerWidget);
    headLightCheck->sigToggled().connect(
        [this](bool on){
            config->isHeadLightEnabled = on;
            config->updateRenderers(Lighting, true);
        });
    signalObjects.push_back(headLightCheck);

    headLightIntensitySpin = new DoubleSpinBox(ownerWidget);
    headLightIntensitySpin->setDecimals(2);
    headLightIntensitySpin->setSingleStep(0.01);    
    headLightIntensitySpin->setRange(0.0, 2.0);
    headLightIntensitySpin->sigValueChanged().connect(
        [this](double intensity){
            config->headLightIntensity = intensity;
            config->updateRenderers(Lighting, true);
        });
    signalObjects.push_back(headLightIntensitySpin);

    worldLightCheck = new CheckBox(_("World light"), ownerWidget);
    worldLightCheck->sigToggled().connect(
        [this](bool on){
            config->isWorldLightEnabled = on;
            config->updateRenderers(Lighting, true);
        });
    signalObjects.push_back(worldLightCheck);

    worldLightIntensitySpin = new DoubleSpinBox(ownerWidget);
    worldLightIntensitySpin->setDecimals(2);
    worldLightIntensitySpin->setSingleStep(0.01);    
    worldLightIntensitySpin->setRange(0.0, 2.0);
    worldLightIntensitySpin->sigValueChanged().connect(
        [this](double intensity){
            config->worldLightIntensity = intensity;
            config->updateRenderers(Lighting, true);
        });
    signalObjects.push_back(worldLightIntensitySpin);

    ambientLightCheck = new CheckBox(_("Ambient light"), ownerWidget);
    ambientLightCheck->sigToggled().connect(
        [this](bool on){
            config->isAmbientLightEnabled = on;
            config->updateRenderers(Lighting, true);
        });
    signalObjects.push_back(ambientLightCheck);
    
    ambientIntensitySpin = new DoubleSpinBox(ownerWidget);
    ambientIntensitySpin->setDecimals(2);
    ambientIntensitySpin->setSingleStep(0.01);    
    ambientIntensitySpin->setRange(0.0, 1.0);
    ambientIntensitySpin->sigValueChanged().connect(
        [this](double intensity){
            config->ambientIntensity = intensity;
            config->updateRenderers(Lighting, true);
        });
    signalObjects.push_back(ambientIntensitySpin);

    additionalLightSetCheck = new CheckBox(_("Additional lights"), ownerWidget);
    additionalLightSetCheck->sigToggled().connect(
        [this](bool on){
            config->isAdditionalLightSetEnabled = on;
            config->updateRenderers(Lighting, true);
        });
    signalObjects.push_back(additionalLightSetCheck);

    worldLightShadowCheck = new CheckBox(_("Shadow"), ownerWidget);
    worldLightShadowCheck->sigToggled().connect(
        [this](bool on){
            config->isWorldLightShadowEnabled = on;
            config->updateRenderers(Shadow, true);
        });
    signalObjects.push_back(worldLightShadowCheck);

    for(int i=0; i < MaxNumShadows; ++i){
        auto& sws = shadowWidgetSets[i];
        sws.check = new CheckBox(_("Shadow for light"), ownerWidget);
        sws.check->sigToggled().connect(
            [this, i](bool on){
                config->shadowInfos[i].isEnabled = on;
                config->updateRenderers(Shadow, true);
            });
        signalObjects.push_back(sws.check);

        auto spin = new SpinBox(ownerWidget);
        spin->setToolTip(_("Index of the light source causing the shadow"));
        spin->setRange(1, 99);
        spin->sigValueChanged().connect(
            [this, i](int index){
                config->shadowInfos[i].lightIndex = index - 1;
                config->updateRenderers(Shadow, true);
            });
        signalObjects.push_back(spin);
        sws.lightSpin = spin;
    }

    shadowAntiAliasingCheck = new CheckBox(_("Anti-aliasing of shadows"), ownerWidget);
    shadowAntiAliasingCheck->sigToggled().connect(
        [this](bool on){
            config->isShadowAntiAliasingEnabled = on;
            config->updateRenderers(Shadow, true);
        });
    signalObjects.push_back(shadowAntiAliasingCheck);

    textureCheck = new CheckBox(_("Texture"), ownerWidget);
    textureCheck->sigToggled().connect(
        [this](bool on){
            config->isTextureEnabled = on;
            config->updateRenderers(Drawing, true);
        });
    signalObjects.push_back(textureCheck);

    fogCheck = new CheckBox(_("Fog"), ownerWidget);
    fogCheck->sigToggled().connect(
        [this](bool on){
            config->isFogEnabled = on;
            config->updateRenderers(Effect, true);
        });
    signalObjects.push_back(fogCheck);

    backgroundColorButton = new PushButton(_("Background color"), ownerWidget);
    backgroundColorButton->sigClicked().connect(
        [this](){
            if(SceneRendererConfig::inputColorWithColorDialog(
                   _("Background Color"), config->backgroundColor)){
                config->updateRenderers(Drawing, true);
            }
        });

    defaultColorButton = new PushButton(_("Default color"), ownerWidget);
    defaultColorButton->sigClicked().connect(
        [this](){
            if(SceneRendererConfig::inputColorWithColorDialog(
                   _("Default Color"), config->defaultColor)){
                config->updateRenderers(Drawing, true);
            }
        });

    lineWidthSpin = new DoubleSpinBox(ownerWidget);
    lineWidthSpin->setDecimals(1);
    lineWidthSpin->setRange(0.1, 10.0);
    lineWidthSpin->setSingleStep(0.1);
    lineWidthSpin->setValue(1.0);
    lineWidthSpin->sigValueChanged().connect(
        [this](double width){
            config->lineWidth = width;
            config->updateRenderers(Drawing, true);
        });
    signalObjects.push_back(lineWidthSpin);

    pointSizeSpin = new DoubleSpinBox(ownerWidget);
    pointSizeSpin->setDecimals(1);
    pointSizeSpin->setRange(0.1, 10.0);
    pointSizeSpin->setSingleStep(0.1);
    pointSizeSpin->setValue(1.0);
    pointSizeSpin->sigValueChanged().connect(
        [this](double size){
            config->pointSize = size;
            config->updateRenderers(Drawing, true);
        });
    signalObjects.push_back(pointSizeSpin);

    upsideDownCheck = new CheckBox(_("Upside down"), ownerWidget);
    upsideDownCheck->sigToggled().connect(
        [this](bool on){
            config->isUpsideDownEnabled = on;
            config->updateRenderers(Effect, true);
        });
    signalObjects.push_back(upsideDownCheck);

    updateWidgets();
}


ConfigWidgetSet::~ConfigWidgetSet()
{
    delete ownerWidget;
}


QWidget* ConfigWidgetSet::createLightingPanel()
{
    if(lightingPanel){
        return lightingPanel;
    }
    
    lightingPanel = new QWidget(ownerWidget);
    auto vbox = new QVBoxLayout;
    vbox->setContentsMargins(0, 0, 0, 0);
    lightingPanel->setLayout(vbox);
    auto& ws = *config->widgetSet;
    QHBoxLayout* hbox;

    hbox = new QHBoxLayout;
    hbox->addWidget(new QLabel(_("Lighting mode")));
    for(int i=0; i < config->lightingMode.size(); ++i){
        if(auto radio = ws.lightingModeRadios[i]){
            hbox->addWidget(radio);
        }
    }
    hbox->addStretch();
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout;
    hbox->addWidget(ws.smoothShadingCheck);
    hbox->addSpacing(10);
    hbox->addWidget(new QLabel(_("Back face culling: ")));
    for(int i=0; i < config->cullingMode.size(); ++i){
        if(auto radio = ws.cullingModeRadios[i]){
            hbox->addWidget(radio);
        }
    }
    hbox->addStretch();
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout;
    hbox->addWidget(ws.worldLightCheck);
    hbox->addWidget(new QLabel(_("Intensity")));
    hbox->addWidget(ws.worldLightIntensitySpin);
    hbox->addWidget(ws.worldLightShadowCheck);
    hbox->addStretch();
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout;
    hbox->addWidget(ws.headLightCheck);
    hbox->addWidget(new QLabel(_("Intensity")));
    hbox->addWidget(ws.headLightIntensitySpin);
    hbox->addSpacing(10);
    hbox->addWidget(ws.ambientLightCheck);
    hbox->addWidget(new QLabel(_("Intensity")));
    hbox->addWidget(ws.ambientIntensitySpin);
    hbox->addStretch();
    vbox->addLayout(hbox);
    
    hbox = new QHBoxLayout;
    hbox->addWidget(ws.additionalLightSetCheck);
    for(int i=0; i < MaxNumShadows; ++i){
        if(i > 0){
            hbox->addSpacing(10);
        }
        auto lightBox = new QHBoxLayout;
        lightBox->setSpacing(0);
        auto& sws = shadowWidgetSets[i];
        lightBox->addWidget(sws.check);
        lightBox->addWidget(sws.lightSpin);
        hbox->addLayout(lightBox);
    }
    hbox->addStretch();
    vbox->addLayout(hbox);
    
    hbox = new QHBoxLayout;
    hbox->addWidget(ws.textureCheck);
    hbox->addWidget(fogCheck);
    hbox->addWidget(ws.shadowAntiAliasingCheck);
    hbox->addStretch();
    vbox->addLayout(hbox);

    return lightingPanel;
}


void ConfigWidgetSet::updateWidgets()
{
    for(auto& obj : signalObjects){
        obj->blockSignals(true);
    }
    
    lightingModeRadios[config->lightingMode.which()]->setChecked(true);
    smoothShadingCheck->setChecked(config->isSmoothShadingEnabled);
    cullingModeRadios[config->cullingMode.which()]->setChecked(true);
    headLightCheck->setChecked(config->isHeadLightEnabled);
    headLightIntensitySpin->setValue(config->headLightIntensity);
    worldLightCheck->setChecked(config->isWorldLightEnabled);
    worldLightIntensitySpin->setValue(config->worldLightIntensity);
    worldLightShadowCheck->setChecked(config->isWorldLightShadowEnabled);
    ambientLightCheck->setChecked(config->isAmbientLightEnabled);
    ambientIntensitySpin->setValue(config->ambientIntensity);
    additionalLightSetCheck->setChecked(config->isAdditionalLightSetEnabled);
    textureCheck->setChecked(config->isTextureEnabled);
    fogCheck->setChecked(config->isFogEnabled);
    shadowAntiAliasingCheck->setChecked(config->isShadowAntiAliasingEnabled);

    updateShadowWidgets();
    
    lineWidthSpin->setValue(config->lineWidth);
    pointSizeSpin->setValue(config->pointSize);
    upsideDownCheck->setChecked(config->isUpsideDownEnabled);

    for(auto& obj : signalObjects){
        obj->blockSignals(false);
    }
}


void ConfigWidgetSet::updateShadowWidgets()
{
    bool on = config->isShadowCastingAvailable;
    
    for(int i=0; i < MaxNumShadows; ++i){
        auto& sws = shadowWidgetSets[i];
        auto& info = config->shadowInfos[i];
        sws.check->setEnabled(on);
        sws.check->setChecked(info.isEnabled && on);
        sws.lightSpin->setEnabled(on);
        sws.lightSpin->setValue(info.lightIndex + 1);
    }
    
    shadowAntiAliasingCheck->setEnabled(on);
}
    
    
ConfigDialog::ConfigDialog(SceneRendererConfig::Impl* config)
    : config(config)
{
    auto ws = config->getOrCreateConfigWidgetSet();

    auto vbox = new QVBoxLayout;
    setLayout(vbox);

    auto lightingPanel = ws->createLightingPanel();
    vbox->addWidget(lightingPanel);
    
    QHBoxLayout* hbox;

    hbox = new QHBoxLayout;
    hbox->addWidget(ws->backgroundColorButton);
    hbox->addWidget(ws->upsideDownCheck);
    hbox->addStretch();
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout;
    hbox->addWidget(ws->defaultColorButton);
    hbox->addWidget(new QLabel(_("Default line width")));
    hbox->addWidget(ws->lineWidthSpin);
    hbox->addWidget(new QLabel(_("Default point size")));
    hbox->addWidget(ws->pointSizeSpin);
    hbox->addStretch();
    vbox->addLayout(hbox);
}
