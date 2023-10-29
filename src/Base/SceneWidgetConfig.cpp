#include "SceneWidgetConfig.h"
#include "SceneWidget.h"
#include "Buttons.h"
#include "ButtonGroup.h"
#include "CheckBox.h"
#include "SpinBox.h"
#include "DoubleSpinBox.h"
#include "Separator.h"
#include <cnoid/GLSceneRenderer>
#include <cnoid/ValueTree>
#include <cnoid/EigenArchive>
#include <QBoxLayout>
#include <QGridLayout>
#include <QLabel>
#include <fmt/format.h>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

namespace {

constexpr double defaultNormalLength = 0.01;

const char* axisSymbols[] = { "X", "Y", "Z" };

enum ConfigCategory
{
    CameraCategory = 1 << 1,
    DrawingCategory = 1 << 2,
    GridCategory = 1 << 3,
    AllCategories = CameraCategory | DrawingCategory | GridCategory
};

class ConfigWidgetSet
{
public:
    SceneWidgetConfig::Impl* config;
    
    // Used for deleting all the widgets when the ConfigWidgetSet instance is deleted.
    QWidget* ownerWidget;

    QWidget* cameraPanel;
    QWidget* backgroundPanel;
    QWidget* drawingPanel;
    QWidget* debugPanel;

    DoubleSpinBox* fieldOfViewSpin;
    DoubleSpinBox* nearClipSpin;
    DoubleSpinBox* farClipSpin;
    CheckBox* restrictCameraRollCheck;
    ButtonGroup* verticalAxisGroup;
    RadioButton* verticalAxisRadios[3];
    CheckBox* coordinateAxesCheck;

    struct GridWidgetSet {
        CheckBox* check;
        DoubleSpinBox* spanSpin;
        DoubleSpinBox* intervalSpin;
        PushButton* colorButton;
    };
    GridWidgetSet gridWidgetSets[3];
    
    CheckBox* normalVisualizationCheck;
    DoubleSpinBox* normalLengthSpin;
    CheckBox* lightweightViewChangeCheck;
    PushButton* fpsTestButton;
    SpinBox* fpsTestIterationSpin;
    bool isDoingFpsTest;
    PushButton* pickingImageButton;

    // The following objects are obtained from SceneRendererConfig
    PushButton* backgroundColorButton;
    PushButton* defaultColorButton;
    DoubleSpinBox* lineWidthSpin;
    DoubleSpinBox* pointSizeSpin;;
    CheckBox* upsideDownCheck;

    vector<QObject*> signalObjects;

    ConfigWidgetSet(SceneWidgetConfig::Impl* config);
    ~ConfigWidgetSet();
    QWidget* createCameraPanel();
    QWidget* createBackgroundPanel();
    QWidget* createDrawingPanel();
    QWidget* createDebugPanel();
    void updateWidgets();
    void onFpsTestButtonClicked();
};

}

namespace cnoid {

class SceneWidgetConfig::Impl
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    SceneWidgetConfig* self;
    vector<SceneWidget*> sceneWidgets;

    double fieldOfView; // degree
    double nearClipDistance;
    double farClipDistance;
    int verticalAxis;
    bool isCameraRollRistricted;
    bool isCoordinateAxesEnabled;

    struct GridInfo{
        double span;
        double interval;
        Vector3f color;
        bool isEnabled;
    };
    GridInfo gridInfos[3];
        
    bool isNormalVisualizationEnabled;
    double normalLength;
    bool isLightweightViewChangedEnabled;
    
    ConfigWidgetSet* widgetSet;

    Impl(SceneWidgetConfig* self);
    Impl(const Impl& org, SceneWidgetConfig* self);
    ~Impl();
    void updateSceneWidget(SceneWidget* sceneWidget, unsigned int categories);
    void updateSceneWidgets(unsigned int categories, bool isInteractive);
    bool store(Mapping* archive);
    bool restore(const Mapping* archive);
    ConfigWidgetSet* getOrCreateConfigWidgetSet();
    void doFpsTest(int iteration);
    void cancelFpsTest();
    void showPickingImageWindow();
};

}


SceneWidgetConfig::SceneWidgetConfig()
{
    impl = new Impl(this);
}


SceneWidgetConfig::Impl::Impl(SceneWidgetConfig* self)
    : self(self)
{
    fieldOfView = 35.0;
    nearClipDistance = 0.04;
    farClipDistance = 200.0;
    verticalAxis = 2; // Z
    isCameraRollRistricted = true;
    isCoordinateAxesEnabled = true;

    for(auto& info : gridInfos){
        info.span = 10.0;
        info.interval = 0.5;
        info.color << 0.9f, 0.9f, 0.9f;
        info.isEnabled = false;
    }
    gridInfos[SceneWidget::XY_Grid].isEnabled = true;

    isNormalVisualizationEnabled = false;
    normalLength = defaultNormalLength;
    isLightweightViewChangedEnabled = false;
    
    widgetSet = nullptr;
}


SceneWidgetConfig::SceneWidgetConfig(const SceneWidgetConfig& org)
    : SceneRendererConfig(org)
{
    impl = new Impl(*org.impl, this);
}


SceneWidgetConfig::Impl::Impl(const Impl& org, SceneWidgetConfig* self)
    : self(self)
{
    fieldOfView = org.fieldOfView;
    nearClipDistance = org.nearClipDistance;
    farClipDistance = org.farClipDistance;
    verticalAxis = org.verticalAxis;
    isCameraRollRistricted = org.isCameraRollRistricted;
    isCoordinateAxesEnabled = org.isCoordinateAxesEnabled;
    
    for(int i=0; i < 3; ++i){
        gridInfos[i] = org.gridInfos[i];
    }
        
    isNormalVisualizationEnabled = org.isNormalVisualizationEnabled;
    normalLength = org.normalLength;
    isLightweightViewChangedEnabled = org.isLightweightViewChangedEnabled;
    
    widgetSet = nullptr;
}


SceneWidgetConfig::~SceneWidgetConfig()
{
    delete impl;
}


SceneWidgetConfig::Impl::~Impl()
{
    if(widgetSet){
        delete widgetSet;
    }
}


void SceneWidgetConfig::addSceneWidget(SceneWidget* widget, bool doUpdateSceneWidget)
{
    impl->sceneWidgets.push_back(widget);
    addRenderer(widget->renderer(), doUpdateSceneWidget);

    if(doUpdateSceneWidget){
        impl->updateSceneWidget(widget, AllCategories);
    }
}


void SceneWidgetConfig::removeSceneWidget(SceneWidget* widget)
{
    removeRenderer(widget->renderer());
    
    auto& widgets = impl->sceneWidgets;
    auto it = std::find(widgets.begin(), widgets.end(), widget);
    if(it != widgets.end()){
        widgets.erase(it);
    }
}    


void SceneWidgetConfig::clearSceneWidgets()
{
    impl->sceneWidgets.clear();
    clearRenderers();
}


void SceneWidgetConfig::Impl::updateSceneWidget(SceneWidget* sceneWidget, unsigned int categories)
{
    if(categories & CameraCategory){
        sceneWidget->setFieldOfView(radian(fieldOfView));
        sceneWidget->setClipDistances(nearClipDistance, farClipDistance);
        sceneWidget->setInteractiveCameraRollRestricted(isCameraRollRistricted);
        sceneWidget->setVerticalAxis(verticalAxis);
    }
    
    if(categories & DrawingCategory){
        sceneWidget->setCoordinateAxes(isCoordinateAxesEnabled);
        auto renderer = sceneWidget->renderer<GLSceneRenderer>();
        renderer->setNormalVisualizationLength(normalLength);
        renderer->setNormalVisualizationEnabled(isNormalVisualizationEnabled);
        sceneWidget->setLightweightViewChangeEnabled(isLightweightViewChangedEnabled);
    }

    if(categories & GridCategory){
        for(int i=0; i < 3; ++i){
            auto& info = gridInfos[i];
            auto grid = static_cast<SceneWidget::GridPlane>(i);
            sceneWidget->setGridEnabled(grid, info.isEnabled);
            sceneWidget->setGridGeometry(grid, info.span, info.interval);
            sceneWidget->setGridColor(grid, info.color);
        }
        sceneWidget->updateGrids();
    }
}


void SceneWidgetConfig::Impl::updateSceneWidgets(unsigned int categories, bool isInteractive)
{
    for(auto& sceneWidget : sceneWidgets){
        updateSceneWidget(sceneWidget, categories);
        if(isInteractive){
            sceneWidget->renderScene();
        }
    }
}


// Public version of this function updates all the targets including those
// implemented in the base classes
void SceneWidgetConfig::updateSceneWidgets()
{
    updateRenderers();
    impl->updateSceneWidgets(AllCategories, false);
}


void SceneWidgetConfig::onRendererConfigUpdated(bool isInteractive)
{
    if(isInteractive){
        for(auto& sceneWidget : impl->sceneWidgets){
            sceneWidget->renderScene();
        }
    }
}


bool SceneWidgetConfig::store(Mapping* archive)
{
    SceneRendererConfig::store(archive);
    return impl->store(archive);
}


bool SceneWidgetConfig::Impl::store(Mapping* archive)
{
    archive->write("field_of_view", fieldOfView);
    auto distances = archive->createFlowStyleListing("clip_distances");
    distances->append(nearClipDistance);
    distances->append(farClipDistance);
    
    if(!isCameraRollRistricted){
        archive->write("restrictCameraRoll", false);
    }
    if(verticalAxis != 2){
        archive->write("verticalAxis", axisSymbols[verticalAxis]);
    }

    static const char* planes[] = { "floor", "xz", "yz" };
    for(int i=0; i < 3; ++i){
        auto& info = gridInfos[i];
        auto& plane = planes[i];
        archive->write(format("{}Grid", plane), info.isEnabled);
        archive->write(format("{}GridSpan", plane), info.span);
        archive->write(format("{}GridInterval", plane), info.interval);
    }
    write(archive, "xy_grid_color", gridInfos[0].color);
    write(archive, "xz_grid_color", gridInfos[1].color);
    write(archive, "yz_grid_color", gridInfos[2].color);

    if(isNormalVisualizationEnabled){
        archive->write("normalVisualization", true);
    }
    if(normalLength != defaultNormalLength){
        archive->write("normalLength", normalLength);
    }
    if(isLightweightViewChangedEnabled){
        archive->write("lightweightViewChange", true);
    }
    if(!isCoordinateAxesEnabled){
        archive->write("coordinateAxes", false);
    }

    return true;
}


bool SceneWidgetConfig::restore(const Mapping* archive)
{
    return SceneRendererConfig::restore(archive) && impl->restore(archive);
}


bool SceneWidgetConfig::Impl::restore(const Mapping* archive)
{
    bool cameraStateFound = false;
    if(archive->read("field_of_view", fieldOfView)){
        cameraStateFound = true;
    }
    auto clipDistancePair = archive->findListing("clip_distances");
    if(clipDistancePair->isValid()){
        if(clipDistancePair->size() >= 1){
            nearClipDistance = clipDistancePair->at(0)->toDouble();
            cameraStateFound = true;
            if(clipDistancePair->size() >= 2){
                farClipDistance = clipDistancePair->at(1)->toDouble();
            }
        }
    }
    if(!cameraStateFound){
        // For the backward compatibility
        auto cameraList = archive->findListing("cameras");
        if(cameraList->isValid()){
            for(int i=0; i < cameraList->size(); ++i){
                auto state = cameraList->at(i)->toMapping();
                if(state->read("fieldOfView", fieldOfView)){
                    fieldOfView = degree(fieldOfView);
                    cameraStateFound = true;
                }
                cameraStateFound |= state->read("near", nearClipDistance);
                cameraStateFound |= state->read("far", farClipDistance);
                if(cameraStateFound){
                    break;
                }
            }
        }
        if(!cameraStateFound){ // Even older format
            auto state = archive->findMapping("camera");
            if(state->isValid()){
                if(state->read("fieldOfView", fieldOfView)){
                    fieldOfView = degree(fieldOfView);
                }
                state->read("near", nearClipDistance);
                state->read("far", farClipDistance);
            }
        }
    }

    isCameraRollRistricted = archive->get("restrictCameraRoll", true);

    string symbol;
    verticalAxis = 2;
    if(archive->read("verticalAxis", symbol)){
        for(int i=0; i < 3; ++i){
            if(symbol == axisSymbols[i]){
                verticalAxis = i;
                break;
            }
        }
    }
    
    static const char* planes[] = { "floor", "xz", "yz" };
    for(int i=0; i < 3; ++i){
        auto& info = gridInfos[i];
        auto& plane = planes[i];
        archive->read(format("{}Grid", plane), info.isEnabled);
        archive->read(format("{}GridSpan", plane), info.span);
        archive->read(format("{}GridInterval", plane), info.interval);
    }
    read(archive, "xy_grid_color", gridInfos[0].color);
    read(archive, "xz_grid_color", gridInfos[1].color);
    read(archive, "yz_grid_color", gridInfos[2].color);

    isNormalVisualizationEnabled = archive->get("normalVisualization", false);
    normalLength = archive->get("normalLength", defaultNormalLength);
    isLightweightViewChangedEnabled = archive->get("lightweightViewChange", false);
    
    isCoordinateAxesEnabled = archive->get("coordinateAxes", true);

    if(widgetSet){
        widgetSet->updateWidgets();
    }

    updateSceneWidgets(AllCategories, false);

    return true;
}


void SceneWidgetConfig::createConfigWidgets()
{
    impl->getOrCreateConfigWidgetSet();
}


ConfigWidgetSet* SceneWidgetConfig::Impl::getOrCreateConfigWidgetSet()
{
    if(!widgetSet){
        self->SceneRendererConfig::createConfigWidgets();
        
        widgetSet = new ConfigWidgetSet(this);
        widgetSet->backgroundColorButton = self->backgroundColorButton();
        widgetSet->defaultColorButton = self->defaultColorButton();
        widgetSet->lineWidthSpin = self->lineWidthSpin();
        widgetSet->pointSizeSpin = self->pointSizeSpin();
        widgetSet->upsideDownCheck = self->upsideDownCheck();
    }
    return widgetSet;
}


QWidget* SceneWidgetConfig::getOrCreateCameraPanel()
{
    auto ws = impl->getOrCreateConfigWidgetSet();
    if(!ws->cameraPanel){
        ws->createCameraPanel();
    }
    return ws->cameraPanel;
}


QWidget* SceneWidgetConfig::getOrCreateBackgroundPanel()
{
    auto ws = impl->getOrCreateConfigWidgetSet();
    if(!ws->backgroundPanel){
        ws->createBackgroundPanel();
    }
    return ws->backgroundPanel;
}


QWidget* SceneWidgetConfig::getOrCreateDrawingPanel()
{
    auto ws = impl->getOrCreateConfigWidgetSet();
    if(!ws->drawingPanel){
        ws->createDrawingPanel();
    }
    return ws->drawingPanel;
}


QWidget* SceneWidgetConfig::getOrCreateDebugPanel()
{
    auto ws = impl->getOrCreateConfigWidgetSet();
    if(!ws->debugPanel){
        ws->createDebugPanel();
    }
    return ws->debugPanel;
}


PushButton* SceneWidgetConfig::fpsTestButton()
{
    return impl->getOrCreateConfigWidgetSet()->fpsTestButton;
}


void SceneWidgetConfig::Impl::doFpsTest(int iteration)
{
    for(auto& widget : sceneWidgets){
        widget->doFpsTest(iteration);
    }
}


void SceneWidgetConfig::Impl::cancelFpsTest()
{
    for(auto& widget : sceneWidgets){
        widget->cancelFpsTest();
    }
}


void SceneWidgetConfig::Impl::showPickingImageWindow()
{
    for(auto& widget : sceneWidgets){
        //widget->showPickingImageWindow();
    }
}


ConfigWidgetSet::ConfigWidgetSet(SceneWidgetConfig::Impl* config_)
    : config(config_)
{
    ownerWidget = new QWidget;

    cameraPanel = nullptr;
    backgroundPanel = nullptr;
    drawingPanel = nullptr;
    debugPanel = nullptr;

    //! \todo check the actual size
    signalObjects.reserve(10);

    fieldOfViewSpin = new DoubleSpinBox(ownerWidget);
    fieldOfViewSpin->setDecimals(0);
    fieldOfViewSpin->setRange(1, 179);
    fieldOfViewSpin->sigValueChanged().connect(
        [this](int fov){
            config->fieldOfView = fov;
            config->updateSceneWidgets(CameraCategory, true);
        });
    signalObjects.push_back(fieldOfViewSpin);

    nearClipSpin = new DoubleSpinBox(ownerWidget);
    nearClipSpin->setDecimals(4);
    nearClipSpin->setRange(0.0001, 9.9999);
    nearClipSpin->setSingleStep(0.0001);
    nearClipSpin->sigValueChanged().connect(
        [this](double d){
            config->nearClipDistance = d;
            config->updateSceneWidgets(CameraCategory, true);
        });
    signalObjects.push_back(nearClipSpin);

    farClipSpin = new DoubleSpinBox(ownerWidget);
    farClipSpin->setDecimals(1);
    farClipSpin->setRange(0.1, 9999999.9);
    farClipSpin->setSingleStep(0.1);
    farClipSpin->sigValueChanged().connect(
        [this](double d){
            config->farClipDistance = d;
            config->updateSceneWidgets(CameraCategory, true);
        });
    signalObjects.push_back(farClipSpin);

    restrictCameraRollCheck = new CheckBox(_("Restrict camera roll"), ownerWidget);
    restrictCameraRollCheck->sigToggled().connect(
        [this](bool on){
            config->isCameraRollRistricted = on;
            config->updateSceneWidgets(CameraCategory, true);
        });
    signalObjects.push_back(restrictCameraRollCheck);

    verticalAxisGroup = new ButtonGroup(ownerWidget);
    for(int i=0; i< 3; ++i){
        verticalAxisRadios[i] = new RadioButton(axisSymbols[i], ownerWidget);
        verticalAxisGroup->addButton(verticalAxisRadios[i], i);
    }
    verticalAxisGroup->sigButtonToggled().connect(
        [this](int axis, bool on){
            if(on){
                config->verticalAxis = axis;
                config->updateSceneWidgets(CameraCategory, true);
            }
        });
    signalObjects.push_back(verticalAxisGroup);

    coordinateAxesCheck = new CheckBox(_("Coordinate axes"), ownerWidget);
    coordinateAxesCheck->sigToggled().connect(
        [this](bool on){
            config->isCoordinateAxesEnabled = on;
            config->updateSceneWidgets(DrawingCategory, true);
        });
    signalObjects.push_back(coordinateAxesCheck);

    const char* gridLabels[] = { _("XY (Floor) Grid"), _("XZ Grid"), _("YZ Grid") };

    for(int i=0; i < 3; ++i){
        auto& gws = gridWidgetSets[i];
        gws.check = new CheckBox(gridLabels[i], ownerWidget);
        gws.check->sigToggled().connect(
            [this, i](bool on){
                config->gridInfos[i].isEnabled = on;
                config->updateSceneWidgets(GridCategory, true);
            });
        signalObjects.push_back(gws.check);

        auto spanSpin = new DoubleSpinBox(ownerWidget);
        spanSpin->setAlignment(Qt::AlignCenter);
    	spanSpin->setDecimals(1);
    	spanSpin->setRange(0.0, 99.9);
        spanSpin->setSingleStep(0.1);
        spanSpin->sigValueChanged().connect(
            [this, i](double span){
                config->gridInfos[i].span = span;
                config->updateSceneWidgets(GridCategory, true);
            });
        signalObjects.push_back(spanSpin);
        gws.spanSpin = spanSpin;

        auto intervalSpin = new DoubleSpinBox(ownerWidget);
        intervalSpin->setAlignment(Qt::AlignCenter);
    	intervalSpin->setDecimals(3);
    	intervalSpin->setRange(0.001, 9.999);
        intervalSpin->setSingleStep(0.001);
        intervalSpin->sigValueChanged().connect(
            [this, i](double interval){
                config->gridInfos[i].interval = interval;
                config->updateSceneWidgets(GridCategory, true);
            });
        signalObjects.push_back(intervalSpin);
        gws.intervalSpin = intervalSpin;

        gws.colorButton = new PushButton(_("Color"), ownerWidget);
        gws.colorButton->sigClicked().connect(
            [this, i, gridLabels](){
                auto& color = config->gridInfos[i].color;
                if(SceneRendererConfig::inputColorWithColorDialog(
                       format(_("{0} Color"), gridLabels[i]), color)){
                    config->updateSceneWidgets(GridCategory, true);
                }
            });
    }
    
    normalVisualizationCheck = new CheckBox(_("Normal Visualization"), ownerWidget);
    normalVisualizationCheck->sigToggled().connect(
        [this](bool on){
            config->isNormalVisualizationEnabled = on;
            config->updateSceneWidgets(DrawingCategory, true);
        });
    signalObjects.push_back(normalVisualizationCheck);

    normalLengthSpin = new DoubleSpinBox(ownerWidget);
    normalLengthSpin->setToolTip(_("Normal length"));
    normalLengthSpin->setDecimals(3);
    normalLengthSpin->setRange(0.0, 1000.0);
    normalLengthSpin->setSingleStep(0.001);
    normalLengthSpin->sigValueChanged().connect(
        [this](double length){
            config->normalLength = length;
            config->updateSceneWidgets(DrawingCategory, true);
        });
    signalObjects.push_back(normalLengthSpin);

    lightweightViewChangeCheck = new CheckBox(_("Lightweight view change"), ownerWidget);
    lightweightViewChangeCheck->sigToggled().connect(
        [this](bool on){
            config->isLightweightViewChangedEnabled = on;
            config->updateSceneWidgets(DrawingCategory, true);
        });
    signalObjects.push_back(lightweightViewChangeCheck);

    fpsTestButton = new PushButton(_("FPS Test"), ownerWidget);

    /**
       Use QObject::connect function directly instead of sigClicked so that the click
       to cancel the FPS test can be processed during QCoreApplication::processEvents
       called inside the doFpsTest function.
    */
    QObject::connect(
        fpsTestButton, &QPushButton::clicked,
        [this](){
            if(!isDoingFpsTest){
                isDoingFpsTest = true;
                auto label = fpsTestButton->text();
                fpsTestButton->setText(_("Cancel"));
                config->doFpsTest(fpsTestIterationSpin->value());
                fpsTestButton->setText(label);
                isDoingFpsTest = false;
            } else {
                isDoingFpsTest = false;
                config->cancelFpsTest();
            }
        });

    fpsTestIterationSpin = new SpinBox(ownerWidget);
    fpsTestIterationSpin->setToolTip(_("Number of test iterations"));
    fpsTestIterationSpin->setRange(1, 99);
    fpsTestIterationSpin->setValue(1);

    isDoingFpsTest = false;

    pickingImageButton = new PushButton(_("Picking image"), ownerWidget);
    pickingImageButton->sigClicked().connect([this](){ config->showPickingImageWindow(); });

    updateWidgets();
}


ConfigWidgetSet::~ConfigWidgetSet()
{
    delete ownerWidget;
}


QWidget* ConfigWidgetSet::createCameraPanel()
{
    if(cameraPanel){
        return cameraPanel;
    }

    cameraPanel = new QWidget(ownerWidget);
    auto vbox = new QVBoxLayout;
    vbox->setContentsMargins(0, 0, 0, 0);
    cameraPanel->setLayout(vbox);
    QHBoxLayout* hbox;

    hbox = new QHBoxLayout;
    hbox->addWidget(new QLabel(_("Field of view")));
    hbox->addWidget(fieldOfViewSpin);
    hbox->addWidget(new QLabel("[deg]"));
    hbox->addSpacing(8);
    hbox->addWidget(new QLabel(_("Clip distances")));
    hbox->addSpacing(8);
    hbox->addWidget(new QLabel(_("Near")));
    hbox->addWidget(nearClipSpin);
    hbox->addWidget(new QLabel(_("Far")));
    hbox->addWidget(farClipSpin);
    hbox->addStretch();
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout;
    hbox->addWidget(restrictCameraRollCheck);
    hbox->addSpacing(8);
    hbox->addWidget(new QLabel(_("Vertical axis")));
    for(int i=0; i < 3; ++i){
        hbox->addWidget(verticalAxisRadios[i]);
    }
    hbox->addSpacing(8);
    hbox->addWidget(upsideDownCheck);
    hbox->addStretch();
    vbox->addLayout(hbox);

    return cameraPanel;
}


QWidget* ConfigWidgetSet::createBackgroundPanel()
{
    if(backgroundPanel){
        return backgroundPanel;
    }

    backgroundPanel = new QWidget(ownerWidget);
    auto grid = new QGridLayout;
    grid->setContentsMargins(0, 0, 0, 0);
    backgroundPanel->setLayout(grid);
    
    grid->addWidget(backgroundColorButton, 0, 0);
    grid->addWidget(coordinateAxesCheck, 2, 0);
    grid->addWidget(new VSeparator, 0, 2, 3, 1);

    for(int i=0; i < 3; ++i){
        auto& gws = gridWidgetSets[i];
        grid->addWidget(gws.check, i, 4);
        grid->addWidget(new QLabel(_("Span")), i, 5);
        grid->addWidget(gws.spanSpin, i, 6);
        grid->addWidget(new QLabel(_("Interval")), i, 7);
        grid->addWidget(gws.intervalSpin, i, 8);
        grid->addWidget(gws.colorButton, i, 9);
    }

    grid->setColumnStretch(10, 10);

    return backgroundPanel;
}


QWidget* ConfigWidgetSet::createDrawingPanel()
{
    if(drawingPanel){
        return drawingPanel;
    }

    drawingPanel = new QWidget(ownerWidget);
    auto vbox = new QVBoxLayout;
    vbox->setContentsMargins(0, 0, 0, 0);
    drawingPanel->setLayout(vbox);
    QHBoxLayout* hbox;

    hbox = new QHBoxLayout;
    hbox->addWidget(defaultColorButton);
    hbox->addWidget(new QLabel(_("Default line width")));
    hbox->addWidget(lineWidthSpin);
    hbox->addWidget(new QLabel(_("Default point size")));
    hbox->addWidget(pointSizeSpin);
    hbox->addStretch();
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout;
    hbox->addWidget(normalVisualizationCheck);
    hbox->addWidget(normalLengthSpin);
    hbox->addWidget(lightweightViewChangeCheck);
    hbox->addWidget(fpsTestButton);
    hbox->addWidget(fpsTestIterationSpin);
    hbox->addStretch();
    vbox->addLayout(hbox);

    return drawingPanel;
}


QWidget* ConfigWidgetSet::createDebugPanel()
{
    if(debugPanel){
        return debugPanel;
    }
    debugPanel = new QWidget(ownerWidget);
    auto hbox = new QHBoxLayout;
    hbox->setContentsMargins(0, 0, 0, 0);
    hbox->addWidget(pickingImageButton);
    hbox->addStretch();
    debugPanel->setLayout(hbox);

    return debugPanel;
}


void ConfigWidgetSet::updateWidgets()
{
    for(auto& obj : signalObjects){
        obj->blockSignals(true);
    }
    
    fieldOfViewSpin->setValue(config->fieldOfView);
    nearClipSpin->setValue(config->nearClipDistance);
    farClipSpin->setValue(config->farClipDistance);
    restrictCameraRollCheck->setChecked(config->isCameraRollRistricted);
    verticalAxisRadios[config->verticalAxis]->setChecked(true);
    coordinateAxesCheck->setChecked(config->isCoordinateAxesEnabled);

    for(int i=0; i < 3; ++i){
        auto& gws = gridWidgetSets[i];
        auto& info = config->gridInfos[i];
        gws.check->setChecked(info.isEnabled);
        gws.spanSpin->setValue(info.span);
        gws.intervalSpin->setValue(info.interval);
    }
    
    normalVisualizationCheck->setChecked(config->isNormalVisualizationEnabled);
    normalLengthSpin->setValue(config->normalLength);
    lightweightViewChangeCheck->setChecked(config->isLightweightViewChangedEnabled);

    for(auto& obj : signalObjects){
        obj->blockSignals(false);
    }
}
