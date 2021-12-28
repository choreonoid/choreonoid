#ifndef CNOID_BASE_SCENE_RENDERER_CONFIG_H
#define CNOID_BASE_SCENE_RENDERER_CONFIG_H

#include <cnoid/Referenced>
#include <cnoid/EigenTypes>
#include <QWidget>
#include <string>
#include "exportdecl.h"

namespace cnoid {

class SceneRenderer;
class Mapping;
class PushButton;
class DoubleSpinBox;
class CheckBox;

class CNOID_EXPORT SceneRendererConfig : public Referenced
{
public:
    SceneRendererConfig();
    SceneRendererConfig(const SceneRendererConfig& org);
    ~SceneRendererConfig();

    void addRenderer(SceneRenderer* renderer, bool doUpdateRenderer);
    void removeRenderer(SceneRenderer* renderer);
    void clearRenderers();
    void updateRenderers();
    
    virtual bool store(Mapping* archive);
    virtual bool restore(const Mapping* archive);

    virtual void showConfigDialog();

    static bool inputColorWithColorDialog(const std::string& title, Vector3f& io_color);

    class Impl;

protected:
    virtual void onRendererConfigUpdated(bool isInteractive);
    virtual void createConfigWidgets();
    virtual void updateConfigWidgets();
    
    QWidget* getOrCreateLightingPanel();
    PushButton* backgroundColorButton();
    PushButton* defaultColorButton();
    DoubleSpinBox* pointSizeSpin();
    DoubleSpinBox* lineWidthSpin();
    CheckBox* upsideDownCheck();

private:
    Impl* impl;
};

}

#endif
