#ifndef CNOID_BASE_SCENE_RENDERER_CONFIG_H
#define CNOID_BASE_SCENE_RENDERER_CONFIG_H

#include <cnoid/Referenced>
#include <cnoid/Signal>
#include <cnoid/EigenTypes>
#include <QWidget>
#include <QPushButton>
#include <string>
#include "exportdecl.h"

namespace cnoid {

class SceneRenderer;
class Mapping;
class Menu;
class PushButton;
class DoubleSpinBox;
class CheckBox;
class ComboBox;

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

    static int getSystemDefaultMsaaLevel();
    static void setSystemDefaultMsaaLevel(int level);
    static SignalProxy<void()> sigSystemDefaultMsaaLevelChanged();
    static void setMenuAsOpenGLMsaaLevelMenu(Menu* menu);

    static bool inputColorWithColorDialog(const std::string& title, Vector3f& io_color, QPushButton* button);
    static void setColorButtonColor(QPushButton* button, const Vector3f& color);

    class Impl;

protected:
    virtual void onRendererConfigUpdated(bool isInteractive);
    virtual void createConfigWidgets();
    virtual void updateConfigWidgets();
    
    QWidget* getOrCreateLightingPanel();
    PushButton* backgroundColorButton();
    PushButton* defaultColorButton();
    PushButton* collisionHighlightColorButton();
    PushButton* collisionLineColorButton();
    DoubleSpinBox* pointSizeSpin();
    DoubleSpinBox* lineWidthSpin();
    CheckBox* upsideDownCheck();
    ComboBox* msaaLevelCombo();

private:
    Impl* impl;
};

}

#endif
