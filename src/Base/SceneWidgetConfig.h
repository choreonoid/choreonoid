#ifndef CNOID_BASE_SCENE_WIDGET_CONFIG_H
#define CNOID_BASE_SCENE_WIDGET_CONFIG_H

#include "SceneRendererConfig.h"
#include "exportdecl.h"

namespace cnoid {

class SceneWidget;

class CNOID_EXPORT SceneWidgetConfig : public SceneRendererConfig
{
public:
    ~SceneWidgetConfig();

    void setSceneWidget(SceneWidget* widget);
    void addSceneWidget(SceneWidget* widget);
    void clearSceneWidgets();
    
    virtual bool store(Mapping* archive) override;
    virtual bool restore(const Mapping* archive) override;

    class Impl;

protected:
    SceneWidgetConfig();
    SceneWidgetConfig(const SceneWidgetConfig& org);

    virtual void onRendererConfigUpdated(bool isInteractive) override;

    virtual void createConfigWidgets() override;
    QWidget* getOrCreateCameraPanel();
    QWidget* getOrCreateBackgroundPanel();
    QWidget* getOrCreateDrawingPanel();
    QWidget* getOrCreateDebugPanel();

private:
    Impl* impl;
};

}

#endif
