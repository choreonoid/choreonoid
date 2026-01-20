#ifndef CNOID_BASE_SCENE_WIDGET_CONFIG_H
#define CNOID_BASE_SCENE_WIDGET_CONFIG_H

#include "SceneRendererConfig.h"
#include "DisplayValueFormat.h"
#include <optional>
#include "exportdecl.h"

namespace cnoid {

class SceneWidget;
class PushButton;

class CNOID_EXPORT SceneWidgetConfig : public SceneRendererConfig
{
public:
    //! Fix length unit for Near/Far and grid settings (pass nullopt to follow DisplayValueFormat)
    static void setFixedLengthUnit(std::optional<DisplayValueFormat::LengthUnit> unit, int decimals);

    ~SceneWidgetConfig();

    void addSceneWidget(SceneWidget* widget, bool doUpdateSceneWidget);
    void removeSceneWidget(SceneWidget* widget);
    void clearSceneWidgets();
    void updateSceneWidgets();
    
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

    PushButton* fpsTestButton();

private:
    Impl* impl;
};

}

#endif
