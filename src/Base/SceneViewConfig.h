#ifndef CNOID_BASE_SCENE_VIEW_CONFIG_H
#define CNOID_BASE_SCENE_VIEW_CONFIG_H

#include "SceneWidgetConfig.h"
#include "exportdecl.h"

namespace cnoid {

class SceneView;
class Archive;

class CNOID_EXPORT SceneViewConfig : public SceneWidgetConfig
{
public:
    SceneViewConfig(SceneView* view = nullptr);
    SceneViewConfig(const SceneViewConfig& org, SceneView* view = nullptr);
    ~SceneViewConfig();

    void setSceneView(SceneView* view);
    void addSceneView(SceneView* view);
    void clearSceneViews();

    virtual bool store(Archive* archive);
    virtual bool restore(const Archive* archive);
    virtual void showConfigDialog() override;

    class Impl;

protected:
    QWidget* getOrCreateViewOptionaPanel();

private:
    Impl* impl;
};

}

#endif
