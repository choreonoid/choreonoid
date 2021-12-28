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
    SceneViewConfig();
    SceneViewConfig(const SceneViewConfig& org);
    ~SceneViewConfig();

    void addSceneView(SceneView* view, bool doUpdateSceneView);
    void removeSceneView(SceneView* view);
    void clearSceneViews();

    virtual bool store(Archive* archive);
    virtual bool restore(const Archive* archive);
    virtual void updateSceneViews();
    virtual void showConfigDialog() override;

    class Impl;

protected:
    std::string getTargetSceneViewSetCaption();
    QWidget* getOrCreateViewOptionaPanel();

private:
    Impl* impl;
};

typedef ref_ptr<SceneViewConfig> SceneViewConfigPtr;

}

#endif
