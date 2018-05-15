/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_SCENE_BAR_H
#define CNOID_BASE_SCENE_BAR_H

#include <cnoid/ToolBar>
#include "exportdecl.h"

namespace cnoid {

class SceneWidget;
class SceneBarImpl;

class CNOID_EXPORT SceneBar : public ToolBar
{
public:
    static void initialize(ExtensionManager* ext);
    static SceneBar* instance();

    SceneWidget* targetSceneWidget();
    void sceneWidget(std::vector<SceneWidget*>& widget);

    bool isCollisionVisualizationButtonSetVisible() const;
    void setCollisionVisualizationButtonSetVisible(bool on);

protected:
    SceneBar();

private:
    ~SceneBar();
    SceneBarImpl* impl;
};

}

#endif
