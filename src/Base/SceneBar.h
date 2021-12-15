/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_SCENE_BAR_H
#define CNOID_BASE_SCENE_BAR_H

#include <cnoid/ToolBar>
#include "exportdecl.h"

namespace cnoid {

class SceneWidget;
class SceneView;

class CNOID_EXPORT SceneBar : public ToolBar
{
public:
    static void initialize(ExtensionManager* ext);
    static SceneBar* instance();

    void addCustomModeButton(int modeId, const QIcon& icon, const QString& caption);
    void removeCustomModeButton(int modeId);

    SceneView* currentSceneView();
    
    [[deprecated("Use SceneBar::currentSceneView().")]]
    SceneWidget* targetSceneWidget();

protected:
    virtual bool restoreState(const Archive& archive);

private:
    SceneBar();
    ~SceneBar();

    class Impl;
    Impl* impl;
};

}

#endif
