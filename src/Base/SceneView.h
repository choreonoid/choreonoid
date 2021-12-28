/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_SCENE_VIEW_H
#define CNOID_BASE_SCENE_VIEW_H

#include "View.h"
#include "exportdecl.h"

namespace cnoid {

class SceneViewConfig;
class SceneWidget;
class SceneWidgetEventHandler;
class SceneRenderer;
class SgGroup;

class CNOID_EXPORT SceneView : public View
{
public:
    static void initializeClass(ExtensionManager* ext);

    //! This function returns the default instance
    static SceneView* instance();

    /**
       This function is used in the BodySyncCameraItem implementation, but
       the implementation should not use the function and the function should be removed.
    */
    static std::vector<SceneView*> instances();

    /**
       If you want to add a custom mode button to the scene bar, use the SceneBar::addCustomModeButton function.
       \return Mode id
    */
    static int registerCustomMode(SceneWidgetEventHandler* modeHandler);

    /**
       If the corresponding custom mode button is added to the scene bar, remove it with
       the SceneBar::removeCustomModeButton function.
    */
    static void unregisterCustomMode(int mode);
    
    static int customModeId(const std::string& modeName);

    static void blockEditModeForAllViews(Referenced* requester);
    static void unblockEditModeForAllViews(Referenced* requester);

    static SignalProxy<void(SceneView* view)> sigLastFocusSceneViewChanged();
    static SceneView* lastFocusSceneView();
        
    SceneView();
    ~SceneView();
        
    SceneViewConfig* sceneViewConfig();
    SceneWidget* sceneWidget();
    SceneRenderer* renderer();
    SgGroup* scene();

    bool setCustomMode(int mode);
    int customMode() const;

    void setTargetSceneItemCheckId(int checkId);

protected:
    struct NoSceneViewConfig_t { };
    static constexpr NoSceneViewConfig_t NoSceneViewConfig = { };
    SceneView(NoSceneViewConfig_t);

    void setSceneViewConfig(SceneViewConfig* config);
    
    virtual void onFocusChanged(bool on) override;
    virtual QWidget* indicatorOnInfoBar() override;
    virtual bool storeState(Archive& archive) override;
    virtual bool restoreState(const Archive& archive) override;
    virtual void onRestored(bool stateRestored) override;
        
private:
    class Impl;
    Impl* impl;
};

}

#endif
