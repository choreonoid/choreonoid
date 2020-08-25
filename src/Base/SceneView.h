/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_SCENE_VIEW_H
#define CNOID_BASE_SCENE_VIEW_H

#include "View.h"
#include "exportdecl.h"

namespace cnoid {

class SceneWidget;
class SceneWidgetEditable;
class SceneBar;
class SgGroup;
class Item;

class CNOID_EXPORT SceneView : public View
{
public:
    static void initializeClass(ExtensionManager* ext);

    //! This function returns the default instance
    static SceneView* instance();

    /**
       This function is used in the BodyTrackingCameraItem implementation, but
       the implementation should not use the function and the function should be removed.
    */
    static std::vector<SceneView*> instances();

    //! \return Mode id
    static int registerCustomMode(
        SceneWidgetEditable* modeHandler, const QIcon& buttonIcon, const QString& caption);
    static void unregisterCustomMode(int mode);
    static int customModeId(const std::string& modeName);
        
    SceneView();
    ~SceneView();
        
    SceneWidget* sceneWidget();
    SgGroup* scene();

    bool setCustomMode(int mode);
    int customMode() const;
    SignalProxy<void(int mode)> sigCustomModeChanged();
        
protected:
    virtual QWidget* indicatorOnInfoBar() override;
    virtual bool storeState(Archive& archive) override;
    virtual bool restoreState(const Archive& archive) override;
        
private:
    static void onItemAdded(Item* item);

    class Impl;
    Impl* impl;
};

}

#endif
