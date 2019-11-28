/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_SCENE_VIEW_H
#define CNOID_BASE_SCENE_VIEW_H

#include "View.h"
#include "exportdecl.h"

namespace cnoid {

class SceneWidget;
class SgGroup;
class Item;
class SceneViewImpl;

class CNOID_EXPORT SceneView : public View
{
public:
    static void initializeClass(ExtensionManager* ext);
    static SceneView* instance();
        
    SceneView();
    ~SceneView();
        
    SceneWidget* sceneWidget();
    SgGroup* scene();
        
protected:
    virtual void onActivated();
    virtual void onDeactivated();
    virtual QWidget* indicatorOnInfoBar();
    virtual bool storeState(Archive& archive);
    virtual bool restoreState(const Archive& archive);
        
private:
    static void onItemAdded(Item* item);

    SceneViewImpl* impl;
    friend class SceneViewImpl;
};

}

#endif
