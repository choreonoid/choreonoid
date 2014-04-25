/**
   @author Shizuko Hattori
*/

#ifndef CNOID_SCENE_GRAPH_PROPERTY_VIEW_H_INCLUDED 
#define CNOID_SCENE_GRAPH_PROPERTY_VIEW_H_INCLUDED

#include <cnoid/View>

namespace cnoid {

class SceneGraphPropertyViewImpl;

class SceneGraphPropertyView : public View
{
public:
    static void initializeClass(ExtensionManager* ext);

    SceneGraphPropertyView();
    ~SceneGraphPropertyView();

private:
    SceneGraphPropertyViewImpl* impl;

    void keyPressEvent(QKeyEvent* event);
};

}

#endif
