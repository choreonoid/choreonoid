#ifndef CNOID_SCENE_GRAPH_PROPERTY_VIEW_H
#define CNOID_SCENE_GRAPH_PROPERTY_VIEW_H

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
