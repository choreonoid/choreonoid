/**
   @author Shizuko Hattori
*/

#ifndef CNOID_SCENE_GRAPH_VIEW_H
#define CNOID_SCENE_GRAPH_VIEW_H

#include <cnoid/View>
#include <cnoid/SceneGraph>

namespace cnoid {

class SceneGraphViewImpl;

class SceneGraphView : public View
{
public:
    static void initializeClass(ExtensionManager* ext);

    SceneGraphView();
    ~SceneGraphView();

    const SgObject* selectedObject();

    SignalProxy<void(const SgObject*)> sigSelectionChanged();

private:
    SceneGraphViewImpl* impl;
};

}

#endif
