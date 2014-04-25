/**
   @author Shizuko Hattori
*/

#ifndef CNOID_SCENE_GRAPH_VIEW_H_INCLUDED
#define CNOID_SCENE_GRAPH_VIEW_H_INCLUDED

#include <cnoid/View>
#include <cnoid/SceneGraph>
#include <cnoid/SignalProxy>

namespace cnoid {

class SceneGraphViewImpl;

class SceneGraphView : public View
{
public:
    static void initializeClass(ExtensionManager* ext);

    SceneGraphView();
    ~SceneGraphView();

    const SgObject* selectedObject();

    SignalProxy< boost::signal<void(const SgObject*)> > sigSelectionChanged();

private:
    SceneGraphViewImpl* impl;

};

}

#endif
