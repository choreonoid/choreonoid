#include "RenderableItemSceneStatistics.h"
#include "RenderableItem.h"
#include "RootItem.h"
#include "ItemList.h"
#include "MessageView.h"
#include <cnoid/SceneDrawables>
#include <cnoid/PolymorphicSceneNodeFunctionSet>
#include <fmt/format.h>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

namespace {

class SceneCounter : public PolymorphicSceneNodeFunctionSet
{
public:
    int numVertices;
    int numNormals;
    int numTriangles;

    SceneCounter() {
        setFunction<SgGroup>(
            [&](SgGroup* group){
                for(auto child : *group){
                    dispatch(child);
                }
            });

        setFunction<SgShape>(
            [&](SgShape* shape){
                SgMesh* mesh = shape->mesh();
                if(mesh){
                    if(mesh->hasVertices()){
                        numVertices += mesh->vertices()->size();
                    }
                    if(mesh->hasNormals()){
                        numNormals += mesh->normals()->size();
                    }
                    numTriangles += mesh->numTriangles();
                }
            });

        setFunction<SgPointSet>(
            [&](SgPointSet* pointSet){
                if(pointSet->hasVertices()){
                    numVertices += pointSet->vertices()->size();
                }
            });

        updateDispatchTable();
    }
    
    void count(SgNode* node) {
        numVertices = 0;
        numNormals = 0;
        numTriangles = 0;
        dispatch(node);
    }
};

}

void cnoid::putRenderableItemSceneStatistics()
{
    ostream& os = MessageView::instance()->cout();
    os << _("Scene statistics:") << endl;
    
    int numSceneItems = 0;
    int totalNumVertics = 0;
    int totalNumNormals = 0;
    int totalNumTriangles = 0;

    SceneCounter counter;

    for(auto& item : RootItem::instance()->selectedItems()){
        if(auto renderable = dynamic_cast<RenderableItem*>(item.get())){
            if(auto scene = renderable->getScene()){
                os << format(_(" Scene \"{}\":"), item->displayName()) << endl;
                counter.count(scene);
                os << format(_("  Vertices: {}\n"), counter.numVertices);
                os << format(_("  Normals: {}\n"), counter.numNormals);
                os << format(_("  Triangles: {}"), counter.numTriangles) << endl;
                totalNumVertics += counter.numVertices;
                totalNumNormals += counter.numNormals;
                totalNumTriangles += counter.numTriangles;
                ++numSceneItems;
            }
        }
    }

    if(!numSceneItems){
        os << _("No valid scene item is selected.") << endl;
    } else {
        os << format(_("The total number of {} scene items:\n"), numSceneItems);
        os << format(_(" Vertices: {}\n"), totalNumVertics);
        os << format(_(" Normals: {}\n"), totalNumNormals);
        os << format(_(" Triangles: {}"), totalNumTriangles) << endl;
    }
}
