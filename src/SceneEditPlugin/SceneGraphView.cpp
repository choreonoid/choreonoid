/**
   @author Shizuko Hattori
*/

#include "SceneGraphView.h"
#include <cnoid/SceneDrawables>
#include <cnoid/SceneCameras>
#include <cnoid/SceneLights>
#include <cnoid/SceneEffects>
#include <cnoid/SceneMarkers>
#include <cnoid/PolymorphicFunctionSet>
#include <cnoid/TreeWidget>
#include <cnoid/SceneView>
#include <cnoid/ViewManager>
#include <QBoxLayout>
#include <cassert>
#include <list>
#include "gettext.h"

using namespace std;
using namespace std::placeholders;
using namespace cnoid;

namespace {

class SgvMarkerItem;

class SgvItem : public QTreeWidgetItem
{
public:
    SgvItem(SgObject* node, QTreeWidget* widget=0) 
        : QTreeWidgetItem(widget), node(node), markerNode(0), markerItem_(0){ }
    ~SgvItem();
    inline SgvMarkerItem* markerItem() { return markerItem_; }
    inline void setMarkerItem(SgvMarkerItem* markerItem) { markerItem_ = markerItem; }

    SgObject* node;

    SgGroup* group;
    SgvItem* groupItem;
    SgNode* markerNode;
    SgvMarkerItem* markerItem_;
};

class SgvMarkerItem : public QTreeWidgetItem
{
public:
    SgvMarkerItem();
    ~SgvMarkerItem();

    SgLineSetPtr marker;
};

}


namespace cnoid {

class SceneGraphViewImpl : public TreeWidget
{
public:
    SceneGraphViewImpl(SceneGraphView* self, SgNode* sceneRoot);
    ~SceneGraphViewImpl();

    SceneGraphView* self;

    PolymorphicFunctionSet<SgNode> visitor;
    
    SgNode* sceneRoot;
    SgvItem* rootItem;
    SgvItem* parentItem;
    SgvItem* sgvItem;
    Connection connectionOfsceneUpdated;

    SgvItem* selectedSgvItem;
    SgObject* selectedSgObject;
    Signal<void(const SgObject*)> sigSelectionChanged;

    void createGraph();
    void createGraph(SgvItem* item, SgNode* node);
    void onActivated(bool on);
    void onSceneGraphUpdated(const SgUpdate& update);
    SgvItem* findItem(SgvItem* parent, SgObject* obj);
    SgNode* findAddNode(SgGroup* group);
    SgvItem* findRemoveItem(SgGroup* group);
    void removeItem(SgvItem* item);

    void visitObject(SgObject* obj);
    void visitNode(SgNode* node);
    void visitGroup(SgGroup* group);
    void visitShape(SgShape* shape);
    void visitPointSet(SgPointSet* pointSet);

    void onSelectionChanged();
    void addSelectedMarker();
    void updateSelectedMarker(SgLineSet* marker);
    void removeSelectedMarker();
    const SgObject* selectedObject();
};

}


SgvItem::~SgvItem()
{
    if(markerItem_)
        delete markerItem_;
}


SgvMarkerItem::SgvMarkerItem()
{
    marker = new SgLineSet();
    marker->setName("GraphViewMarker");
    marker->getOrCreateMaterial()->setDiffuseColor(Vector3f(1.0f, 0.0f, 1.0f));
    setFlags(Qt::NoItemFlags);
    setText(0, QString("GraphViewMarker (SgCustomGLNode)"));
}


SgvMarkerItem::~SgvMarkerItem()
{
    if(parent())
        ((SgvItem*)parent())->markerItem_ = 0;
}


void SceneGraphView::initializeClass(ExtensionManager* ext)
{
    ext->viewManager().registerClass<SceneGraphView>(
        "SceneGraphView", N_("Scene Graph"), ViewManager::SINGLE_OPTIONAL);
}


SceneGraphView::SceneGraphView()
{
    setDefaultLayoutArea(View::LEFT);
    
    impl = new SceneGraphViewImpl(this, SceneView::instance()->scene());

    QVBoxLayout* layout = new QVBoxLayout();
    layout->addWidget(impl);
    setLayout(layout);
}


SceneGraphView::~SceneGraphView()
{

}


SceneGraphViewImpl::SceneGraphViewImpl(SceneGraphView* self, SgNode* sceneRoot)
    : self(self),
      sceneRoot(sceneRoot)
{
    visitor.setFunction<SgNode>([&](SgNode* node){ visitNode(node); });
    visitor.setFunction<SgGroup>([&](SgNode* node){ visitGroup(static_cast<SgGroup*>(node)); });
    visitor.setFunction<SgShape>([&](SgNode* node){ visitShape(static_cast<SgShape*>(node)); });
    visitor.setFunction<SgPointSet>([&](SgNode* node){ visitPointSet(static_cast<SgPointSet*>(node)); });
    visitor.updateDispatchTable();
    
    setColumnCount(2);
    header()->setStretchLastSection(false);
    header()->setSectionResizeMode(0, QHeaderView::Stretch);
    header()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
    header()->swapSections(0, 1);
    setWordWrap(true);
    setFrameShape(QFrame::NoFrame);
    setHeaderHidden(true);
    setIndentation(12);
    setSelectionMode(QAbstractItemView::SingleSelection);

    self->sigActivated().connect(std::bind(&SceneGraphViewImpl::onActivated, this, true));
    self->sigDeactivated().connect(std::bind(&SceneGraphViewImpl::onActivated, this ,false));

    sigItemSelectionChanged().connect(std::bind(&SceneGraphViewImpl::onSelectionChanged, this));

    parentItem = rootItem = 0;
    visitNode(sceneRoot);

    selectedSgObject = 0;
    selectedSgvItem = 0;
}


SceneGraphViewImpl::~SceneGraphViewImpl()
{
    if(selectedSgvItem && selectedSgvItem->markerItem())
        selectedSgvItem->groupItem->removeChild(selectedSgvItem->markerItem());
}


void SceneGraphViewImpl::createGraph()
{
    createGraph(rootItem, sceneRoot);
}


void SceneGraphViewImpl::createGraph(SgvItem* item, SgNode* node)
{
    SgvItem* oldParent = parentItem;
    parentItem = item;
    list<SgvItem*> children; 
    for(int i=0; i<item->childCount(); i++)
        children.push_back((SgvItem*)item->child(i));

    SgGroup* group = dynamic_cast<SgGroup*>(node);
    if(group){
        for(const auto childNode : *group){
            SgvItem* item_ = findItem(parentItem, childNode);
            if(item_){
                createGraph(item_, childNode);
                children.remove(item_);
            }else{
                visitor.dispatch(childNode);
            }
        }
        for(list<SgvItem*>::iterator it = children.begin(); it != children.end(); it++){
            removeItem(*it);
        }
    }
    parentItem = oldParent;
}


void SceneGraphViewImpl::visitNode(SgNode* node)
{
    if(!parentItem){
        sgvItem = new SgvItem(node,this);
        sgvItem->group = dynamic_cast<SgGroup*>(node);
        sgvItem->groupItem = sgvItem;
        rootItem = sgvItem;
        parentItem = sgvItem;
    }else{
        sgvItem = new SgvItem(node);
        SgGroup* group_ = dynamic_cast<SgGroup*>(node);
        if(group_){
            sgvItem->group = group_;
            sgvItem->groupItem = sgvItem;
        }else{
            sgvItem->group = parentItem->group;
            sgvItem->groupItem = parentItem->groupItem;
        }
        parentItem->addChild(sgvItem);
    }

    string type;
    if(dynamic_cast<SgFog*>(node))
        type = "Fog";
    else if(dynamic_cast<SgOrthographicCamera*>(node))
        type = "SgOrthographicCamera";
    else if(dynamic_cast<SgPerspectiveCamera*>(node))
        type = "SgPerspectiveCamera";
    else if(dynamic_cast<SgCamera*>(node))
        type = "SgCamera";
    else if(dynamic_cast<SgSpotLight*>(node))
        type = "SgSpotLight";
    else if(dynamic_cast<SgPointLight*>(node))
        type = "SgPointLight";
    else if(dynamic_cast<SgDirectionalLight*>(node))
        type = "SgDirectionalLight";
    else if(dynamic_cast<SgLight*>(node))
        type = "SgLight";
    else if(dynamic_cast<SgLineSet*>(node))
        type = "SgLineSet";
    else if(dynamic_cast<SgPointSet*>(node))
        type = "SgPointSet";
    else if(dynamic_cast<SgPlot*>(node))
        type = "SgPlot";
    else if(dynamic_cast<SgShape*>(node))
        type = "SgShape";
    else if(dynamic_cast<SphereMarker*>(node))
        type = "SphereMarker";
    else if(dynamic_cast<SgPosTransform*>(node))
        type = "SgPosTransform";
    else if(dynamic_cast<SgScaleTransform*>(node))
        type = "SgScaleTransform";
    else if(dynamic_cast<SgTransform*>(node))
        type = "SgTransform";
    else if(dynamic_cast<SgInvariantGroup*>(node))
        type = "SgInvariantGroup";
    else if(dynamic_cast<BoundingBoxMarker*>(node))
        type = "BoundingBoxMarker";
    else if(dynamic_cast<SgGroup*>(node))
        type = "SgGroup";
    else if(dynamic_cast<CrossMarker*>(node))
        type = "CrossMarker";
    else if(dynamic_cast<SgNode*>(node))
        type = "SgNode";

    string name = node->name();
    name += " (" + type + ")"; 
    sgvItem->setText(0, QString(name.c_str()));
}


void SceneGraphViewImpl::visitGroup(SgGroup* group)
{
    visitNode(group);
    SgvItem* oldParent = parentItem;
    parentItem = sgvItem;
    for(const auto childNode : *group){
        visitor.dispatch(childNode);
    }
    parentItem = oldParent;
}


void SceneGraphViewImpl::visitObject(SgObject* obj)
{
    sgvItem = new SgvItem(obj);
    SgGroup* group_ = dynamic_cast<SgGroup*>(obj);
    if(group_){
        sgvItem->group = group_;
        sgvItem->groupItem = sgvItem;
    }else{
        sgvItem->group = parentItem->group;
        sgvItem->groupItem = parentItem->groupItem;
    }
    parentItem->addChild(sgvItem);

    string type;
    if(dynamic_cast<SgPolygonMesh*>(obj))
        type = "SgPolygonMesh";
    else if(dynamic_cast<SgMesh*>(obj))
        type = "SgMesh";
    else if(dynamic_cast<SgMeshBase*>(obj))
        type = "SgMeshBase";
    else if(dynamic_cast<SgTexture*>(obj)){
        sgvItem->markerNode = (SgNode*)parentItem->node;
        type = "SgTexture";
    }else if(dynamic_cast<SgTextureTransform*>(obj)){
        sgvItem->markerNode = (SgNode*)parentItem->node;
        type = "SgTextureTransform";
    }else if(dynamic_cast<SgMaterial*>(obj)){
        sgvItem->markerNode = (SgNode*)parentItem->node;
        type = "SgMaterial";
    }    else if(dynamic_cast<SgObject*>(obj))
        type = "SgObject";

    string name = obj->name();
    name += " (" + type + ")"; 
    sgvItem->setText(0, QString(name.c_str()));
}


void SceneGraphViewImpl::visitShape(SgShape* shape)
{
    visitNode(shape);
    SgvItem* oldParent = parentItem;
    parentItem = sgvItem;
    if(shape->material())
        visitObject(shape->material());
    if(shape->mesh())
        visitObject(shape->mesh());
    if(shape->texture())
        visitObject(shape->texture());
    parentItem = oldParent;
}


void SceneGraphViewImpl::visitPointSet(SgPointSet* pointSet)
{
    visitNode(pointSet);
    SgvItem* oldParent = parentItem;
    parentItem = sgvItem;
    if(pointSet->material())
        visitObject(pointSet->material());
    parentItem = oldParent;
}


void SceneGraphViewImpl::onActivated(bool on)
{
    if(on){
        createGraph();
        connectionOfsceneUpdated =
            sceneRoot->sigUpdated().connect(
                std::bind(&SceneGraphViewImpl::onSceneGraphUpdated, this, _1));
    } else {
        connectionOfsceneUpdated.disconnect();
    }
}


SgNode* SceneGraphViewImpl::findAddNode(SgGroup* group)
{
    for(SgGroup::const_iterator it = group->begin(); it != group->end(); it++){
        SgNode* node = it->get();
        if(!findItem(parentItem, node))
            return node;
    }
    return 0;
}


SgvItem* SceneGraphViewImpl::findRemoveItem(SgGroup* group)
{
    int num = parentItem->childCount();
    for(int i=0; i<num; i++){
        SgvItem* childItem = (SgvItem*)parentItem->child(i);
        if(!group->contains((SgNode*)childItem->node))
            return childItem;
    }
    return 0;
}


SgvItem* SceneGraphViewImpl::findItem(SgvItem* parent, SgObject* obj)
{
    int num = parent->childCount();
    for(int i=0; i<num; i++){
        SgvItem* childItem = (SgvItem*)parent->child(i);
        if( obj == childItem->node )
            return childItem;
    }
    return 0;
}


void SceneGraphViewImpl::onSceneGraphUpdated(const SgUpdate& update)
{
    if(update.action() & (SgUpdate::ADDED)){
        parentItem = rootItem;
        SgUpdate::Path path = update.path();
        if(path.size() > 1){
            for(SgUpdate::Path::reverse_iterator it = ++path.rbegin(); it != path.rend(); ++it){
                parentItem = findItem(parentItem, *it);
                if(!parentItem)
                    return;
            }
        }

        SgObject* parent = update.path().front();
        SgGroup* group = dynamic_cast<SgGroup*>(parent);
        if(group){
            SgNode* node = findAddNode(group);
            if(node){
                visitor.dispatch(node);
            }
        }
    }
    if(update.action() & (SgUpdate::REMOVED)){
        parentItem = rootItem;
        SgUpdate::Path path = update.path();
        if(path.size() > 1){
            for(SgUpdate::Path::reverse_iterator it = ++path.rbegin(); it != path.rend(); ++it){
                parentItem = findItem(parentItem, *it);
                if(!parentItem)
                    return;
            }
        }

        SgObject* parent = update.path().front();
        SgGroup* group = dynamic_cast<SgGroup*>(parent);
        if(group){
            SgvItem* item = findRemoveItem(group);
            if(item){
                removeItem(item);
            }
        }
    }
}


void SceneGraphViewImpl::removeItem(SgvItem* item)
{
    int num = item->childCount();
    if(num){
        SgvItem* oldParent = parentItem;
        parentItem = item;
        SgvItem* child = (SgvItem*)item->child(0);
        while(child){
            removeItem(child);
            child = (SgvItem*)item->child(0);
        }
        parentItem = oldParent;
    }
    parentItem->removeChild(item);
    if(item->node->name()!="GraphViewMarker")
        delete item;
}



void SceneGraphViewImpl::addSelectedMarker()
{
    if(!selectedSgvItem->markerItem()){
        selectedSgvItem->setMarkerItem( new SgvMarkerItem() );
    }
    auto marker = selectedSgvItem->markerItem()->marker;
    if(marker && !selectedSgvItem->group->contains(marker)){
        selectedSgvItem->group->addChild(marker);
        selectedSgvItem->groupItem->addChild(selectedSgvItem->markerItem());
        updateSelectedMarker(marker);
    }
}


void SceneGraphViewImpl::updateSelectedMarker(SgLineSet* marker)
{
    SgNode* node = 0;
    SgTransform* trans = 0;
    SgMeshBase* mesh = 0;

    if(selectedSgvItem->markerNode){
        node = selectedSgvItem->markerNode;
    }else if(dynamic_cast<SgScaleTransform*>(selectedSgvItem->node) ||
             dynamic_cast<SgPosTransform*>(selectedSgvItem->node)){
        trans = dynamic_cast<SgTransform*>(selectedSgvItem->node);
    }else{
        mesh = dynamic_cast<SgMeshBase*>(selectedSgvItem->node);
        if(!mesh)
            node = dynamic_cast<SgNode*>(selectedSgvItem->node);
    }

    BoundingBoxf bb;
    if(trans)
        bb = trans->untransformedBoundingBox();
    else if(mesh){
        bb = mesh->boundingBox();
    }else if(node){
        bb = node->boundingBox();
    }else
        return;
    if(bb.empty())
        return;

    SgVertexArray& vertices = *marker->getOrCreateVertices();
    vertices.clear();

    const Vector3f& min = bb.min();
    const Vector3f& max = bb.max();

    float y[2];
    y[0] = min.y();
    y[1] = max.y();
            
    for(int i=0; i < 2; ++i){
        vertices.push_back(Vector3f(min.x(), y[i], min.z()));
        vertices.push_back(Vector3f(min.x(), y[i], max.z()));
        vertices.push_back(Vector3f(min.x(), y[i], max.z()));
        vertices.push_back(Vector3f(max.x(), y[i], max.z()));

        vertices.push_back(Vector3f(max.x(), y[i], max.z()));
        vertices.push_back(Vector3f(max.x(), y[i], min.z()));
        vertices.push_back(Vector3f(max.x(), y[i], min.z()));
        vertices.push_back(Vector3f(min.x(), y[i], min.z()));
    }

    float z[2];
    z[0] = min.z();
    z[1] = max.z();
    for(int i=0; i < 2; ++i){
        vertices.push_back(Vector3f(min.x(), min.y(), z[i]));
        vertices.push_back(Vector3f(min.x(), max.y(), z[i]));
        vertices.push_back(Vector3f(max.x(), min.y(), z[i]));
        vertices.push_back(Vector3f(max.x(), max.y(), z[i]));
    }

    const int n = vertices.size();
    SgIndexArray& lineVertices = marker->lineVertices();
    lineVertices.resize(n);
    for(int i=0; i < n; ++i){
        lineVertices[i] = i;
    }

    marker->notifyUpdate();
}


void SceneGraphViewImpl::removeSelectedMarker()
{
    if(selectedSgvItem){
        SgvMarkerItem* markerItem_ = selectedSgvItem->markerItem();
        if(markerItem_){
            selectedSgvItem->group->removeChild(markerItem_->marker);
            selectedSgvItem->groupItem->removeChild(markerItem_);
            selectedSgvItem->group->notifyUpdate();
        }
    }
}


void SceneGraphViewImpl::onSelectionChanged()
{
    removeSelectedMarker();

    selectedSgObject = 0;

    QList<QTreeWidgetItem*> selected = selectedItems();
    for(int i=0; i < selected.size(); ++i){
        selectedSgvItem = dynamic_cast<SgvItem*>(selected[i]);
        if(selectedSgvItem){
            addSelectedMarker();
            selectedSgObject = selectedSgvItem->node;
            sigSelectionChanged(selectedSgObject);
        }
    }
}


const SgObject* SceneGraphView::selectedObject()
{
    return impl->selectedObject();
}


const SgObject* SceneGraphViewImpl::selectedObject()
{
    return selectedSgObject;
}


SignalProxy<void(const SgObject*)> SceneGraphView::sigSelectionChanged()
{
    return impl->sigSelectionChanged;
}
