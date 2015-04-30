/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "PointSetItem.h"
#include <cnoid/ItemManager>
#include <cnoid/MenuManager>
#include <cnoid/Archive>
#include <cnoid/EigenArchive>
#include <cnoid/SceneWidget>
#include <cnoid/SceneWidgetEditable>
#include <cnoid/PointSetUtil>
#include <cnoid/SceneMarker>
#include <cnoid/Exception>
#include <cnoid/FileUtil>
#include <cnoid/GLSceneRenderer>
#include <cnoid/SceneCamera>
#include <boost/bind.hpp>
#include <boost/dynamic_bitset.hpp>
#include "gettext.h"

using namespace std;
using namespace boost;
using namespace cnoid;

namespace {

class ScenePointSet;

typedef Signal<bool(int editType, const PointSetItem::Region& region), LogicalProduct> SigRegionFixed;
typedef SignalProxy<bool(int editType, const PointSetItem::Region& region), LogicalProduct> SigRegionFixedProxy;

struct RegionImpl
{
    RegionImpl() { }
    RegionImpl(int n) : normals(n), points(n) { }
    vector<Vector3> normals;
    vector<Vector3> points;
};
    

enum RenderingMode {
    POINT_MODE, VOXEL_MODE, N_RENDERING_MODES
};

class ScenePointSet : public SgPosTransform, public SceneWidgetEditable
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    weak_ref_ptr<PointSetItem> weakPointSetItem;
    SgPointSetPtr orgPointSet;
    SgPointSetPtr visiblePointSet;
    SgShapePtr voxels;
    float voxelSize;
    SgInvariantGroupPtr invariant;
    Selection renderingMode;
    bool isEditable_;

    Signal<void(const Affine3& T)> sigOffsetPositionChanged;
    
    Signal<void()> sigAttentionPointsChanged;
    SgGroupPtr attentionPointMarkerGroup;
    
    SigRegionFixed sigRegionFixed;

    ScenePointSet(PointSetItemImpl* pointSetItem);

    bool setRenderingMode(int mode);
    void setPointSize(double size);
    void setVoxelSize(double size);
    int numAttentionPoints() const;
    Vector3 attentionPoint(int index) const;
    void clearAttentionPoints(bool doNotify);
    void addAttentionPoint(const Vector3& point, bool doNotify);
    void setAttentionPoint(const Vector3& p, bool doNotify);
    bool removeAttentionPoint(const Vector3& point, double distanceThresh, bool doNotify);
    void notifyAttentionPointChange();
    void updateVisualization(bool updateContents);
    void updateVisiblePointSet();
    void updateVoxels();
    bool isEditable() const { return isEditable_; }
    void setEditable(bool on) { isEditable_ = on; }

    virtual bool onButtonPressEvent(const SceneWidgetEvent& event);
    virtual bool onPointerMoveEvent(const SceneWidgetEvent& event);
    virtual void onContextMenuRequest(const SceneWidgetEvent& event, MenuManager& menuManager);
    virtual void onSceneModeChanged(const SceneWidgetEvent& event);

    void startEraserMode(SceneWidget* sceneWidget);
};

typedef ref_ptr<ScenePointSet> ScenePointSetPtr;


class RectLineOverlay : public SgOverlay
{
public:
    int left, right, top, bottom;
    SgVertexArrayPtr vertices;
    RectLineOverlay();
    virtual void calcViewVolume(double viewportWidth, double viewportHeight, ViewVolume& io_volume);
    void setRect(int x0, int y0, int x1, int y1);
};

typedef ref_ptr<RectLineOverlay> RectLineOverlayPtr;


class PointSetEraser : public SceneWidgetEditable, public Referenced
{
public:
    ScenePointSet* scenePointSet;
    weak_ref_ptr<PointSetItem> weakPointSetItem;
    SceneWidget* sceneWidget;
    RectLineOverlayPtr rect;
    int x0, y0;

    PointSetEraser(ScenePointSet* scenePointSet, weak_ref_ptr<PointSetItem>& weakPointSetItem, SceneWidget* sceneWidget);
    ~PointSetEraser();
    void showRectangle(bool on);
    virtual void onSceneModeChanged(const SceneWidgetEvent& event);
    virtual bool onButtonPressEvent(const SceneWidgetEvent& event);
    virtual bool onButtonReleaseEvent(const SceneWidgetEvent& event);
    virtual bool onPointerMoveEvent(const SceneWidgetEvent& event);
    virtual void onContextMenuRequest(const SceneWidgetEvent& event, MenuManager& menuManager);
};


}


namespace cnoid {

class PointSetItemImpl
{
public:
    PointSetItem* self;
    SgPointSetPtr pointSet;
    ScenePointSetPtr scenePointSet;
    ScopedConnection pointSetUpdateConnection;

    PointSetItemImpl(PointSetItem* self);
    PointSetItemImpl(PointSetItem* self, const PointSetItemImpl& org);
    bool onEditableChanged(bool on);
    void removePoints(const PointSetItem::Region& region);
    template<class ElementContainer>
    void removeSubElements(ElementContainer& elements, SgIndexArray& indices, const vector<int>& indicesToRemove);
    bool onTranslationPropertyChanged(const std::string& value);
    bool onRotationPropertyChanged(const std::string& value);
};

}


static bool loadPCD(PointSetItem* item, const std::string& filename, std::ostream& os)
{
    try {
        cnoid::loadPCD(item->pointSet(), filename);
        os << item->pointSet()->vertices()->size() << " points have been loaded.";
        item->pointSet()->notifyUpdate();
        return true;
    } catch (boost::exception& ex) {
        if(std::string const * message = boost::get_error_info<error_info_message>(ex)){
            os << *message;
        }
    }
    return false;
}


static bool saveAsPCD(PointSetItem* item, const std::string& filename, std::ostream& os)
{
    try {
        cnoid::savePCD(item->pointSet(), filename, item->offsetPosition());
        return true;
    } catch (boost::exception& ex) {
        if(std::string const * message = boost::get_error_info<error_info_message>(ex)){
            os << *message;
        }
    }
    return false;
}


void PointSetItem::initializeClass(ExtensionManager* ext)
{
    static bool initialized = false;
    if(!initialized){
        ItemManager& im = ext->itemManager();
        im.registerClass<PointSetItem>(N_("PointSetItem"));
        im.addCreationPanel<PointSetItem>();
        im.addLoaderAndSaver<PointSetItem>(
            _("Point Cloud (PCD)"), "PCD-FILE", "pcd",
            boost::bind(::loadPCD, _1, _2, _3),
            boost::bind(::saveAsPCD, _1, _2, _3),
            ItemManager::PRIORITY_CONVERSION);
        
        initialized = true;
    }
}


PointSetItem::PointSetItem()
{
    impl = new PointSetItemImpl(this);
    initialize();
}


PointSetItemImpl::PointSetItemImpl(PointSetItem* self)
    : self(self)
{
    pointSet = new SgPointSet;
    scenePointSet = new ScenePointSet(this);
}


PointSetItem::PointSetItem(const PointSetItem& org)
    : Item(org)
{
    impl = new PointSetItemImpl(this, *org.impl);
    initialize();
}


PointSetItemImpl::PointSetItemImpl(PointSetItem* self, const PointSetItemImpl& org)
    : self(self)
{
    pointSet = new SgPointSet(*org.pointSet);
    scenePointSet = new ScenePointSet(this);
    scenePointSet->T() = org.scenePointSet->T();
}


void PointSetItem::initialize()
{
    impl->pointSetUpdateConnection.reset(
        impl->pointSet->sigUpdated().connect(
            boost::bind(&PointSetItem::notifyUpdate, this)));
}


PointSetItem::~PointSetItem()
{
    delete impl;
}


ItemPtr PointSetItem::doDuplicate() const
{
    return new PointSetItem(*this);
}


void PointSetItem::setName(const std::string& name)
{
    impl->scenePointSet->setName(name);
    impl->pointSet->setName(name);
    Item::setName(name);
}


SgNode* PointSetItem::getScene()
{
    return impl->scenePointSet;
}


void PointSetItem::notifyUpdate()
{
    impl->scenePointSet->updateVisualization(true);
    Item::notifyUpdate();
}


const SgPointSet* PointSetItem::pointSet() const
{
    return impl->pointSet;
}


SgPointSet* PointSetItem::pointSet()
{
    return impl->pointSet;
}


const Affine3& PointSetItem::offsetPosition() const
{
    return impl->scenePointSet->T();
}


void PointSetItem::setOffsetPosition(const Affine3& T)
{
    impl->scenePointSet->setPosition(T);
}


SignalProxy<void(const Affine3& T)> PointSetItem::sigOffsetPositionChanged()
{
    return impl->scenePointSet->sigOffsetPositionChanged;
}


void PointSetItem::notifyOffsetPositionChange()
{
    impl->scenePointSet->sigOffsetPositionChanged(impl->scenePointSet->T());
    impl->scenePointSet->notifyUpdate();
    Item::notifyUpdate();
}


double PointSetItem::pointSize() const
{
    return impl->scenePointSet->visiblePointSet->pointSize();
}
    

void PointSetItem::setPointSize(double size)
{
    impl->scenePointSet->setPointSize(size);
}


double PointSetItem::voxelSize() const
{
    return impl->scenePointSet->voxelSize;
}
    

void PointSetItem::setVoxelSize(double size)
{
    impl->scenePointSet->setVoxelSize(size);
}


void PointSetItem::setEditable(bool on)
{
    impl->scenePointSet->setEditable(on);
}


bool PointSetItem::isEditable() const
{
    return impl->scenePointSet->isEditable();
}


bool PointSetItemImpl::onEditableChanged(bool on)
{
    scenePointSet->setEditable(on);
    return true;
}


int PointSetItem::numAttentionPoints() const
{
    return impl->scenePointSet->numAttentionPoints();
}


Vector3 PointSetItem::attentionPoint(int index) const
{
    return impl->scenePointSet->attentionPoint(index);
}
        
    
void PointSetItem::clearAttentionPoints()
{
    impl->scenePointSet->clearAttentionPoints(false);
}


void PointSetItem::addAttentionPoint(const Vector3& p)
{
    impl->scenePointSet->addAttentionPoint(p, false);
}


SignalProxy<void()> PointSetItem::sigAttentionPointsChanged()
{
    return impl->scenePointSet->sigAttentionPointsChanged;
}


boost::optional<Vector3> PointSetItem::attentionPoint() const
{
    if(numAttentionPoints() == 1){
        return attentionPoint(0);
    }
    return boost::none;
}


void PointSetItem::clearAttentionPoint()
{
    impl->scenePointSet->clearAttentionPoints(false);
}


void PointSetItem::setAttentionPoint(const Vector3& p)
{
    impl->scenePointSet->setAttentionPoint(p, false);
}


SignalProxy<void()> PointSetItem::sigAttentionPointChanged()
{
    return impl->scenePointSet->sigAttentionPointsChanged;
}


void PointSetItem::notifyAttentionPointChange()
{
    impl->scenePointSet->notifyAttentionPointChange();
}


PointSetItem::Region::Region()
{
    impl = new RegionImpl;
}


PointSetItem::Region::Region(int numSurroundingPlanes)
{
    impl = new RegionImpl(numSurroundingPlanes);
}
    

PointSetItem::Region::Region(const PointSetItem::Region& org)
{
    RegionImpl* p = new RegionImpl;
    RegionImpl* orgImpl = (RegionImpl*)(org.impl);
    p->normals = orgImpl->normals;
    p->points = orgImpl->points;
    impl = p;
}


PointSetItem::Region& PointSetItem::Region::operator=(const PointSetItem::Region& org)
{
    RegionImpl* p = (RegionImpl*)impl;
    RegionImpl* orgImpl = (RegionImpl*)(org.impl);
    p->normals = orgImpl->normals;
    p->points = orgImpl->points;
    return *this;
}


void PointSetItem::Region::setNumSurroundingPlanes(int n)
{
    RegionImpl* p = (RegionImpl*)impl;
    p->normals.resize(n);
    p->points.resize(n);
}


int PointSetItem::Region::numSurroundingPlanes() const
{
    RegionImpl* p = (RegionImpl*)impl;
    return p->normals.size();
}

    
void PointSetItem::Region::addSurroundingPlane(const Vector3& normal, const Vector3& point)
{
    RegionImpl* p = (RegionImpl*)impl;
    p->normals.push_back(normal);
    p->points.push_back(point);
}


Vector3& PointSetItem::Region::normal(int index)
{
    RegionImpl* p = (RegionImpl*)impl;
    return p->normals[index];
}


const Vector3& PointSetItem::Region::normal(int index) const
{
    RegionImpl* p = (RegionImpl*)impl;
    return p->normals[index];
}


Vector3& PointSetItem::Region::point(int index)
{
    RegionImpl* p = (RegionImpl*)impl;
    return p->points[index];
}


const Vector3& PointSetItem::Region::point(int index) const
{
    RegionImpl* p = (RegionImpl*)impl;
    return p->points[index];
}


SigRegionFixedProxy PointSetItem::sigRegionFixed()
{
    return impl->scenePointSet->sigRegionFixed;
}


void PointSetItem::removePoints(const PointSetItem::Region& region)
{
    impl->removePoints(region);
}


void PointSetItemImpl::removePoints(const PointSetItem::Region& region)
{
    vector<int> indicesToRemove;
    const Affine3 T = scenePointSet->T();
    SgVertexArray orgPoints(*pointSet->vertices());
    const int numOrgPoints = orgPoints.size();

    const int numPlanes = region.numSurroundingPlanes();
    vector<double> d(numPlanes);
    for(int i=0; i < numPlanes; ++i){
        d[i] = region.normal(i).dot(region.point(i));
    }

    for(int i=0; i < numOrgPoints; ++i){
        const Vector3 point = T * orgPoints[i].cast<Vector3::Scalar>();
        for(int j=0; j < numPlanes; ++j){
            const double distance = point.dot(region.normal(j)) - d[j];
            if(distance < 0.0){
                goto notInRegion;
            }
        }
        indicesToRemove.push_back(i);
      notInRegion:
        ;
    }

    if(!indicesToRemove.empty()){
        SgVertexArray& points = *pointSet->vertices();
        points.clear();
        int j = 0;
        int nextIndexToRemove = indicesToRemove[j++];
        for(int i=0; i < numOrgPoints; ++i){
            if(i == nextIndexToRemove){
                if(j < indicesToRemove.size()){
                    nextIndexToRemove = indicesToRemove[j++];
                }
            } else {
                points.push_back(orgPoints[i]);
                
            }
        }
        if(pointSet->hasNormals()){
            removeSubElements(*pointSet->normals(), pointSet->normalIndices(), indicesToRemove);
        }
        if(pointSet->hasColors()){
            removeSubElements(*pointSet->colors(), pointSet->colorIndices(), indicesToRemove);
        }

        pointSet->notifyUpdate();
    }
}    


template<class ElementContainer>
void PointSetItemImpl::removeSubElements(ElementContainer& elements, SgIndexArray& indices, const vector<int>& indicesToRemove)
{
    const ElementContainer orgElements(elements);
    const int numOrgElements = orgElements.size();
    elements.clear();
    
    if(indices.empty()){
        int j = 0;
        int nextIndexToRemove = indicesToRemove[j++];
        for(int i=0; i < numOrgElements; ++i){
            if(i == nextIndexToRemove){
                if(j < indicesToRemove.size()){
                    nextIndexToRemove = indicesToRemove[j++];
                }
            } else {
                elements.push_back(orgElements[i]);
            }
        }
    } else {
        const SgIndexArray orgIndices(indices);
        const int numOrgIndices = orgIndices.size();
        indices.clear();
        dynamic_bitset<> elementValidness(numOrgElements);
        int j = 0;
        int nextIndexToRemove = indicesToRemove[j++];
        for(int i=0; i < numOrgIndices; ++i){
            if(i == nextIndexToRemove){
                if(j < indicesToRemove.size()){
                    nextIndexToRemove = indicesToRemove[j++];
                }
            } else {
                int index = orgIndices[i];
                elementValidness.set(index);
                indices.push_back(index);
            }
        }
        vector<int> indexMap(numOrgElements);
        int newIndex = 0;
        for(int i=0; i < numOrgElements; ++i){
            if(elementValidness.test(i)){
                elements.push_back(orgElements[i]);
                ++newIndex;
            }
            indexMap[i] = newIndex;
        }
        for(size_t i=0; i < indices.size(); ++i){
            indices[i] = indexMap[i];
        }
    }
}


void PointSetItem::doPutProperties(PutPropertyFunction& putProperty)
{
    ScenePointSet* scene = impl->scenePointSet;
    putProperty(_("File"), getFilename(filePath()));
    putProperty(_("Rendering mode"), scene->renderingMode,
                boost::bind(&ScenePointSet::setRenderingMode, scene, _1));
    putProperty.decimals(1).min(0.0)(_("Point size"), pointSize(),
                                     boost::bind(&ScenePointSet::setPointSize, scene, _1), true);
    putProperty.decimals(4)(_("Voxel size"), voxelSize(),
                            boost::bind(&ScenePointSet::setVoxelSize, scene, _1), true);
    putProperty(_("Editable"), isEditable(), boost::bind(&PointSetItemImpl::onEditableChanged, impl, _1));
    const SgVertexArray* points = impl->pointSet->vertices();
    putProperty(_("Num points"), static_cast<int>(points ? points->size() : 0));
    putProperty(_("Translation"), str(Vector3(offsetPosition().translation())),
                boost::bind(&PointSetItemImpl::onTranslationPropertyChanged, impl, _1));
    Vector3 rpy(rpyFromRot(offsetPosition().linear()));
    putProperty("RPY", str(TO_DEGREE * rpy), boost::bind(&PointSetItemImpl::onRotationPropertyChanged, impl, _1));
    
}


bool PointSetItemImpl::onTranslationPropertyChanged(const std::string& value)
{
    Vector3 p;
    if(toVector3(value, p)){
        scenePointSet->setTranslation(p);
        self->notifyOffsetPositionChange();
        return true;
    }
    return false;
}


bool PointSetItemImpl::onRotationPropertyChanged(const std::string& value)
{
    Vector3 rpy;
    if(toVector3(value, rpy)){
        scenePointSet->setRotation(rotFromRpy(TO_RADIAN * rpy));
        self->notifyOffsetPositionChange();
        return true;
    }
    
    return false;
}


bool PointSetItem::store(Archive& archive)
{
    ScenePointSet* scene = impl->scenePointSet;
    if(!filePath().empty()){
        archive.writeRelocatablePath("file", filePath());
        archive.write("format", fileFormat());
    }
    archive.write("renderingMode", scene->renderingMode.selectedSymbol());
    archive.write("pointSize", pointSize());
    archive.write("voxelSize", scene->voxelSize);
    archive.write("isEditable", isEditable());
    return true;
}


bool PointSetItem::restore(const Archive& archive)
{
    ScenePointSet* scene = impl->scenePointSet;
    string symbol;
    if(archive.read("renderingMode", symbol)){
        scene->setRenderingMode(scene->renderingMode.index(symbol));
    }
    scene->setPointSize(archive.get("pointSize", pointSize()));
    scene->setVoxelSize(archive.get("voxelSize", voxelSize()));
    setEditable(archive.get("isEditable", isEditable()));
    
    std::string filename, formatId;
    if(archive.readRelocatablePath("file", filename) && archive.read("format", formatId)){
        return load(filename, archive.currentParentItem(), formatId);
    }
    return true;
}


ScenePointSet::ScenePointSet(PointSetItemImpl* pointSetItemImpl)
    : weakPointSetItem(pointSetItemImpl->self),
      orgPointSet(pointSetItemImpl->pointSet),
      renderingMode(N_RENDERING_MODES)
{
    visiblePointSet = new SgPointSet;

    voxels = new SgShape;
    voxels->getOrCreateMaterial();
    voxelSize = 0.01f;

    renderingMode.setSymbol(POINT_MODE, N_("Point"));
    renderingMode.setSymbol(VOXEL_MODE, N_("Voxel"));
    renderingMode.select(POINT_MODE);

    isEditable_ = false;
}


bool ScenePointSet::setRenderingMode(int mode)
{
    bool result = true;
    if(mode != renderingMode.which()){
        result = renderingMode.select(mode);
        if(invariant){
            updateVisualization(true);
        }
    }
    return result;
}


void ScenePointSet::setPointSize(double size)
{
    if(size != visiblePointSet->pointSize()){
        visiblePointSet->setPointSize(size);
        if(renderingMode.is(POINT_MODE) && invariant){
            updateVisualization(false);
        }
    }
}

void ScenePointSet::setVoxelSize(double size)
{
    if(size != voxelSize){
        voxelSize = size;
        if(renderingMode.is(VOXEL_MODE) && invariant){
            updateVisualization(true);
        }
    }
}


int ScenePointSet::numAttentionPoints() const
{
    return attentionPointMarkerGroup ? attentionPointMarkerGroup->numChildren() : 0;
}


Vector3 ScenePointSet::attentionPoint(int index) const
{
    if(index < numAttentionPoints()){
        CrossMarker* marker = dynamic_cast<CrossMarker*>(attentionPointMarkerGroup->child(index));
        if(marker){
            return T() * marker->translation();
        }
    }
    return Vector3::Zero();
}


void ScenePointSet::clearAttentionPoints(bool doNotify)
{
    if(attentionPointMarkerGroup){
        if(!attentionPointMarkerGroup->empty()){
            attentionPointMarkerGroup->clearChildren();
            if(doNotify){
                notifyAttentionPointChange();
            }
        }
    }
}


void ScenePointSet::addAttentionPoint(const Vector3& point, bool doNotify)
{
    if(!attentionPointMarkerGroup){
        attentionPointMarkerGroup = new SgGroup;
        addChild(attentionPointMarkerGroup);
    }
    
    Vector3f color(1.0f, 1.0f, 0.0f);
    CrossMarker* marker = new CrossMarker(0.02, color);
    marker->setTranslation(T().inverse() * point);
    attentionPointMarkerGroup->addChild(marker);

    if(doNotify){
        notifyAttentionPointChange();
    }
}


void ScenePointSet::setAttentionPoint(const Vector3& p, bool doNotify)
{
    clearAttentionPoints(false);
    addAttentionPoint(p, doNotify);
}


bool ScenePointSet::removeAttentionPoint(const Vector3& point, double distanceThresh, bool doNotify)
{
    bool removed = false;
    if(attentionPointMarkerGroup){
        SgGroup::iterator iter = attentionPointMarkerGroup->begin();
        while(iter != attentionPointMarkerGroup->end()){
            CrossMarker* marker = dynamic_cast<CrossMarker*>(iter->get());
            if(point.isApprox(marker->translation(), distanceThresh)){
                iter = attentionPointMarkerGroup->erase(iter);
                removed = true;
            } else {
                ++iter;
            }
        }
        if(removed){
            if(doNotify){
                notifyAttentionPointChange();
            }
        }
    }
    return removed;
}


void ScenePointSet::notifyAttentionPointChange()
{
    if(attentionPointMarkerGroup){
        attentionPointMarkerGroup->notifyUpdate();
    }
    sigAttentionPointsChanged();
}


void ScenePointSet::updateVisualization(bool updateContents)
{
    if(invariant){
        removeChild(invariant);
        invariant->removeChild(visiblePointSet);
        invariant->removeChild(voxels);
    }
    invariant = new SgInvariantGroup;
    
    if(renderingMode.is(POINT_MODE)){
        if(updateContents){
            updateVisiblePointSet();
        }
        invariant->addChild(visiblePointSet);
    } else {
        if(updateContents){
            updateVoxels();
        }
        invariant->addChild(voxels);
    }
    addChild(invariant, true);

    clearAttentionPoints(true);
}


void ScenePointSet::updateVisiblePointSet()
{
    visiblePointSet->setVertices(orgPointSet->vertices());
    visiblePointSet->setNormals(orgPointSet->normals());
    visiblePointSet->normalIndices() = orgPointSet->normalIndices();
    visiblePointSet->setColors(orgPointSet->colors());
    visiblePointSet->colorIndices() = orgPointSet->colorIndices();
}


void ScenePointSet::updateVoxels()
{
    SgMeshPtr mesh;
    if(orgPointSet->hasVertices()){
        mesh = new SgMesh;
        mesh->setSolid(true);
        const SgVertexArray& points = *orgPointSet->vertices();
        const int n = points.size();
        SgVertexArray& vertices = *mesh->getOrCreateVertices();
        vertices.reserve(n * 8);
        SgNormalArray& normals = *mesh->setNormals(new SgNormalArray(6));
        normals[0] <<  1.0f,  0.0f,  0.0f;
        normals[1] << -1.0f,  0.0f,  0.0f;
        normals[2] <<  0.0f,  1.0f,  0.0f;
        normals[3] <<  0.0f, -1.0f,  0.0f;
        normals[4] <<  0.0f,  0.0f,  1.0f;
        normals[5] <<  0.0f,  0.0f, -1.0f;
        SgIndexArray& normalIndices = mesh->normalIndices();
        normalIndices.reserve(12 * 3 * n);
        mesh->reserveNumTriangles(n * 12);
        const float s = voxelSize / 2.0;
        for(int i=0; i < n; ++i){
            const int top = vertices.size();
            const Vector3f& p = points[i];
            const float x0 = p.x() + s;
            const float x1 = p.x() - s;
            const float y0 = p.y() + s;
            const float y1 = p.y() - s;
            const float z0 = p.z() + s;
            const float z1 = p.z() - s;
            vertices.push_back(Vector3f(x0, y0, z0));
            vertices.push_back(Vector3f(x1, y0, z0));
            vertices.push_back(Vector3f(x1, y1, z0));
            vertices.push_back(Vector3f(x0, y1, z0));
            vertices.push_back(Vector3f(x0, y0, z1));
            vertices.push_back(Vector3f(x1, y0, z1));
            vertices.push_back(Vector3f(x1, y1, z1));
            vertices.push_back(Vector3f(x0, y1, z1));

            static const int boxTriangles[][3] = {
                { 0, 1, 2 }, { 0, 2, 3 }, // +Z
                { 0, 5, 1 }, { 0, 4, 5 }, // +Y
                { 1, 5, 2 }, { 2, 5, 6 }, // -X
                { 2, 6, 3 }, { 3, 6, 7 }, // -Y
                { 0, 3, 4 }, { 3, 7, 4 }, // +X
                { 4, 6, 5 }, { 4, 7, 6 }  // -Z
            };
            static const int boxNormalIndices[] = {
                4, 4, 2, 2, 1, 1, 3, 3, 0, 0, 5, 5
            };
            for(int j=0; j < 12; ++j){
                const int* tri = boxTriangles[j];
                mesh->addTriangle(top + tri[0], top + tri[1], top + tri[2]);
                const int normalIndex = boxNormalIndices[j];
                normalIndices.push_back(normalIndex);
                normalIndices.push_back(normalIndex);
                normalIndices.push_back(normalIndex);
            }
        }
        if(orgPointSet->hasColors()){
            SgColorArray& colors = *mesh->setColors(orgPointSet->colors());
            const int m = colors.size();
            const SgIndexArray& orgColorIndices = orgPointSet->colorIndices();
            SgIndexArray& colorIndices = mesh->colorIndices();
            if(orgColorIndices.empty()){
                colorIndices.reserve(m * 36);
                for(int i=0; i < m; ++i){
                    for(int j=0; j < 36; ++j){
                        colorIndices.push_back(i);
                    }
                }
            } else {
                const int l = orgColorIndices.size();
                colorIndices.reserve(l * 36);
                for(int i=0; i < l; ++i){
                    const int index = orgColorIndices[l];
                    for(int j=0; j < 36; ++j){
                        colorIndices.push_back(index);
                    }
                }
            }
        }
    }
    voxels->setMesh(mesh);
}


bool ScenePointSet::onButtonPressEvent(const SceneWidgetEvent& event)
{
	if(!isEditable_){
        return false;
    }
    
    bool processed = false;
    
    if(event.button() == Qt::LeftButton){
        if(event.modifiers() & Qt::ControlModifier){
            if(!removeAttentionPoint(event.point(), 0.01, true)){
                addAttentionPoint(event.point(), true);
            }
        } else {
            setAttentionPoint(event.point(), true);
        }
        processed = true;
    }
    
    return processed;
}


bool ScenePointSet::onPointerMoveEvent(const SceneWidgetEvent& event)
{
    return false;
}


void ScenePointSet::onContextMenuRequest(const SceneWidgetEvent& event, MenuManager& menuManager)
{
    menuManager.addItem(_("PointSet: Clear Attention Points"))->sigTriggered().connect(
        boost::bind(&ScenePointSet::clearAttentionPoints, this, true));

    PointSetEraser* eraser = dynamic_cast<PointSetEraser*>(event.sceneWidget()->activeEventFilter());
    if(!eraser || eraser->weakPointSetItem.lock() != this->weakPointSetItem.lock()){
        menuManager.addItem(_("PointSet: Start Eraser Mode"))->sigTriggered().connect(
            boost::bind(&ScenePointSet::startEraserMode, this, event.sceneWidget()));
    }
}


void ScenePointSet::onSceneModeChanged(const SceneWidgetEvent& event)
{

}


void ScenePointSet::startEraserMode(SceneWidget* sceneWidget)
{
    if(!weakPointSetItem.expired()){
        sceneWidget->installEventFilter(new PointSetEraser(this, weakPointSetItem, sceneWidget));
    }
}


PointSetEraser::PointSetEraser(ScenePointSet* scenePointSet, weak_ref_ptr<PointSetItem>& weakPointSetItem, SceneWidget* sceneWidget)
    : scenePointSet(scenePointSet),
      weakPointSetItem(weakPointSetItem),
      sceneWidget(sceneWidget)
{
    rect = new RectLineOverlay;
    onSceneModeChanged(sceneWidget->latestEvent());
}


PointSetEraser::~PointSetEraser()
{
    showRectangle(false);
}


void PointSetEraser::showRectangle(bool on)
{
    if(on){
        if(!rect->hasParents()){
            sceneWidget->sceneRoot()->addChild(rect, true);
        }
    } else {
        if(rect->hasParents()){
            sceneWidget->sceneRoot()->removeChild(rect, true);
        }
    }
}


void PointSetEraser::onSceneModeChanged(const SceneWidgetEvent& event)
{
    if(event.sceneWidget()->isEditMode()){
        event.sceneWidget()->setCursor(
            QCursor(QPixmap(":/Base/icons/eraser-cursor.png"), 3, 2));
    } else {
        showRectangle(false);
    }
}


bool PointSetEraser::onButtonPressEvent(const SceneWidgetEvent& event)
{
    x0 = event.x();
    y0 = event.y();
    rect->setRect(x0, y0, x0, y0);
    showRectangle(true);
    return true;
}


bool PointSetEraser::onButtonReleaseEvent(const SceneWidgetEvent& event)
{
    PointSetItemPtr pointSetItem = weakPointSetItem.lock();
    if(pointSetItem){
        if(rect->left < rect->right && rect->bottom < rect->top){
            PointSetItem::Region r(4);
            event.sceneWidget()->unproject(rect->left, rect->top, 0.0, r.point(0));
            event.sceneWidget()->unproject(rect->left, rect->bottom, 0.0, r.point(1));
            event.sceneWidget()->unproject(rect->right, rect->bottom, 0.0, r.point(2));
            event.sceneWidget()->unproject(rect->right, rect->top, 0.0, r.point(3));
            const Vector3 c = event.currentCameraPosition().translation();
        	SgCamera* camera = event.sceneWidget()->renderer().currentCamera();
        	if(dynamic_cast<SgPerspectiveCamera*>(camera)){
                for(int i=0; i < 4; ++i){
                    r.normal(i) = (r.point((i + 1) % 4) - c).cross(r.point(i) - c).normalized();
                }
            } else if(dynamic_cast<SgOrthographicCamera*>(camera)){
                const Vector3 n0 = (r.point(3) - r.point(0)).cross(r.point(1) - r.point(0)).normalized();
                for(int i=0; i< 4; ++i){
                    r.normal(i) = (r.point((i + 1) % 4) - r.point(i)).cross(n0).normalized();
                }
            }

            SigRegionFixed& signal = scenePointSet->sigRegionFixed;
            if(signal.empty() || signal(PointSetItem::REMOVAL, r)){
                pointSetItem->removePoints(r);
            }
        }
    }
    showRectangle(false);
    return true;
}


bool PointSetEraser::onPointerMoveEvent(const SceneWidgetEvent& event)
{
    rect->setRect(x0, y0, event.x(), event.y());
    return true;
}


void PointSetEraser::onContextMenuRequest(const SceneWidgetEvent& event, MenuManager& menuManager)
{
    menuManager.addItem(_("PointSet: Exit Eraser Mode"))->sigTriggered().connect(
        boost::bind(&SceneWidget::removeEventFilter, event.sceneWidget(), this));
}
        

RectLineOverlay::RectLineOverlay()
{
    SgLineSet* lineSet = new SgLineSet;
    vertices = lineSet->getOrCreateVertices();
    vertices->resize(4);
    lineSet->getOrCreateColors()->push_back(Vector3f(1.0f, 0.0f, 0.0f));
    lineSet->reserveNumLines(4);
    SgIndexArray& colorIndices = lineSet->colorIndices();
    colorIndices.reserve(8);
    for(int i=0; i < 4; ++i){
        lineSet->addLine(i, (i + 1) % 4);
        colorIndices.push_back(0);
        colorIndices.push_back(0);
    }
    addChild(lineSet);
}


void RectLineOverlay::calcViewVolume(double viewportWidth, double viewportHeight, ViewVolume& io_volume)
{
    io_volume.left = 0;
    io_volume.right = viewportWidth;
    io_volume.bottom = 0;
    io_volume.top = viewportHeight;
}


void RectLineOverlay::setRect(int x0, int y0, int x1, int y1)
{
    if(x0 <= x1){
        left = x0;
        right = x1;
    } else {
        left = x1;
        right = x0;
    }
    if(y0 >= y1){
        top = y0;
        bottom = y1;
    } else {
        top = y1;
        bottom = y0;
    }

    vertices->at(0) << left,  top,    0.0f;
    vertices->at(1) << left,  bottom, 0.0f;
    vertices->at(2) << right, bottom, 0.0f;
    vertices->at(3) << right, top,    0.0f;

    vertices->notifyUpdate();
}
