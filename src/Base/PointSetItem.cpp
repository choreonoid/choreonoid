#include "PointSetItem.h"
#include "RectRegionMarker.h"
#include "ItemManager.h"
#include "ItemFileIO.h"
#include "MenuManager.h"
#include "PutPropertyFunction.h"
#include "Archive.h"
#include <cnoid/EigenArchive>
#include <cnoid/SceneWidget>
#include <cnoid/SceneWidgetEventHandler>
#include <cnoid/SceneDrawables>
#include <cnoid/SceneMarkers>
#include <cnoid/PointSetUtil>
#include <cnoid/PolyhedralRegion>
#include <cnoid/CloneMap>
#include <cnoid/Exception>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

class PointSetItemPcdFileIo : public ItemFileIoBase<PointSetItem>
{
public:
    PointSetItemPcdFileIo();
    virtual bool load(PointSetItem* item, const std::string& filename) override;
    virtual bool save(PointSetItem* item, const std::string& filename) override;
};

class ScenePointSet : public SgPosTransform, public SceneWidgetEventHandler
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    weak_ref_ptr<PointSetItem> weakPointSetItem;
    SgPointSetPtr orgPointSet;
    SgPointSetPtr visiblePointSet;
    SgUpdate update;
    SgShapePtr voxels;
    float voxelSize;
    SgInvariantGroupPtr invariant;
    Selection renderingMode;
    RectRegionMarkerPtr regionMarker;
    ScopedConnection eraserModeMenuItemConnection;
    bool isEditable_;

    Signal<void()> sigAttentionPointsChanged;
    SgGroupPtr attentionPointMarkerGroup;
    
    ScenePointSet(PointSetItem::Impl* pointSetItem);

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

    virtual bool onButtonPressEvent(SceneWidgetEvent* event) override;
    virtual bool onPointerMoveEvent(SceneWidgetEvent* event) override;
    virtual bool onContextMenuRequest(SceneWidgetEvent* event) override;
    void onContextMenuRequestInEraserMode(SceneWidgetEvent* event);
    void onRegionFixed(const PolyhedralRegion& region);
};

typedef ref_ptr<ScenePointSet> ScenePointSetPtr;

class PointSetLocation : public LocationProxy
{
public:
    PointSetItem* item;

    PointSetLocation(PointSetItem* item);
    virtual std::string getName() const override;
    virtual Isometry3 getLocation() const override;
    virtual bool setLocation(const Isometry3& T) override;
    virtual SignalProxy<void()> sigLocationChanged() override;
};

}

namespace cnoid {

class PointSetItem::Impl
{
public:
    PointSetItem* self;
    SgPointSetPtr pointSet;
    ScenePointSetPtr scene;
    ScopedConnection pointSetUpdateConnection;
    Signal<void()> sigOffsetPositionChanged;
    Signal<void(const PolyhedralRegion& region)> sigPointsInRegionRemoved;
    ref_ptr<PointSetLocation> location;

    Impl(PointSetItem* self);
    Impl(PointSetItem* self, const Impl& org, CloneMap* cloneMap);
    void initialize();
    void setRenderingMode(int mode);
    bool onEditableChanged(bool on);
    void removePoints(const PolyhedralRegion& region);
    template<class ElementContainer>
    void removeSubElements(ElementContainer& elements, SgIndexArray& indices, const vector<int>& indicesToRemove);
    bool onTranslationPropertyChanged(const std::string& value);
    bool onRotationPropertyChanged(const std::string& value);
};

}


PointSetItemPcdFileIo::PointSetItemPcdFileIo()
    : ItemFileIoBase("PCD", Load | Save)
{
    setCaption(_("Point Cloud"));
    setFileTypeCaption("PCD");
    setExtensionForLoading("pcd");
    addFormatAlias("PCD-FILE");
}


bool PointSetItemPcdFileIo::load(PointSetItem* item, const std::string& filename)
{
    try {
        cnoid::loadPCD(item->pointSet(), filename);
        os() << item->pointSet()->vertices()->size() << " points have been loaded.";
        auto itype = currentInvocationType();
        if(itype == Dialog || itype == DragAndDrop){
            item->setChecked(true);
        }
        return true;
    } catch (boost::exception& ex) {
        if(std::string const * message = boost::get_error_info<error_info_message>(ex)){
            putError(*message);
        }
    }
    return false;
}


bool PointSetItemPcdFileIo::save(PointSetItem* item, const std::string& filename)
{
    try {
        cnoid::savePCD(item->pointSet(), filename, item->offsetPosition());
        return true;
    } catch (boost::exception& ex) {
        if(std::string const * message = boost::get_error_info<error_info_message>(ex)){
            putError(*message);
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
        im.addFileIO<PointSetItem>(new PointSetItemPcdFileIo);
        initialized = true;
    }
}


PointSetItem::PointSetItem()
{
    setAttributes(FileImmutable | Reloadable);
    impl = new Impl(this);
}


PointSetItem::Impl::Impl(PointSetItem* self)
    : self(self)
{
    pointSet = new SgPointSet;
    scene = new ScenePointSet(this);

    initialize();
}


PointSetItem::PointSetItem(const PointSetItem& org, CloneMap* cloneMap)
    : Item(org)
{
    impl = new Impl(this, *org.impl, cloneMap);
}


PointSetItem::Impl::Impl(PointSetItem* self, const Impl& org, CloneMap* cloneMap)
    : self(self)
{
    pointSet = CloneMap::getClone(org.pointSet, cloneMap);
    scene = new ScenePointSet(this);
    scene->T() = org.scene->T();

    initialize();
}


void PointSetItem::Impl::initialize()
{
    pointSetUpdateConnection.reset(
        pointSet->sigUpdated().connect([&](const SgUpdate&){ self->notifyUpdate(); }));
}


PointSetItem::~PointSetItem()
{
    delete impl;
}


Item* PointSetItem::doCloneItem(CloneMap* cloneMap) const
{
    return new PointSetItem(*this, cloneMap);
}


bool PointSetItem::setName(const std::string& name)
{
    impl->scene->setName(name);
    impl->pointSet->setName(name);
    Item::setName(name);
    return true;
}


void PointSetItem::notifyUpdate()
{
    impl->scene->updateVisualization(true);
    Item::notifyUpdate();
}


SgNode* PointSetItem::getScene()
{
    return impl->scene;
}


LocationProxyPtr PointSetItem::getLocationProxy()
{
    if(!impl->location){
        impl->location = new PointSetLocation(this);
    }
    return impl->location;
}


const SgPointSet* PointSetItem::pointSet() const
{
    return impl->pointSet;
}


SgPointSet* PointSetItem::pointSet()
{
    return impl->pointSet;
}


const Isometry3& PointSetItem::offsetPosition() const
{
    return impl->scene->T();
}


void PointSetItem::setOffsetPosition(const Isometry3& T)
{
    impl->scene->setPosition(T);
}


Vector3 PointSetItem::offsetTranslation() const
{
    return impl->scene->translation();
}


void PointSetItem::setOffsetTranslation(const Vector3& p)
{
    impl->scene->setTranslation(p);
}


SignalProxy<void()> PointSetItem::sigOffsetPositionChanged()
{
    return impl->sigOffsetPositionChanged;
}


void PointSetItem::notifyOffsetPositionChange(bool doNotifyScene)
{
    impl->sigOffsetPositionChanged();
    if(doNotifyScene){
        impl->scene->notifyUpdate(impl->scene->update.withAction(SgUpdate::Modified));
    }
    Item::notifyUpdate();
}


SgPointSet* PointSetItem::getTransformedPointSet() const
{
    SgPointSet* transformed = new SgPointSet;
    SgVertexArray& points = *transformed->getOrCreateVertices();
    SgVertexArray* orgPoints = impl->pointSet->vertices();
    if(orgPoints){
        const int n = orgPoints->size();
        points.resize(n);
        const Isometry3& T = offsetPosition();
        for(int i=0; i < n; ++i){
            points[i] = (T * (*orgPoints)[i].cast<Isometry3::Scalar>()).cast<SgVertexArray::Scalar>();
        }
    }
    return transformed;
}


void PointSetItem::setRenderingMode(int mode)
{
    impl->setRenderingMode(mode);
}


void PointSetItem::Impl::setRenderingMode(int mode)
{
    scene->renderingMode.select(mode);
}



int PointSetItem::renderingMode() const
{
    return impl->scene->renderingMode.which();
}


double PointSetItem::pointSize() const
{
    return impl->scene->visiblePointSet->pointSize();
}
    

void PointSetItem::setPointSize(double size)
{
    impl->scene->setPointSize(size);
}


double PointSetItem::defaultVoxelSize()
{
    return 0.01f;
}


double PointSetItem::voxelSize() const
{
    return impl->scene->voxelSize;
}
    

void PointSetItem::setVoxelSize(double size)
{
    impl->scene->setVoxelSize(size);
}


void PointSetItem::setEditable(bool on)
{
    impl->scene->setEditable(on);
}


bool PointSetItem::isEditable() const
{
    return impl->scene->isEditable();
}


bool PointSetItem::Impl::onEditableChanged(bool on)
{
    scene->setEditable(on);
    return true;
}


int PointSetItem::numAttentionPoints() const
{
    return impl->scene->numAttentionPoints();
}


Vector3 PointSetItem::attentionPoint(int index) const
{
    return impl->scene->attentionPoint(index);
}
        
    
void PointSetItem::clearAttentionPoints()
{
    impl->scene->clearAttentionPoints(false);
}


void PointSetItem::addAttentionPoint(const Vector3& p)
{
    impl->scene->addAttentionPoint(p, false);
}


SignalProxy<void()> PointSetItem::sigAttentionPointsChanged()
{
    return impl->scene->sigAttentionPointsChanged;
}


stdx::optional<Vector3> PointSetItem::attentionPoint() const
{
    if(numAttentionPoints() == 1){
        return attentionPoint(0);
    }
    return stdx::nullopt;
}


void PointSetItem::clearAttentionPoint()
{
    impl->scene->clearAttentionPoints(false);
}


void PointSetItem::setAttentionPoint(const Vector3& p)
{
    impl->scene->setAttentionPoint(p, false);
}


SignalProxy<void()> PointSetItem::sigAttentionPointChanged()
{
    return impl->scene->sigAttentionPointsChanged;
}


void PointSetItem::notifyAttentionPointChange()
{
    impl->scene->notifyAttentionPointChange();
}


SignalProxy<void(const PolyhedralRegion& region)> PointSetItem::sigPointsInRegionRemoved()
{
    return impl->sigPointsInRegionRemoved;
}


void PointSetItem::removePoints(const PolyhedralRegion& region)
{
    impl->removePoints(region);
}


void PointSetItem::Impl::removePoints(const PolyhedralRegion& region)
{
    vector<int> indicesToRemove;
    const Isometry3 T = scene->T();
    SgVertexArray orgPoints(*pointSet->vertices());
    const int numOrgPoints = orgPoints.size();

    for(int i=0; i < numOrgPoints; ++i){
        if(region.checkInside(T * orgPoints[i].cast<Vector3::Scalar>())){
            indicesToRemove.push_back(i);
        }
    }

    if(!indicesToRemove.empty()){
        SgVertexArray& points = *pointSet->vertices();
        points.clear();
        int j = 0;
        int nextIndexToRemove = indicesToRemove[j++];
        for(int i=0; i < numOrgPoints; ++i){
            if(i == nextIndexToRemove){
                if(j < static_cast<int>(indicesToRemove.size())){
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

        pointSet->notifyUpdate(scene->update.withAction(SgUpdate::Modified));
    }

    sigPointsInRegionRemoved(region);
}


template<class ElementContainer>
void PointSetItem::Impl::removeSubElements(ElementContainer& elements, SgIndexArray& indices, const vector<int>& indicesToRemove)
{
    const ElementContainer orgElements(elements);
    const int numOrgElements = orgElements.size();
    elements.clear();
    
    if(indices.empty()){
        int j = 0;
        int nextIndexToRemove = indicesToRemove[j++];
        for(int i=0; i < numOrgElements; ++i){
            if(i == nextIndexToRemove){
                if(j < static_cast<int>(indicesToRemove.size())){
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
        vector<bool> elementValidness(numOrgElements, false);
        int j = 0;
        int nextIndexToRemove = indicesToRemove[j++];
        for(int i=0; i < numOrgIndices; ++i){
            if(i == nextIndexToRemove){
                if(j < static_cast<int>(indicesToRemove.size())){
                    nextIndexToRemove = indicesToRemove[j++];
                }
            } else {
                int index = orgIndices[i];
                elementValidness[index] = true;
                indices.push_back(index);
            }
        }
        vector<int> indexMap(numOrgElements);
        int newIndex = 0;
        for(int i=0; i < numOrgElements; ++i){
            if(elementValidness[i]){
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
    ScenePointSet* scene = impl->scene;
    putProperty(_("File"), fileName());
    putProperty(_("Rendering mode"), scene->renderingMode,
                [&](int mode){ impl->setRenderingMode(mode); return true; });

    putProperty.min(0.0);
    putProperty.decimals(1)
        (_("Point size"), pointSize(),
         [=](double size){ scene->setPointSize(size); return true; });
    putProperty.decimals(4)
        (_("Voxel size"), voxelSize(),
         [=](double size){ scene->setVoxelSize(size); return true; });
    
    putProperty(_("Editable"), isEditable(), [&](bool on){ return impl->onEditableChanged(on); });
    const SgVertexArray* points = impl->pointSet->vertices();
    putProperty(_("Num points"), static_cast<int>(points ? points->size() : 0));
    putProperty(_("Translation"), str(Vector3(offsetPosition().translation())),
                [&](const string& value){ return impl->onTranslationPropertyChanged(value); });
    Vector3 rpy(TO_DEGREE * rpyFromRot(offsetPosition().linear()));
    putProperty(_("Rotation"), str(rpy), [&](const string& value){ return impl->onRotationPropertyChanged(value);});
}


bool PointSetItem::Impl::onTranslationPropertyChanged(const std::string& value)
{
    Vector3 p;
    if(toVector3(value, p)){
        scene->setTranslation(p);
        self->notifyOffsetPositionChange();
        return true;
    }
    return false;
}


bool PointSetItem::Impl::onRotationPropertyChanged(const std::string& value)
{
    Vector3 rpy;
    if(toVector3(value, rpy)){
        scene->setRotation(rotFromRpy(TO_RADIAN * rpy));
        self->notifyOffsetPositionChange();
        return true;
    }
    return false;
}


bool PointSetItem::store(Archive& archive)
{
    archive.writeFileInformation(this);

    ScenePointSet* scene = impl->scene;
    write(archive, "translation", Vector3(scene->translation()));
    writeDegreeAngleAxis(archive, "rotation", AngleAxis(scene->rotation()));
    // The following element is used to distinguish the value type from the old one using radian.
    // The old format is deprecated, and writing the following element should be omitted in the future.
    archive.write("angle_unit", "degree");
    archive.write("rendering_mode", scene->renderingMode.selectedSymbol());
    archive.write("point_size", pointSize());
    archive.write("voxel_size", scene->voxelSize);
    archive.write("is_editable", isEditable());
    
    return true;
}


bool PointSetItem::restore(const Archive& archive)
{
    std::string filename, formatId;
    if(archive.read("file", filename) && archive.read("format", formatId)){
        filename = archive.resolveRelocatablePath(filename);
        if(filename.empty()){
            return false; // Invalid relocatable path
        }
        if(!load(filename, archive.currentParentItem(), formatId)){
            return false;
        }
    }
    
    ScenePointSet* scene = impl->scene;

    Vector3 translation;
    if(read(archive, "translation", translation)){
        scene->setTranslation(translation);
    }
    AngleAxis rot;
    string unit;
    bool hasRot = false;
    if(archive.read("angle_unit", unit) && unit == "degree"){
        hasRot = readDegreeAngleAxis(archive, "rotation", rot);
    } else { // for the backward compatibility
        hasRot = readAngleAxis(archive, "rotation", rot);
    }
    if(hasRot){
        scene->setRotation(rot);
    }
    
    string symbol;
    if(archive.read({ "rendering_mode", "renderingMode" }, symbol)){
        impl->setRenderingMode(scene->renderingMode.index(symbol));
    }
    scene->setPointSize(archive.get({ "point_size", "pointSize" }, pointSize()));
    scene->setVoxelSize(archive.get({ "voxel_size", "voxelSize" }, voxelSize()));
    setEditable(archive.get({ "is_editable", "isEditable" }, isEditable()));

    return true;
}


ScenePointSet::ScenePointSet(PointSetItem::Impl* pointSetItemImpl)
    : weakPointSetItem(pointSetItemImpl->self),
      orgPointSet(pointSetItemImpl->pointSet),
      renderingMode(PointSetItem::N_RENDERING_MODES)
{
    visiblePointSet = new SgPointSet;

    voxels = new SgShape;
    voxels->getOrCreateMaterial();
    voxelSize = PointSetItem::defaultVoxelSize();

    renderingMode.setSymbol(PointSetItem::POINT, N_("Point"));
    renderingMode.setSymbol(PointSetItem::VOXEL, N_("Voxel"));
    renderingMode.select(PointSetItem::POINT);

    regionMarker = new RectRegionMarker;
    regionMarker->setEditModeCursor(QCursor(QPixmap(":/Base/icon/eraser-cursor.png"), 3, 2));
    regionMarker->sigRegionFixed().connect(
        [&](const PolyhedralRegion& region){ onRegionFixed(region); });
    regionMarker->sigContextMenuRequest().connect(
        [&](SceneWidgetEvent* event){ onContextMenuRequestInEraserMode(event); });

    isEditable_ = false;
}


void ScenePointSet::setPointSize(double size)
{
    if(size != visiblePointSet->pointSize()){
        visiblePointSet->setPointSize(size);
        if(renderingMode.is(PointSetItem::POINT) && invariant){
            updateVisualization(false);
        }
    }
}

void ScenePointSet::setVoxelSize(double size)
{
    if(size != voxelSize){
        voxelSize = size;
        if(renderingMode.is(PointSetItem::VOXEL) && invariant){
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
        attentionPointMarkerGroup->notifyUpdate(update);
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
    
    if(renderingMode.is(PointSetItem::POINT)){
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
    addChild(invariant, update);

    clearAttentionPoints(true);
}


void ScenePointSet::updateVisiblePointSet()
{
    visiblePointSet->setVertices(orgPointSet->vertices());
    visiblePointSet->setNormals(orgPointSet->normals());
    visiblePointSet->normalIndices() = orgPointSet->normalIndices();
    visiblePointSet->setColors(orgPointSet->colors());
    visiblePointSet->colorIndices() = orgPointSet->colorIndices();
    visiblePointSet->notifyUpdate(update);
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


bool ScenePointSet::onButtonPressEvent(SceneWidgetEvent* event)
{
    if(!isEditable_){
        return false;
    }
    
    bool processed = false;
    
    if(event->button() == Qt::LeftButton){
        if(event->modifiers() & Qt::ControlModifier){
            if(!removeAttentionPoint(event->point(), 0.01, true)){
                addAttentionPoint(event->point(), true);
            }
        } else {
            setAttentionPoint(event->point(), true);
        }
        processed = true;
    }
    
    return processed;
}


bool ScenePointSet::onPointerMoveEvent(SceneWidgetEvent* event)
{
    return false;
}


bool ScenePointSet::onContextMenuRequest(SceneWidgetEvent* event)
{
    if(isEditable_){
        auto menu = event->contextMenu();
        menu->addItem(_("PointSet: Clear Attention Points"))->sigTriggered().connect(
            [&](){ clearAttentionPoints(true); });

        if(!regionMarker->isEditing()){
            SceneWidget* sceneWidget = event->sceneWidget();
            eraserModeMenuItemConnection.reset(
                menu->addItem(_("PointSet: Start Eraser Mode"))->sigTriggered().connect(
                    [&, sceneWidget](){ regionMarker->startEditing(sceneWidget); }));
        }
        return true;
    }
    return false;
}


void ScenePointSet::onContextMenuRequestInEraserMode(SceneWidgetEvent* event)
{
    auto item = event->contextMenu()->addItem(_("PointSet: Exit Eraser Mode"))
        ->sigTriggered().connect([&](){ regionMarker->finishEditing(); });
    eraserModeMenuItemConnection.reset(item);
}


void ScenePointSet::onRegionFixed(const PolyhedralRegion& region)
{
    PointSetItem* item = weakPointSetItem.lock();
    if(item){
        item->removePoints(region);
    }
}


PointSetLocation::PointSetLocation(PointSetItem* item)
    : LocationProxy(GlobalLocation),
      item(item)
{

}


std::string PointSetLocation::getName() const
{
    return fmt::format(_("Offset position of {0}"), item->displayName());
}


Isometry3 PointSetLocation::getLocation() const
{
    return item->offsetPosition();
}


bool PointSetLocation::setLocation(const Isometry3& T)
{
    item->setOffsetPosition(T);
    item->notifyUpdate();
    return true;
}


SignalProxy<void()> PointSetLocation::sigLocationChanged()
{
    return item->sigOffsetPositionChanged();
}
