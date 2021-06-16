/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "MultiPointSetItem.h"
#include "ItemManager.h"
#include "MenuManager.h"
#include "RootItem.h"
#include "SceneWidget.h"
#include "SceneWidgetEventHandler.h"
#include "PutPropertyFunction.h"
#include "Archive.h"
#include <cnoid/PointSetUtil>
#include <cnoid/EigenArchive>
#include <cnoid/YAMLReader>
#include <cnoid/YAMLWriter>
#include <cnoid/SceneMarkers>
#include <cnoid/Exception>
#include <cnoid/UTF8>
#include <cnoid/stdx/filesystem>
#include "gettext.h"

using namespace std;
using namespace cnoid;
namespace filesystem = cnoid::stdx::filesystem;

namespace {

class SceneMultiPointSet : public SgPosTransform, public SceneWidgetEventHandler
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    weak_ref_ptr<MultiPointSetItem> weakMultiPointSetItem;
    bool isEditable;
    SgGroupPtr pointSetGroup;
    SgGroupPtr attentionPointMarkerGroup;
    Signal<void(const Isometry3& T)> sigOffsetPositionChanged;
    Signal<void()> sigAttentionPointsChanged;
    
    RectRegionMarkerPtr regionMarker;
    ScopedConnection eraserModeMenuItemConnection;

    SceneMultiPointSet(MultiPointSetItem::Impl* multiPointSetItem);

    int numAttentionPoints() const;
    Vector3 attentionPoint(int index) const;
    void clearAttentionPoints(bool doNotify);
    void addAttentionPoint(const Vector3& point, bool doNotify);
    void setAttentionPoint(const Vector3& p, bool doNotify);
    bool removeAttentionPoint(const Vector3& point, double distanceThresh, bool doNotify);
    void notifyAttentionPointChange();

    virtual bool onButtonPressEvent(SceneWidgetEvent* event) override;
    virtual bool onPointerMoveEvent(SceneWidgetEvent* event) override;
    virtual bool onContextMenuRequest(SceneWidgetEvent* event, MenuManager* menuManager) override;

    void onContextMenuRequestInEraserMode(SceneWidgetEvent* event, MenuManager* menuManager);
    void onRegionFixed(const PolyhedralRegion& region);    
};

typedef ref_ptr<SceneMultiPointSet> SceneMultiPointSetPtr;

}

namespace cnoid {
        
class MultiPointSetItem::Impl
{
public:
    MultiPointSetItem* self;

    struct PointSetItemInfo : public Referenced {
        int index;
        ScopedConnection pointSetUpdateConnection;
            
    };
    typedef ref_ptr<PointSetItemInfo> PointSetItemInfoPtr;

    typedef map<PointSetItem*, PointSetItemInfoPtr> ItemInfoMap;
    ItemInfoMap itemInfoMap;

    ItemList<PointSetItem> pointSetItems;
    Selection visibilityMode;
    ItemList<PointSetItem> lastSelectedPointSetItems;
    
    SceneMultiPointSetPtr scene;
    Selection renderingMode;
    double pointSize;
    double voxelSize;
    ScopedConnection itemSelectionChangedConnection;
    ScopedConnection subTreeChangedConnection;
    Signal<void(int index)> sigPointSetItemAdded;
    Signal<void(int index)> sigPointSetItemUpdated;

    MappingPtr outputArchive;
    ListingPtr outputFileListing;
    bool isAutoSaveMode;
    filesystem::path autoSaveFilePath;

    Impl(MultiPointSetItem* self);
    Impl(MultiPointSetItem* self, const Impl& org);
    void updateVisibilities();
    void onSelectedItemsChanged();
    void onSubTreeChanged();
    void onPointSetUpdated(PointSetItem* item);
    void setRenderingMode(int mode);
    void setPointSize(double size);
    void setVoxelSize(double size);
    void selectSinglePointSetItem(int index);
    bool onRenderingModePropertyChanged(int mode);
    bool onTopTranslationPropertyChanged(const std::string& value);
    bool onTopRotationPropertyChanged(const std::string& value);

    static bool loadItem(MultiPointSetItem* item, const std::string& filename);
    bool load(const std::string& filename);
    static bool saveItem(MultiPointSetItem* item, const std::string& filename);
    bool save(const std::string& filename);
    bool outputPointSetItem(int index);
    bool writeOutputArchive(const std::string& filename);
    bool startAutomaticSave(const std::string& filename);
    void saveAdditionalPointSet(int index);
};

}

void MultiPointSetItem::initializeClass(ExtensionManager* ext)
{
    static bool initialized = false;
    if(!initialized){
        ItemManager& im = ext->itemManager();
        im.registerClass<MultiPointSetItem>(N_("MultiPointSetItem"));
        im.addCreationPanel<MultiPointSetItem>();
        im.addLoaderAndSaver<MultiPointSetItem>(
            _("Multi Point Sets"), "MULTI-PCD-SET", "yaml",
            [](MultiPointSetItem* item, const std::string& filename, std::ostream& os, Item*){
                return MultiPointSetItem::Impl::loadItem(item, filename); },
            [](MultiPointSetItem* item, const std::string& filename, std::ostream& os, Item*){
                return MultiPointSetItem::Impl::saveItem(item, filename); },
            ItemManager::PRIORITY_DEFAULT);
        initialized = true;
    }
}


MultiPointSetItem::MultiPointSetItem()
{
    impl = new Impl(this);
}


MultiPointSetItem::Impl::Impl(MultiPointSetItem* self)
    : self(self),
      visibilityMode(NumVisibilityModes, CNOID_GETTEXT_DOMAIN_NAME),
      renderingMode(NumRenderingModes, CNOID_GETTEXT_DOMAIN_NAME)
{
    visibilityMode.setSymbol(ShowAll, N_("All"));
    visibilityMode.setSymbol(ShowSelected, N_("Selected"));
    visibilityMode.select(ShowAll);
    
    renderingMode.setSymbol(Point, N_("Point"));
    renderingMode.setSymbol(Voxel, N_("Voxel"));
    renderingMode.select(Point);
    pointSize = 0.0;
    voxelSize = PointSetItem::defaultVoxelSize();
    
    scene = new SceneMultiPointSet(this);

    itemSelectionChangedConnection.reset(
        RootItem::instance()->sigSelectedItemsChanged().connect(
            [=](const ItemList<>&){ onSelectedItemsChanged(); }));

    subTreeChangedConnection.reset(
        self->sigSubTreeChanged().connect(
            [&](){ onSubTreeChanged(); }));

    isAutoSaveMode = false;
}


MultiPointSetItem::MultiPointSetItem(const MultiPointSetItem& org)
    : Item(org)
{
    impl = new Impl(this, *org.impl);
}


MultiPointSetItem::Impl::Impl(MultiPointSetItem* self, const Impl& org)
    : Impl(self)
{
    visibilityMode.select(org.visibilityMode.which());
    renderingMode.select(org.renderingMode.which());
}


MultiPointSetItem::~MultiPointSetItem()
{
    delete impl;
}


SgNode* MultiPointSetItem::getScene()
{
    return impl->scene;
}


void MultiPointSetItem::setVisibilityMode(int mode)
{
    impl->visibilityMode.select(mode);
    impl->updateVisibilities();
}


void MultiPointSetItem::Impl::updateVisibilities()
{
    ItemList<PointSetItem>* pItemList;
    if(visibilityMode.is(MultiPointSetItem::ShowAll)){
        pItemList = &pointSetItems;
    } else if(visibilityMode.is(MultiPointSetItem::ShowSelected)){
        pItemList = &lastSelectedPointSetItems;
    }

    scene->pointSetGroup->clearChildren();
    for(auto& item : *pItemList){
        scene->pointSetGroup->addChild(item->getScene());
    }
    scene->notifyUpdate(SgUpdate::ADDED | SgUpdate::REMOVED);
}


void MultiPointSetItem::Impl::onSelectedItemsChanged()
{
    auto selected = self->selectedDescendantItems<PointSetItem>();
    if(!selected.empty() && selected != lastSelectedPointSetItems){
        lastSelectedPointSetItems = selected;
        updateVisibilities();
    }
}


void MultiPointSetItem::Impl::onSubTreeChanged()
{
    auto newPointSetItems = self->descendantItems<PointSetItem>();
    if(newPointSetItems == pointSetItems){
        return;
    }
    pointSetItems = newPointSetItems;

    ItemInfoMap prevMap(itemInfoMap);
    itemInfoMap.clear();

    for(size_t i=0; i < pointSetItems.size(); ++i){
        PointSetItem* item = pointSetItems[i];
        ItemInfoMap::iterator p = prevMap.find(item);
        if(p != prevMap.end()){
            p->second->index = i;
            itemInfoMap.insert(*p);
        } else {
            item->setPointSize(pointSize);
            item->setVoxelSize(voxelSize);
            item->setRenderingMode(renderingMode.which());
            
            PointSetItemInfo* info = new PointSetItemInfo;
            info->index = i;
            info->pointSetUpdateConnection.reset(
                item->pointSet()->sigUpdated().connect(
                    [&, item](const SgUpdate&){ onPointSetUpdated(item); }));
            itemInfoMap.insert(ItemInfoMap::value_type(item, info));
            
            sigPointSetItemAdded(i);
            
            if(isAutoSaveMode){
                saveAdditionalPointSet(i);
            }
        }
    }

    if(visibilityMode.is(MultiPointSetItem::ShowSelected)){
        onSelectedItemsChanged();
    } else {
        updateVisibilities();
    }
}


void MultiPointSetItem::Impl::onPointSetUpdated(PointSetItem* item)
{
    ItemInfoMap::iterator p = itemInfoMap.find(item);
    if(p != itemInfoMap.end()){
        int index = p->second->index;
        sigPointSetItemUpdated(index);
    }
}


void MultiPointSetItem::setRenderingMode(int mode)
{
    impl->setRenderingMode(mode);
}


void MultiPointSetItem::Impl::setRenderingMode(int mode)
{
    renderingMode.select(mode);
    for(size_t i=0; i < pointSetItems.size(); ++i){
        pointSetItems[i]->setRenderingMode(mode);
    }
}


int MultiPointSetItem::renderingMode() const
{
    return impl->renderingMode.which();
}


double MultiPointSetItem::pointSize() const
{
    return impl->pointSize;
}
    

void MultiPointSetItem::setPointSize(double size)
{
    impl->setPointSize(size);
}


void MultiPointSetItem::Impl::setPointSize(double size)
{
    pointSize = size;
    for(size_t i=0; i < pointSetItems.size(); ++i){
        pointSetItems[i]->setPointSize(size);
    }
}


double MultiPointSetItem::voxelSize() const
{
    return impl->voxelSize;
}
    

void MultiPointSetItem::setVoxelSize(double size)
{
    impl->setVoxelSize(size);
}


void MultiPointSetItem::Impl::setVoxelSize(double size)
{
    voxelSize = size;
    for(size_t i=0; i < pointSetItems.size(); ++i){
        pointSetItems[i]->setVoxelSize(size);
    }
}


int MultiPointSetItem::numPointSetItems() const
{
    return impl->pointSetItems.size();
}


PointSetItem* MultiPointSetItem::pointSetItem(int index)
{
    return impl->pointSetItems[index];
}


const PointSetItem* MultiPointSetItem::pointSetItem(int index) const
{
    return impl->pointSetItems[index];
}


int MultiPointSetItem::numVisiblePointSetItems() const
{
    if(impl->visibilityMode.is(ShowAll)){
        return impl->pointSetItems.size();
    } else {
        return impl->lastSelectedPointSetItems.size();
    }
}


int MultiPointSetItem::numActivePointSetItems() const
{
    return numVisiblePointSetItems();
}


PointSetItem* MultiPointSetItem::visiblePointSetItem(int index)
{
    if(impl->visibilityMode.is(ShowAll)){
        return impl->pointSetItems[index];
    } else {
        return impl->lastSelectedPointSetItems[index];
    }
}


PointSetItem* MultiPointSetItem::activePointSetItem(int index)
{
    return visiblePointSetItem(index);
}


void MultiPointSetItem::selectSinglePointSetItem(int index)
{
    impl->selectSinglePointSetItem(index);
}


void MultiPointSetItem::Impl::selectSinglePointSetItem(int index)
{
    if(index < 0 || index >= static_cast<int>(pointSetItems.size())){
        return;
    }
    
    itemSelectionChangedConnection.block();
    for(int i=0; i < pointSetItems.size(); ++i){
        pointSetItems[i]->setSelected(i == index);
    }
    itemSelectionChangedConnection.unblock();

    onSelectedItemsChanged();
}


SignalProxy<void(int index)> MultiPointSetItem::sigPointSetItemAdded()
{
    return impl->sigPointSetItemAdded;
}


SignalProxy<void(int index)> MultiPointSetItem::sigPointSetUpdated()
{
    return impl->sigPointSetItemUpdated;
}


const Isometry3& MultiPointSetItem::offsetPosition() const
{
    return impl->scene->T();
}


void MultiPointSetItem::setOffsetPosition(const Isometry3& T)
{
    impl->scene->setPosition(T);
}


SignalProxy<void(const Isometry3& T)> MultiPointSetItem::sigOffsetPositionChanged()
{
    return impl->scene->sigOffsetPositionChanged;
}


void MultiPointSetItem::notifyOffsetPositionChange()
{
    impl->scene->sigOffsetPositionChanged(impl->scene->T());
    impl->scene->notifyUpdate();
    Item::notifyUpdate();
}


Isometry3 MultiPointSetItem::totalOffsetPositionOf(int index) const
{
    return offsetPosition() * pointSetItem(index)->offsetPosition();
}


SgPointSetPtr MultiPointSetItem::getTransformedPointSet(int index) const
{
    SgPointSetPtr transformed = new SgPointSet;
    SgVertexArray& points = *transformed->getOrCreateVertices();
    const SgVertexArray* orgPoints = pointSetItem(index)->pointSet()->vertices();
    if(orgPoints){
        const int n = orgPoints->size();
        points.resize(n);
        const Isometry3 T = totalOffsetPositionOf(index);//.cast<Isometry3::Scalar>();
        for(int i=0; i < n; ++i){
            points[i] = (T * (*orgPoints)[i].cast<Isometry3::Scalar>()).cast<SgVertexArray::Scalar>();
        }
    }
    return transformed;
}
    

int MultiPointSetItem::numAttentionPoints() const
{
    return impl->scene->numAttentionPoints();
}
        

Vector3 MultiPointSetItem::attentionPoint(int index) const
{
    return impl->scene->attentionPoint(index);
}


void MultiPointSetItem::clearAttentionPoints()
{
    return impl->scene->clearAttentionPoints(false);
}


void MultiPointSetItem::addAttentionPoint(const Vector3& p)
{
    impl->scene->addAttentionPoint(p, false);
}


SignalProxy<void()> MultiPointSetItem::sigAttentionPointsChanged()
{
    return impl->scene->sigAttentionPointsChanged;
}


void MultiPointSetItem::notifyAttentionPointChange()
{
    impl->scene->notifyAttentionPointChange();
}


Item* MultiPointSetItem::doDuplicate() const
{
    return new MultiPointSetItem(*this);
}


void MultiPointSetItem::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("Visibility"), impl->visibilityMode,
                [&](int mode){
                    setVisibilityMode(mode); impl->updateVisibilities(); return true; });
    putProperty(_("Auto save"), false);
    putProperty(_("Num point sets"), numPointSetItems());
    putProperty(_("Rendering mode"), impl->renderingMode,
                [&](int mode){ return impl->onRenderingModePropertyChanged(mode); });
    putProperty.decimals(1).min(0.0)(_("Point size"), pointSize(),
                                     [&](double size){ setPointSize(size); return true; });
    putProperty.decimals(4)(_("Voxel size"), voxelSize(),
                            [&](double size){ setVoxelSize(size); return true; });
    putProperty(_("Translation"), str(Vector3(offsetPosition().translation())),
                [&](const string& value){ return impl->onTopTranslationPropertyChanged(value); });
    Vector3 rpy(TO_DEGREE * rpyFromRot(offsetPosition().linear()));
    putProperty(_("Rotation"), str(rpy), [&](const string& value){ return impl->onTopRotationPropertyChanged(value); });
}


bool MultiPointSetItem::Impl::onRenderingModePropertyChanged(int mode)
{
    if(mode != renderingMode.which()){
        if(renderingMode.select(mode)){
            setRenderingMode(mode);
            return true;
        }
    }
    return false;
}


bool MultiPointSetItem::Impl::onTopTranslationPropertyChanged(const std::string& value)
{
    Vector3 p;
    if(toVector3(value, p)){
        scene->setTranslation(p);
        self->notifyOffsetPositionChange();
        return true;
    }
    return false;
}


bool MultiPointSetItem::Impl::onTopRotationPropertyChanged(const std::string& value)
{
    Vector3 rpy;
    if(toVector3(value, rpy)){
        scene->setRotation(rotFromRpy(TO_RADIAN * rpy));
        self->notifyOffsetPositionChange();
        return true;
    }
    return false;
}


bool MultiPointSetItem::store(Archive& archive)
{
    archive.write("visibilityMode", impl->visibilityMode.selectedSymbol());
    archive.write("autoSave", false);

    auto& scene = impl->scene;
    write(archive, "translation", Vector3(scene->translation()));

    writeDegreeAngleAxis(archive, "rotation", AngleAxis(scene->rotation()));
    // The following element is used to distinguish the value type from the old one using radian.
    // The old format is deprecated, and writing the following element should be omitted in the future.
    archive.write("angle_unit", "degree");

    archive.write("renderingMode", impl->renderingMode.selectedSymbol());
    archive.write("pointSize", pointSize());
    archive.write("voxelSize", voxelSize());
    return true;
}


bool MultiPointSetItem::restore(const Archive& archive)
{
    string symbol;
    if(archive.read("visibilityMode", symbol)){
        setVisibilityMode(impl->visibilityMode.index(symbol));
    }
    
    archive.get("autoSave", false);

    auto& scene = impl->scene;
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
    
    if(archive.read("renderingMode", symbol)){
        impl->onRenderingModePropertyChanged(impl->renderingMode.index(symbol));
    }
    setPointSize(archive.get("pointSize", pointSize()));
    setVoxelSize(archive.get("voxelSize", voxelSize()));
    
    return true;
}


namespace {

SceneMultiPointSet::SceneMultiPointSet(MultiPointSetItem::Impl* multiPointSetItem)
    : weakMultiPointSetItem(multiPointSetItem->self)
{
    pointSetGroup = new SgGroup;
    addChild(pointSetGroup);

    attentionPointMarkerGroup = new SgGroup;
    addChild(attentionPointMarkerGroup);

    regionMarker = new RectRegionMarker;
    regionMarker->setEditModeCursor(QCursor(QPixmap(":/Base/icon/eraser-cursor.png"), 3, 2));
    regionMarker->sigRegionFixed().connect(
        [&](const PolyhedralRegion& region){ onRegionFixed(region); });
    regionMarker->sigContextMenuRequest().connect(
        [&](SceneWidgetEvent* event, MenuManager* manager){
            onContextMenuRequestInEraserMode(event, manager); });
    isEditable = true;
}


int SceneMultiPointSet::numAttentionPoints() const
{
    return attentionPointMarkerGroup->numChildren();
}


Vector3 SceneMultiPointSet::attentionPoint(int index) const
{
    if(index < numAttentionPoints()){
        CrossMarker* marker = dynamic_cast<CrossMarker*>(attentionPointMarkerGroup->child(index));
        if(marker){
            return T() * marker->translation();
        }
    }
    return Vector3::Zero();
}


void SceneMultiPointSet::clearAttentionPoints(bool doNotify)
{
    if(!attentionPointMarkerGroup->empty()){
        attentionPointMarkerGroup->clearChildren();
        if(doNotify){
            notifyAttentionPointChange();
        }
    }
}


void SceneMultiPointSet::addAttentionPoint(const Vector3& point, bool doNotify)
{
    Vector3f color(1.0f, 1.0f, 0.0f);
    CrossMarker* marker = new CrossMarker(0.02, color);
    marker->setTranslation(T().inverse() * point);
    attentionPointMarkerGroup->addChild(marker);

    if(doNotify){
        notifyAttentionPointChange();
    }
}


void SceneMultiPointSet::setAttentionPoint(const Vector3& p, bool doNotify)
{
    clearAttentionPoints(false);
    addAttentionPoint(p, doNotify);
}


bool SceneMultiPointSet::removeAttentionPoint(const Vector3& point, double distanceThresh, bool doNotify)
{
    bool removed = false;

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
    if(removed && doNotify){
        notifyAttentionPointChange();
    }
    return removed;
}


void SceneMultiPointSet::notifyAttentionPointChange()
{
    attentionPointMarkerGroup->notifyUpdate();
    sigAttentionPointsChanged();
}

    
bool SceneMultiPointSet::onButtonPressEvent(SceneWidgetEvent* event)
{
	if(!isEditable){
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


bool SceneMultiPointSet::onPointerMoveEvent(SceneWidgetEvent*)
{
    return false;
}


bool SceneMultiPointSet::onContextMenuRequest(SceneWidgetEvent* event, MenuManager* menuManager)
{
    menuManager->addItem(_("PointSet: Clear Attention Points"))->sigTriggered().connect(
        [&](){ clearAttentionPoints(true); });

    if(!regionMarker->isEditing()){
        SceneWidget* sceneWidget = event->sceneWidget();
        eraserModeMenuItemConnection.reset(
            menuManager->addItem(_("PointSet: Start Eraser Mode"))->sigTriggered().connect(
                [&, sceneWidget](){ regionMarker->startEditing(sceneWidget); }));
    }

    return true;
}


void SceneMultiPointSet::onContextMenuRequestInEraserMode(SceneWidgetEvent*, MenuManager* menuManager)
{
    eraserModeMenuItemConnection.reset(
        menuManager->addItem(_("PointSet: Exit Eraser Mode"))->sigTriggered().connect(
            [&](){ regionMarker->finishEditing(); }));
}


void SceneMultiPointSet::onRegionFixed(const PolyhedralRegion& region)
{
    MultiPointSetItem* item = weakMultiPointSetItem.lock();
    if(item){
        int n = item->numVisiblePointSetItems();
        for(int i=0; i < n; ++i){
            item->activePointSetItem(i)->removePoints(region);
        }
    }
}

}


bool MultiPointSetItem::Impl::loadItem(MultiPointSetItem* item, const std::string& filename)
{
    return item->impl->load(filename);
}


bool MultiPointSetItem::Impl::load(const std::string& filename)
{
    YAMLReader reader;
    if(reader.load(filename)){
        const Mapping& archive = *reader.document()->toMapping();
        const Listing& files = *archive.findListing("files");
        if(files.isValid()){
            auto directory = filesystem::path(fromUTF8(filename)).parent_path();
            for(int i=0; i < files.size(); ++i){
                const Mapping& info = *files[i].toMapping();
                string pcdFilename;
                if(info.read("file", pcdFilename)){
                    filesystem::path path(fromUTF8(pcdFilename));
                    PointSetItemPtr childItem = new PointSetItem();
                    if(childItem->load(toUTF8((directory / path).string()), self, "PCD-FILE")){
                        childItem->setName(toUTF8(path.stem().string()));
                        Isometry3 T;
                        if(read(info, "offsetTransform", T)){
                            childItem->setOffsetPosition(T);
                        }
                        self->addSubItem(childItem);
                    }
                }
            }
        }
    }

    return true;
}


bool MultiPointSetItem::Impl::saveItem(MultiPointSetItem* item, const std::string& filename)
{
    return item->impl->save(filename);
}


bool MultiPointSetItem::Impl::save(const std::string& filename)
{
    outputArchive = new Mapping();
    outputArchive->setFloatingNumberFormat("%.9g");

    outputArchive->write("type", "MultiPointSet");
    outputArchive->write("fileFormat", "PCD");

    outputFileListing = outputArchive->createListing("files");
    const int n = self->numPointSetItems();
    for(int i=0; i < n; ++i){
        outputPointSetItem(i);
    }

    return writeOutputArchive(filename);
}


bool MultiPointSetItem::Impl::outputPointSetItem(int index)
{
    bool result = false;
    
    PointSetItem* item = self->pointSetItem(index);
    if(outputFileListing && !item->name().empty()){
        string filename = item->name() + ".pcd";
        filesystem::path path(fromUTF8(filename));
        string fullPathString = toUTF8((autoSaveFilePath.parent_path() / path).string());

        try {
            cnoid::savePCD(item->pointSet(), fullPathString, item->offsetPosition());

            MappingPtr info = new Mapping();
            info->write("file", filename);
            info->setFloatingNumberFormat("%.9g");
            write(*info, "offsetTransform", item->offsetPosition());
            outputFileListing->insert(index, info);
            result = true;

        } catch (boost::exception& ex) {
            if(std::string const * message = boost::get_error_info<error_info_message>(ex)){
                mvout() << *message << endl;
            }
        }
    }

    return result;
}


bool MultiPointSetItem::Impl::writeOutputArchive(const std::string& filename)
{
    if(outputArchive && !outputFileListing->empty()){
        YAMLWriter writer(filename);
        writer.setKeyOrderPreservationMode(true);
        writer.putComment("Multi point set data format version 1.0 defined by Choreonoid\n");
        writer.putNode(outputArchive);
        return true;
    }
    return false;
}


bool MultiPointSetItem::startAutomaticSave(const std::string& filename)
{
    return impl->startAutomaticSave(filename);
}


bool MultiPointSetItem::Impl::startAutomaticSave(const std::string& filename)
{
    isAutoSaveMode = false;

    autoSaveFilePath = filesystem::absolute(fromUTF8(filename));

    if(!autoSaveFilePath.filename().empty()){
        if(self->numPointSetItems() == 0){
            isAutoSaveMode = true;
        } else {
            try {
                if(filesystem::create_directories(autoSaveFilePath.parent_path())){
                    if(save(toUTF8(autoSaveFilePath.string()))){
                        isAutoSaveMode = true;
                    }
                }
            }
            catch(const filesystem::filesystem_error& ex){
                mvout() << ex.what() << endl;
            }
        }
    }

    return isAutoSaveMode;
}


void MultiPointSetItem::Impl::saveAdditionalPointSet(int index)
{
    if(!outputArchive){
        try {
            if(filesystem::create_directories(autoSaveFilePath.parent_path())){
                save(toUTF8(autoSaveFilePath.string()));
            }
        }
        catch(const filesystem::filesystem_error& ex){
            mvout() << ex.what() << endl;
        }
    } else {
        if(outputPointSetItem(index)){
            writeOutputArchive(toUTF8(autoSaveFilePath.string()));
        }
    }
}


void MultiPointSetItem::stopAutomaticSave()
{
    impl->outputArchive.reset();
    impl->outputFileListing.reset();
    impl->isAutoSaveMode = false;
}
