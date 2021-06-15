#include "PositionTagGroupItem.h"
#include "ItemManager.h"
#include "SceneWidget.h"
#include "SceneWidgetEventHandler.h"
#include "PositionDragger.h"
#include "CoordinateFrameMarker.h"
#include "Dialog.h"
#include "Buttons.h"
#include "CheckBox.h"
#include "LazyCaller.h"
#include "Archive.h"
#include "PutPropertyFunction.h"
#include "MenuManager.h"
#include "UnifiedEditHistory.h"
#include "EditRecord.h"
#include <cnoid/PositionTagGroup>
#include <cnoid/SceneDrawables>
#include <cnoid/ConnectionSet>
#include <cnoid/EigenArchive>
#include <fmt/format.h>
#include <QBoxLayout>
#include <QLabel>
#include <QDialogButtonBox>
#include <map>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

namespace {

map<PositionTagGroupPtr, PositionTagGroupItem*> tagGroupToItemMap;

enum LocationMode { TagGroupLocation, TagLocation };
enum TagDisplayType { Normal, Selected, Highlighted };

constexpr float HighlightSizeRatio = 1.1f;


class SceneTag : public SgPosTransform
{
public:
    SceneTag() : displayType(Normal) { }
    TagDisplayType displayType;
};


class SceneTagGroup : public SgPosTransform, public SceneWidgetEventHandler
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    PositionTagGroupItem::Impl* impl;
    SgPosTransformPtr offsetTransform;
    SgGroupPtr tagMarkerGroup;
    SgNodePtr tagMarker;
    SgNodePtr selectedTagMarker;
    SgNodePtr highlightedTagMarker;
    int highlightedTagIndex;
    SgLineSetPtr markerLineSet;
    CoordinateFrameMarkerPtr originMarker;
    SgLineSetPtr edgeLineSet;
    SgUpdate update;
    ScopedConnectionSet tagGroupConnections;

    PositionDraggerPtr positionDragger;
    int draggingTagIndex;
    Isometry3 initialDraggerPosition;

    struct TagPosition
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        int index;
        Isometry3 T;
        TagPosition(int index, const Isometry3& T) : index(index), T(T) { }
    };
    vector<TagPosition, Eigen::aligned_allocator<TagPosition>> initialTagDragPositions;
    
    SceneTagGroup(PositionTagGroupItem::Impl* itemImpl);
    void finalize();
    void addTagNode(int index, bool doUpdateEdges, bool doNotify);
    SgNode* getOrCreateTagMarker();
    SgNode* getOrCreateSelectedTagMarker();
    SgNode* getOrCreateHighlightedTagMarker();
    void removeTagNode(int index);
    void updateTagNodePosition(int index);
    void updateTagDisplayTypes();
    bool updateTagDisplayType(int index, bool doNotify);
    void updateEdges(bool doNotify);
    void setOriginOffset(const Isometry3& T);
    void setParentPosition(const Isometry3& T);
    void setOriginMarkerVisibility(bool on);
    void setEdgeVisiblility(bool on);
    void setHighlightedTagIndex(int index);
    void attachPositionDragger(int tagIndex);
    void onDraggerDragStarted();
    void onDraggerDragged();
    void onDraggerDragFinished();
    int findPointingTagIndex(SceneWidgetEvent* event);
    virtual bool onPointerMoveEvent(SceneWidgetEvent* event) override;
    virtual void onPointerLeaveEvent(SceneWidgetEvent* event) override;
    virtual bool onButtonPressEvent(SceneWidgetEvent* event) override;
    virtual void onFocusChanged(SceneWidgetEvent* event, bool on) override;
    virtual bool onContextMenuRequest(SceneWidgetEvent* event, MenuManager* menu) override;
};

typedef ref_ptr<SceneTagGroup> SceneTagGroupPtr;


class TargetLocationProxy : public LocationProxy
{
public:
    PositionTagGroupItem::Impl* impl;

    TargetLocationProxy(PositionTagGroupItem::Impl* impl);
    const PositionTag* getTargetTag() const;
    virtual std::string getName() const override;
    virtual Item* getCorrespondingItem() override;
    virtual Isometry3 getLocation() const override;
    virtual bool setLocation(const Isometry3& T) override;
    virtual void finishLocationEditing() override;
    virtual SignalProxy<void()> sigLocationChanged() override;
    virtual LocationProxyPtr getParentLocationProxy() const override;
};

typedef ref_ptr<TargetLocationProxy> TargetLocationProxyPtr;


class TagParentLocationProxy : public LocationProxy
{
public:
    PositionTagGroupItem::Impl* impl;

    TagParentLocationProxy(PositionTagGroupItem::Impl* impl);
    virtual std::string getName() const override;
    virtual Isometry3 getLocation() const override;
    virtual SignalProxy<void()> sigLocationChanged() override;
    virtual LocationProxyPtr getParentLocationProxy() const override;
};

typedef ref_ptr<TagParentLocationProxy> TagParentLocationProxyPtr;


class OffsetEditRecord : public EditRecord
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    PositionTagGroupItemPtr tagGroupItem;
    Isometry3 T_new;
    Isometry3 T_old;

    OffsetEditRecord(
        PositionTagGroupItem* tagGroupItem, const Isometry3& T_new, const Isometry3& T_old);
    OffsetEditRecord(const OffsetEditRecord& org);
    virtual EditRecord* clone() const override;
    virtual std::string label() const override;
    virtual bool undo() override;
    virtual bool redo() override;
};
    

enum TagAction { AddAction = 1, UpdateAction, RemoveAction };

class TagEditRecord : public EditRecord
{
public:
    PositionTagGroupItemPtr tagGroupItem;
    PositionTagGroupItem::Impl* tagGroupItemImpl;
    TagAction action;
    int tagIndex;
    PositionTagPtr newTag;
    PositionTagPtr oldTag;
    
    TagEditRecord(PositionTagGroupItem::Impl* tagGroupItemImpl, TagAction action, int tagIndex,
                  PositionTag* newTag, PositionTag* oldTag);
    TagEditRecord(const TagEditRecord& org);
    void setTags(PositionTag* newTag0, PositionTag* oldTag0);
    virtual EditRecord* clone() const override;
    virtual std::string label() const override;
    virtual bool undo() override;
    virtual bool redo() override;
};
    

class ConversionDialog : public Dialog
{
public:
    QLabel descriptionLabel;
    RadioButton globalCoordRadio;
    RadioButton localCoordRadio;
    CheckBox clearOriginOffsetCheck;
    
    ConversionDialog();
    void setTargets(PositionTagGroupItem* tagGroupItem, LocationProxyPtr newParentLocation);
};

}

namespace cnoid {

class PositionTagGroupItem::Impl
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    PositionTagGroupItem* self;
    PositionTagGroupPtr tagGroup;
    Isometry3 T_parent;
    Isometry3 T_offset;
    ScopedConnectionSet tagGroupConnections;
    Signal<void()> sigOriginOffsetPreviewRequested;
    Signal<void()> sigOriginOffsetUpdated;
    LazyCaller notifyUpdateLater;
    UnifiedEditHistory* history;
    Isometry3 lastEdit_T_offset;
    PositionTagGroupPtr lastEditTagGroup;
    bool isDoingUndoOrRedo;
    bool needToUpdateSelectedTagIndices;
    std::vector<bool> tagSelection;
    int numSelectedTags;
    std::vector<int> selectedTagIndices;
    Signal<void()> sigTagSelectionChanged;
    LocationMode locationMode;
    int locationTargetTagIndex;
    TargetLocationProxyPtr targetLocation;
    Signal<void()> sigTargetLocationChanged;
    LocationProxyPtr groupParentLocation;
    ScopedConnection groupParentLocationConnection;
    TagParentLocationProxyPtr tagParentLocation;
    Signal<void()> sigTagParentLocationChanged;
    SceneTagGroupPtr sceneTagGroup;
    SgFixedPixelSizeGroupPtr fixedSizeTagMarker;
    SgFixedPixelSizeGroupPtr fixedSizeSelectedTagMarker;
    SgFixedPixelSizeGroupPtr fixedSizeHighlightedTagMarker;
    SgMaterialPtr material;
    bool originMarkerVisibility;
    bool edgeVisibility;
    
    Impl(PositionTagGroupItem* self, const Impl* org);
    void setupHandlersForUnifiedEditHistory();
    void onTagAdded(int index);
    void onTagRemoved(int index, PositionTag* tag);
    void onTagPositionUpdated(int index);
    bool clearTagSelection(bool doNotify);
    void selectAllTags();
    void setTagSelected(int tagIndex, bool on, bool doNotify);
    bool checkTagSelected(int tagIndex) const;
    void onTagSelectionChanged(bool doUpdateTagDisplayTypes);
    void removeSelectedTags();
    void setParentItemLocationProxy(
        LocationProxyPtr newParentLocation, bool doCoordinateConversion, bool doClearOriginOffset);
    void convertLocalCoordinates(
        LocationProxy* currentParentLocation, LocationProxy* newParentLocation, bool doClearOriginOffset); 
    void onParentItemLocationChanged();
};

}


void PositionTagGroupItem::initializeClass(ExtensionManager* ext)
{
    static bool initialized = false;
    if(!initialized){
        auto& im = ext->itemManager();
        im.registerClass<PositionTagGroupItem>(N_("PositionTagGroupItem"));
        im.addCreationPanel<PositionTagGroupItem>();
        initialized = true;
    }
}


PositionTagGroupItem* PositionTagGroupItem::findItemOf(PositionTagGroup* tagGroup)
{
    auto p = tagGroupToItemMap.find(tagGroup);
    if(p != tagGroupToItemMap.end()){
        return p->second;
    }
    return nullptr;
}


PositionTagGroupItem::PositionTagGroupItem()
{
    impl = new Impl(this, nullptr);
}


PositionTagGroupItem::PositionTagGroupItem(const PositionTagGroupItem& org)
    : Item(org)
{
    impl = new Impl(this, org.impl);
}


PositionTagGroupItem::Impl::Impl(PositionTagGroupItem* self, const Impl* org)
    : self(self),
      notifyUpdateLater([this](){ this->self->notifyUpdate(); })
{
    if(!org){
        tagGroup = new PositionTagGroup;
    } else {
        tagGroup = new PositionTagGroup(*org->tagGroup);
    }

    tagGroupToItemMap[tagGroup] = self;

    history = UnifiedEditHistory::instance();

    isDoingUndoOrRedo = false;
    needToUpdateSelectedTagIndices = false;
    numSelectedTags = 0;

    locationMode = TagGroupLocation;
    locationTargetTagIndex = -1;
    targetLocation = new TargetLocationProxy(this);
    tagParentLocation = new TagParentLocationProxy(this);

    fixedSizeTagMarker = new SgFixedPixelSizeGroup;
    fixedSizeSelectedTagMarker = new SgFixedPixelSizeGroup;
    fixedSizeHighlightedTagMarker = new SgFixedPixelSizeGroup;
    material = new SgMaterial;
    originMarkerVisibility = false;
    edgeVisibility = false;
    
    float s;
    if(!org){
        s = 24.0f;
        originMarkerVisibility = false;
        edgeVisibility = false;
        T_parent.setIdentity();
        T_offset.setIdentity();
    } else {
        s = org->fixedSizeTagMarker->pixelSizeRatio();
        originMarkerVisibility = org->originMarkerVisibility;
        edgeVisibility = org->edgeVisibility;
        T_parent = org->T_parent;
        T_offset = org->T_offset;
    }
    fixedSizeTagMarker->setPixelSizeRatio(s);
    fixedSizeSelectedTagMarker->setPixelSizeRatio(s * HighlightSizeRatio);
    fixedSizeHighlightedTagMarker->setPixelSizeRatio(s * HighlightSizeRatio);
}


PositionTagGroupItem::~PositionTagGroupItem()
{
    tagGroupToItemMap.erase(impl->tagGroup);
    delete impl;
}


Item* PositionTagGroupItem::doDuplicate() const
{
    return new PositionTagGroupItem(*this);
}


void PositionTagGroupItem::onConnectedToRoot()
{
    impl->setupHandlersForUnifiedEditHistory();
}


void PositionTagGroupItem::Impl::setupHandlersForUnifiedEditHistory()
{
    lastEdit_T_offset = T_offset;
    lastEditTagGroup = tagGroup->clone();
    
    tagGroupConnections.add(
        tagGroup->sigTagAdded().connect(
            [&](int index){ onTagAdded(index); }));
    tagGroupConnections.add(
        tagGroup->sigTagRemoved().connect(
            [&](int index, PositionTag* tag){ onTagRemoved(index, tag); }));
    tagGroupConnections.add(
        tagGroup->sigTagPositionUpdated().connect(
            [&](int index){ onTagPositionUpdated(index); }));
}


void PositionTagGroupItem::onDisconnectedFromRoot()
{
    impl->tagGroupConnections.disconnect();
}


bool PositionTagGroupItem::setName(const std::string& name)
{
    impl->tagGroup->setName(name);
    Item::setName(name);
    return true;
}


const PositionTagGroup* PositionTagGroupItem::tagGroup() const
{
    return impl->tagGroup;
}


PositionTagGroup* PositionTagGroupItem::tagGroup()
{
    return impl->tagGroup;
}


const Isometry3& PositionTagGroupItem::parentFramePosition() const
{
    return impl->T_parent;
}


const Isometry3& PositionTagGroupItem::originOffset() const
{
    return impl->T_offset;
}


void PositionTagGroupItem::setOriginOffset(const Isometry3& T_offset, bool requestPreview)
{
    impl->T_offset = T_offset;

    if(requestPreview){
        impl->sigOriginOffsetPreviewRequested();
        impl->sigTargetLocationChanged();
        if(impl->sceneTagGroup){
            impl->sceneTagGroup->setOriginOffset(T_offset);
        }
    }
}


Isometry3 PositionTagGroupItem::originPosition() const
{
    return impl->T_parent * impl->T_offset;
}


SignalProxy<void()> PositionTagGroupItem::sigOriginOffsetPreviewRequested()
{
    return impl->sigOriginOffsetPreviewRequested;
}


SignalProxy<void()> PositionTagGroupItem::sigOriginOffsetUpdated()
{
    return impl->sigOriginOffsetUpdated;
}


void PositionTagGroupItem::notifyOriginOffsetUpdate(bool requestPreview)
{
    if(requestPreview){
        impl->sigOriginOffsetPreviewRequested();
    }
    impl->sigOriginOffsetUpdated();

    impl->history->addRecord(new OffsetEditRecord(this, impl->T_offset, impl->lastEdit_T_offset));
    impl->lastEdit_T_offset = impl->T_offset;
}


void PositionTagGroupItem::Impl::onTagAdded(int index)
{
    if(index < static_cast<int>(tagSelection.size())){
        tagSelection.insert(tagSelection.begin() + index, false);
        needToUpdateSelectedTagIndices = true;
    }

    auto tag = tagGroup->tagAt(index);
    lastEditTagGroup->insert(index, new PositionTag(*tag));
    
    if(!isDoingUndoOrRedo){
        history->addRecord(new TagEditRecord(this, AddAction, index, tag, nullptr));
    }

    notifyUpdateLater();
}


void PositionTagGroupItem::Impl::onTagRemoved(int index, PositionTag* tag)
{
    if(index < static_cast<int>(tagSelection.size())){
        tagSelection.erase(tagSelection.begin() + index);
        needToUpdateSelectedTagIndices = true;
    }

    lastEditTagGroup->removeAt(index);

    if(!isDoingUndoOrRedo){
        history->addRecord(new TagEditRecord(this, RemoveAction, index, nullptr, tag));
    }
    
    notifyUpdateLater();
}


void PositionTagGroupItem::Impl::onTagPositionUpdated(int index)
{
    auto newTag = tagGroup->tagAt(index);
    auto lastTag = lastEditTagGroup->tagAt(index);

    if(!isDoingUndoOrRedo){
        history->addRecord(new TagEditRecord(this, UpdateAction, index, newTag, lastTag));
    }

    *lastTag = *newTag;

    if(locationMode == TagLocation && locationTargetTagIndex == index){
        sigTargetLocationChanged();
    }
    
    notifyUpdateLater();
}


void PositionTagGroupItem::clearTagSelection()
{
    impl->clearTagSelection(true);
}


bool PositionTagGroupItem::Impl::clearTagSelection(bool doNotify)
{
    bool doClear = (numSelectedTags > 0);
    tagSelection.clear();
    if(doClear){
        numSelectedTags = 0;
        needToUpdateSelectedTagIndices = true;
        if(doNotify){
            onTagSelectionChanged(true);
        }
    }
    return doClear;
}


void PositionTagGroupItem::Impl::selectAllTags()
{
    tagSelection.clear();
    numSelectedTags = tagGroup->numTags();
    tagSelection.resize(numSelectedTags, true);
    needToUpdateSelectedTagIndices = true;
    onTagSelectionChanged(true);
}


void PositionTagGroupItem::setTagSelected(int tagIndex, bool on)
{
    impl->setTagSelected(tagIndex, on, true);
}


void PositionTagGroupItem::Impl::setTagSelected(int tagIndex, bool on, bool doNotify)
{
    if(tagIndex >= tagSelection.size()){
        tagSelection.resize(tagIndex + 1);
    }
    if(on != tagSelection[tagIndex]){
        tagSelection[tagIndex]= on;
        if(on){
            ++numSelectedTags;
        } else {
            --numSelectedTags;
        }
        needToUpdateSelectedTagIndices = true;
        if(doNotify){
            if(sceneTagGroup){
                sceneTagGroup->updateTagDisplayType(tagIndex, true);
            }
            onTagSelectionChanged(false);
        }
    }
}


bool PositionTagGroupItem::checkTagSelected(int tagIndex) const
{
    return impl->checkTagSelected(tagIndex);
}


bool PositionTagGroupItem::Impl::checkTagSelected(int tagIndex) const
{
    if(tagIndex < tagSelection.size()){
        return tagSelection[tagIndex];
    }
    return false;
}


const std::vector<int>& PositionTagGroupItem::selectedTagIndices() const
{
    if(impl->needToUpdateSelectedTagIndices){
        impl->selectedTagIndices.clear();
        for(size_t i=0; i < impl->tagSelection.size(); ++i){
            if(impl->tagSelection[i]){
                impl->selectedTagIndices.push_back(i);
            }
        }
        impl->needToUpdateSelectedTagIndices = false;
    }
    return impl->selectedTagIndices;
}


void PositionTagGroupItem::setSelectedTagIndices(const std::vector<int>& indices)
{
    auto prevIndices = selectedTagIndices();
    if(indices != prevIndices){
        impl->clearTagSelection(false);
        for(auto& index : indices){
            impl->setTagSelected(index, true, false);
        }
        impl->onTagSelectionChanged(true);
    }
}


void PositionTagGroupItem::Impl::onTagSelectionChanged(bool doUpdateTagDisplayTypes)
{
    if(sceneTagGroup && doUpdateTagDisplayTypes){
        sceneTagGroup->updateTagDisplayTypes();
    }
    
    sigTagSelectionChanged();

    if(numSelectedTags == 0){
        locationMode = TagGroupLocation;
        locationTargetTagIndex = -1;
    } else {
        locationMode = TagLocation;
        locationTargetTagIndex = 0;
        for(size_t i=0; i < tagSelection.size(); ++i){
            if(tagSelection[i]){
                locationTargetTagIndex = i;
                break;
            }
        }
    }        
    targetLocation->notifyAttributeChange();
    sigTargetLocationChanged();
}


SignalProxy<void()> PositionTagGroupItem::sigTagSelectionChanged()
{
    return impl->sigTagSelectionChanged;
}


void PositionTagGroupItem::Impl::removeSelectedTags()
{
    auto selection = tagSelection;
    size_t n = selection.size();
    int numRemoved = 0;
    for(int i=0; i < n; ++i){
        if(selection[i]){
            tagGroup->removeAt(i - numRemoved++);
        }
    }
}


SgNode* PositionTagGroupItem::getScene()
{
    if(!impl->sceneTagGroup){
        impl->sceneTagGroup = new SceneTagGroup(impl);
    }
    return impl->sceneTagGroup;
}


LocationProxyPtr PositionTagGroupItem::getLocationProxy()
{
    return impl->targetLocation;
}


bool PositionTagGroupItem::onNewTreePositionCheck(bool isManualOperation, std::function<void()>& out_callbackWhenAdded)
{
    bool accepted = true;

    LocationProxyPtr newParentLocation;
    if(auto parentLocatableItem = findOwnerItem<LocatableItem>()){
        newParentLocation = parentLocatableItem->getLocationProxy();
    }
    bool isParentLocationChanged = (newParentLocation != impl->groupParentLocation);
    bool doCoordinateConversion = false;
    bool doClearOriginOffset = false;
    
    if(isManualOperation && isParentLocationChanged){
        static ConversionDialog* dialog = nullptr;
        if(!dialog){
            dialog = new ConversionDialog;
        }
        dialog->setTargets(this, newParentLocation);
        if(dialog->exec() == QDialog::Accepted){
            doCoordinateConversion = dialog->globalCoordRadio.isChecked();
            doClearOriginOffset = dialog->clearOriginOffsetCheck.isChecked();
        } else {
            accepted = false;
        }
    }

    if(accepted && isParentLocationChanged){
        out_callbackWhenAdded =
            [this, newParentLocation, doCoordinateConversion, doClearOriginOffset](){
                impl->setParentItemLocationProxy(
                    newParentLocation, doCoordinateConversion, doClearOriginOffset);
        };
    }
    
    return accepted;
}


void PositionTagGroupItem::Impl::setParentItemLocationProxy
(LocationProxyPtr newParentLocation, bool doCoordinateConversion, bool doClearOriginOffset)
{
    groupParentLocationConnection.disconnect();

    if(doCoordinateConversion){
        convertLocalCoordinates(groupParentLocation, newParentLocation, doClearOriginOffset);
    }
        
    groupParentLocation = newParentLocation;

    if(groupParentLocation){
        groupParentLocationConnection =
            groupParentLocation->sigLocationChanged().connect(
                [&](){ onParentItemLocationChanged(); });
    }

    onParentItemLocationChanged();

    // Notify the change of the parent location proxy
    targetLocation->notifyAttributeChange();
}


void PositionTagGroupItem::Impl::convertLocalCoordinates
(LocationProxy* currentParentLocation, LocationProxy* newParentLocation, bool doClearOriginOffset)
{
    Isometry3 T0 = self->originPosition();

    if(doClearOriginOffset){
        self->setOriginOffset(Isometry3::Identity(), false);
        self->notifyOriginOffsetUpdate();
    }

    Isometry3 T1 = T_offset;
    if(newParentLocation){
        T1 = newParentLocation->getLocation() * T1;
    }

    Isometry3 Tc = T1.inverse(Eigen::Isometry) * T0;
    int n = tagGroup->numTags();
    for(int i=0; i < n; ++i){
        auto tag = tagGroup->tagAt(i);
        if(tag->hasAttitude()){
            tag->setPosition(Tc * tag->position());
        } else {
            tag->setTranslation(Tc * tag->translation());
        }
        tagGroup->notifyTagPositionUpdate(i);
    }

    self->notifyUpdate();
}


void PositionTagGroupItem::Impl::onParentItemLocationChanged()
{
    if(groupParentLocation){
        T_parent = groupParentLocation->getLocation();
    } else {
        T_parent.setIdentity();
    }
    if(sceneTagGroup){
        sceneTagGroup->setParentPosition(T_parent);
    }
    sigTargetLocationChanged();
}


double PositionTagGroupItem::tagMarkerSize() const
{
    return impl->fixedSizeTagMarker->pixelSizeRatio();
}


void PositionTagGroupItem::setTagMarkerSize(double s)
{
    auto& marker = impl->fixedSizeTagMarker;
    if(s != marker->pixelSizeRatio()){
        marker->setPixelSizeRatio(s);
        impl->fixedSizeSelectedTagMarker->setPixelSizeRatio(s * HighlightSizeRatio);
        impl->fixedSizeHighlightedTagMarker->setPixelSizeRatio(s * HighlightSizeRatio);
        if(impl->sceneTagGroup){
            impl->sceneTagGroup->notifyUpdate();
        }
    }
}


bool PositionTagGroupItem::originMarkerVisibility() const
{
    return impl->originMarkerVisibility;
}
    

void PositionTagGroupItem::setOriginMarkerVisibility(bool on)
{
    if(on != impl->originMarkerVisibility){
        impl->originMarkerVisibility = on;
        if(impl->sceneTagGroup){
            impl->sceneTagGroup->setOriginMarkerVisibility(on);
        }
    }
}


bool PositionTagGroupItem::edgeVisibility() const
{
    return impl->edgeVisibility;
}


void PositionTagGroupItem::setEdgeVisiblility(bool on)
{
    if(on != impl->edgeVisibility){
        impl->edgeVisibility = on;
        if(impl->sceneTagGroup){
            impl->sceneTagGroup->setEdgeVisiblility(on);
        }
    }
}


float PositionTagGroupItem::transparency() const
{
    return impl->material->transparency();
}


void PositionTagGroupItem::setTransparency(float t)
{
    impl->material->setTransparency(t);
    impl->material->notifyUpdate();
}


void PositionTagGroupItem::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("Number of tags"), impl->tagGroup->numTags());
    putProperty(_("Offset translation"), str(Vector3(impl->T_offset.translation())));
    Vector3 rpy(degree(rpyFromRot(impl->T_offset.linear())));
    putProperty(_("Offset rotation (RPY)"), str(rpy));
    putProperty(_("Origin marker"), impl->originMarkerVisibility,
                [&](bool on){ setOriginMarkerVisibility(on); return true; });
    putProperty(_("Tag marker size"), tagMarkerSize(),
                [&](double s){ setTagMarkerSize(s); return true; });
    putProperty(_("Show edges"), impl->edgeVisibility,
                [&](bool on){ setEdgeVisiblility(on); return true; });
    putProperty(_("Transparency"), transparency(),
                [&](float t){ setTransparency(t); return true; });
}


bool PositionTagGroupItem::store(Archive& archive)
{
    impl->tagGroup->write(&archive);

    archive.setFloatingNumberFormat("%.9g");
    cnoid::write(archive, "offset_translation", impl->T_offset.translation());
    cnoid::write(archive, "offset_rpy", degree(rpyFromRot(impl->T_offset.linear())));

    archive.write("origin_marker", impl->originMarkerVisibility);
    archive.write("tag_marker_size", tagMarkerSize());
    archive.write("show_edges", impl->edgeVisibility);
    
    return true;
}


bool PositionTagGroupItem::restore(const Archive& archive)
{
    Vector3 v;
    if(cnoid::read(archive, "offset_translation", v)){
        impl->T_offset.translation() = v;
    }
    if(cnoid::read(archive, "offset_rpy", v)){
        impl->T_offset.linear() = rotFromRpy(radian(v));
    }
    if(archive.get("origin_marker", false)){
        setOriginMarkerVisibility(true);
    }
    double s;
    if(archive.read("tag_marker_size", s)){
        setTagMarkerSize(s);
    }
    bool on;
    if(archive.read("show_edges", on)){
        setEdgeVisiblility(on);
    }
    
    return impl->tagGroup->read(&archive);
}


namespace {

SceneTagGroup::SceneTagGroup(PositionTagGroupItem::Impl* impl)
    : impl(impl)
{
    auto tagGroup = impl->tagGroup;

    setPosition(impl->T_parent);

    offsetTransform = new SgPosTransform;
    offsetTransform->setPosition(impl->T_offset);
    addChild(offsetTransform);

    tagMarkerGroup = new SgGroup;
    offsetTransform->addChild(tagMarkerGroup);

    highlightedTagIndex = -1;

    int n = tagGroup->numTags();
    for(int i=0; i < n; ++i){
        addTagNode(i, false, false);
    }

    edgeLineSet = new SgLineSet;
    edgeLineSet->setLineWidth(2.0f);
    auto material = edgeLineSet->getOrCreateMaterial();
    material->setDiffuseColor(Vector3f(0.9f, 0.9f, 0.9f));
    if(impl->edgeVisibility){
        setEdgeVisiblility(true);
    }

    if(impl->originMarkerVisibility){
        setOriginMarkerVisibility(true);
    }

    draggingTagIndex = -1;    
    
    tagGroupConnections.add(
        tagGroup->sigTagAdded().connect(
            [&](int index){ addTagNode(index, true, true); }));
    tagGroupConnections.add(
        tagGroup->sigTagRemoved().connect(
            [&](int index, PositionTag*){ removeTagNode(index); }));
    tagGroupConnections.add(
        tagGroup->sigTagPositionChanged().connect(
            [&](int index){ updateTagNodePosition(index); }));
}


void SceneTagGroup::finalize()
{
    tagGroupConnections.disconnect();
    impl = nullptr;
}


void SceneTagGroup::addTagNode(int index, bool doUpdateEdges, bool doNotify)
{
    auto tag = impl->tagGroup->tagAt(index);
    auto tagNode = new SceneTag;
    if(impl->checkTagSelected(index)){
        tagNode->addChild(getOrCreateSelectedTagMarker());
        tagNode->displayType = Selected;
    } else {
        tagNode->addChild(getOrCreateTagMarker());
    }        
    tagNode->setPosition(tag->position());
    tagMarkerGroup->insertChild(index, tagNode, doNotify ? &update : nullptr);

    if(doUpdateEdges){
        updateEdges(doNotify);
    }
}
    

SgNode* SceneTagGroup::getOrCreateTagMarker()
{
    if(!tagMarker){
        auto lines = new SgLineSet;

        auto& vertices = *lines->getOrCreateVertices(4);
        constexpr float r = 3.0f;
        constexpr float offset = -1.0f; // 0.0f or -1.0f
        vertices[0] << 0.0f,     0.0f,     offset;        // Origin
        vertices[1] << 0.0f,     0.0f,     1.0f + offset; // Z direction
        vertices[2] << 1.0f / r, 0.0f,     offset;        // X direction
        vertices[3] << 0.0f,     1.0f / r, offset;        // Y direction
        
        auto& colors = *lines->getOrCreateColors(3);
        colors[0] << 0.9f, 0.0f, 0.0f; // Red
        colors[1] << 0.0f, 0.8f, 0.0f; // Green
        colors[2] << 0.0f, 0.0f, 0.8f; // Blue
        
        lines->setNumLines(5);
        lines->setLineWidth(1.0f);
        lines->resizeColorIndicesForNumLines(5);
        // Origin -> Z, Blue
        lines->setLine(0, 0, 1);    
        lines->setLineColor(0, 2);
        // Origin -> X, Red
        lines->setLine(1, 0, 2);
        lines->setLineColor(1, 0);
        // Origin -> Y, Green
        lines->setLine(2, 0, 3);
        lines->setLineColor(2, 1);
        // Z -> X, Red
        lines->setLine(3, 1, 2);
        lines->setLineColor(3, 0);
        // Z -> Y, Green
        lines->setLine(4, 1, 3);
        lines->setLineColor(4, 1);

        lines->setMaterial(impl->material);

        impl->fixedSizeTagMarker->addChild(lines);
        markerLineSet = lines;
        tagMarker = impl->fixedSizeTagMarker;
    }

    return tagMarker;
}


SgNode* SceneTagGroup::getOrCreateSelectedTagMarker()
{
    if(!selectedTagMarker){
        if(!tagMarker){
            getOrCreateTagMarker();
        }
        auto lines = new SgLineSet(*markerLineSet);
        lines->setLineWidth(3.0f);
        auto colors = new SgColorArray;
        colors->reserve(3);
        colors->emplace_back(1.0f, 0.3f, 0.3f); // Red
        colors->emplace_back(0.2f, 1.0f, 0.2f); // Green
        colors->emplace_back(0.3f, 0.3f, 1.0f); // Blue
        lines->setColors(colors);
        
        impl->fixedSizeSelectedTagMarker->addChild(lines);
        selectedTagMarker = impl->fixedSizeSelectedTagMarker;
    }

    return selectedTagMarker;
}


SgNode* SceneTagGroup::getOrCreateHighlightedTagMarker()
{
    if(!highlightedTagMarker){
        if(!tagMarker){
            getOrCreateTagMarker();
        }
        auto lines = new SgLineSet(*markerLineSet);
        lines->setLineWidth(3.0f);
        auto colors = new SgColorArray;
        colors->reserve(3);
        colors->emplace_back(1.0f, 1.0f, 0.0f); // Yellow
        colors->emplace_back(1.0f, 1.0f, 0.0f); // Yellow
        colors->emplace_back(1.0f, 1.0f, 0.0f); // Yellow
        lines->setColors(colors);

        impl->fixedSizeHighlightedTagMarker->addChild(lines);
        highlightedTagMarker = impl->fixedSizeHighlightedTagMarker;
    }

    return highlightedTagMarker;
}


void SceneTagGroup::removeTagNode(int index)
{
    tagMarkerGroup->removeChildAt(index, update);
    updateEdges(true);
}


void SceneTagGroup::updateTagNodePosition(int index)
{
    auto tag = impl->tagGroup->tagAt(index);
    auto node = static_cast<SgPosTransform*>(tagMarkerGroup->child(index));
    if(tag->hasAttitude()){
        node->setPosition(tag->position());
    } else {
        node->setTranslation(tag->translation());
    }
    update.setAction(SgUpdate::MODIFIED);
    node->notifyUpdate(update);

    if(index == draggingTagIndex){
        positionDragger->setPosition(tag->position());
        positionDragger->notifyUpdate(update);
    }

    updateEdges(true);
}


void SceneTagGroup::updateTagDisplayTypes()
{
    bool updated = false;
    int n = tagMarkerGroup->numChildren();
    for(int i=0; i < n; ++i){
        if(updateTagDisplayType(i, false)){
            updated = true;
        }
    }
    if(updated){
        tagMarkerGroup->notifyUpdate(update.withAction(SgUpdate::MODIFIED));
    }
}


bool SceneTagGroup::updateTagDisplayType(int index, bool doNotify)
{
    bool updated = false;
    auto tagNode = static_cast<SceneTag*>(tagMarkerGroup->child(index));
    TagDisplayType displayType;
    if(index == highlightedTagIndex){
        displayType = Highlighted;
    } else if(impl->checkTagSelected(index)){
        displayType = Selected;
    } else {
        displayType = Normal;
    }
    if(displayType != tagNode->displayType){
        SgUpdate* pUpdate = doNotify ? &update : nullptr;
        tagNode->clearChildren(pUpdate);
        switch(displayType){
        case Normal:
        default:
            tagNode->addChild(getOrCreateTagMarker(), pUpdate);
            break;
        case Selected:
            tagNode->addChild(getOrCreateSelectedTagMarker(), pUpdate);
            break;
        case Highlighted:
            tagNode->addChild(getOrCreateHighlightedTagMarker(), pUpdate);
            break;
        }
        tagNode->displayType = displayType;
    }
    return updated;
}


void SceneTagGroup::updateEdges(bool doNotify)
{
    if(!impl->edgeVisibility){
        return;
    }
    
    auto& vertices = *edgeLineSet->getOrCreateVertices();
    vertices.clear();
    for(auto& tag : *impl->tagGroup){
        vertices.push_back(tag->translation().cast<SgVertexArray::Scalar>());
    }
    int numLines = vertices.size() - 1;
    if(numLines < 0){
        numLines = 0;
    }
    int numLines0 = edgeLineSet->numLines();
    edgeLineSet->setNumLines(numLines);
    if(numLines > numLines0){
        for(int i = numLines0; i < numLines; ++i){
            edgeLineSet->setLine(i, i, i + 1);
        }
    }
    if(doNotify){
        vertices.notifyUpdate(update.withAction(SgUpdate::MODIFIED));
    }
}


void SceneTagGroup::setOriginOffset(const Isometry3& T)
{
    offsetTransform->setPosition(T);
    offsetTransform->notifyUpdate(update.withAction(SgUpdate::MODIFIED));
}


void SceneTagGroup::setParentPosition(const Isometry3& T)
{
    setPosition(T);
    notifyUpdate(update.withAction(SgUpdate::MODIFIED));
}


void SceneTagGroup::setOriginMarkerVisibility(bool on)
{
    if(on){
        if(!originMarker){
            originMarker = new CoordinateFrameMarker;
        }
        offsetTransform->addChild(originMarker, update);
    } else {
        if(originMarker){
            offsetTransform->removeChild(originMarker, update);
        }
    }
}


void SceneTagGroup::setEdgeVisiblility(bool on)
{
    if(on){
        updateEdges(false);
        offsetTransform->addChild(edgeLineSet, update);
    } else {
        offsetTransform->removeChild(edgeLineSet, update);
    }
}


void SceneTagGroup::setHighlightedTagIndex(int index)
{
    if(index != highlightedTagIndex){
        int prevHighlightedTagIndex = highlightedTagIndex;
        highlightedTagIndex = index;
        if(prevHighlightedTagIndex >= 0){
            updateTagDisplayType(prevHighlightedTagIndex, true);
        }
        if(highlightedTagIndex >= 0){
            updateTagDisplayType(highlightedTagIndex, true);
        }
    }
}


void SceneTagGroup::attachPositionDragger(int tagIndex)
{
    PositionTag* tag = nullptr;
    if(tagIndex >= 0){
        tag = impl->tagGroup->tagAt(tagIndex);
    }

    if(!tag){
        draggingTagIndex = -1;
        offsetTransform->removeChild(positionDragger, update);

    } else {
        if(!positionDragger){
            positionDragger = new PositionDragger(PositionDragger::AllAxes, PositionDragger::PositiveOnlyHandle);
            positionDragger->setOverlayMode(true);
            positionDragger->setPixelSize(96, 3);
            positionDragger->setDisplayMode(PositionDragger::DisplayInEditMode);
            positionDragger->sigDragStarted().connect([&](){ onDraggerDragStarted(); });
            positionDragger->sigPositionDragged().connect([&](){ onDraggerDragged(); });
            positionDragger->sigDragFinished().connect([&](){ onDraggerDragFinished(); });
        }

        positionDragger->setPosition(tag->position());
        offsetTransform->addChildOnce(positionDragger, update);
        draggingTagIndex = tagIndex;
    }
}


void SceneTagGroup::onDraggerDragStarted()
{
    initialDraggerPosition = positionDragger->position();

    auto& selection = impl->tagSelection;
    initialTagDragPositions.clear();
    initialTagDragPositions.reserve(impl->numSelectedTags);
    int arrayIndex = 0;
    for(size_t i=0; i < selection.size(); ++i){
        if(selection[i]){
            initialTagDragPositions.emplace_back(i, impl->tagGroup->tagAt(i)->position());
        }
    }
}
    

void SceneTagGroup::onDraggerDragged()
{
    Isometry3 T = positionDragger->draggingPosition();
    Isometry3 T_base = T * initialDraggerPosition.inverse(Eigen::Isometry);

    for(auto& tagpos0 : initialTagDragPositions){
        auto tag = impl->tagGroup->tagAt(tagpos0.index);
        if(tagpos0.index == draggingTagIndex){
            tag->setPosition(T);
            positionDragger->setPosition(T);
            positionDragger->notifyUpdate(update.withAction(SgUpdate::MODIFIED));
        } else {
            tag->setPosition(T_base * tagpos0.T);
        }
        impl->tagGroup->notifyTagPositionChange(tagpos0.index);
    }
}


void SceneTagGroup::onDraggerDragFinished()
{
    for(auto& tagpos0 : initialTagDragPositions){
        impl->tagGroup->notifyTagPositionUpdate(tagpos0.index, false);
    }
}


int SceneTagGroup::findPointingTagIndex(SceneWidgetEvent* event)
{
    auto path = event->nodePath();
    size_t tagNodeIndex = 0;
    for(size_t i=0; i < path.size(); ++i){
        auto node = path[i];
        if(node == tagMarkerGroup){
            tagNodeIndex = i + 1;
            break;
        }
    }
    int tagIndex = -1;
    if(tagNodeIndex > 0 && tagNodeIndex < path.size()){
        auto tagNode = dynamic_cast<SceneTag*>(path[tagNodeIndex]);
        if(tagNode){
            tagIndex = tagMarkerGroup->findChildIndex(tagNode);
        }
    }
    return tagIndex;
}


bool SceneTagGroup::onPointerMoveEvent(SceneWidgetEvent* event)
{
    int tagIndex = findPointingTagIndex(event);
    setHighlightedTagIndex(tagIndex);
    return (tagIndex >= 0);
}


void SceneTagGroup::onPointerLeaveEvent(SceneWidgetEvent* event)
{
    setHighlightedTagIndex(-1);
}


bool SceneTagGroup::onButtonPressEvent(SceneWidgetEvent* event)
{
    bool processed = false;

    int tagIndex = findPointingTagIndex(event);
    
    if(event->button() == Qt::LeftButton){
        if(tagIndex >= 0){
            bool selected = impl->checkTagSelected(tagIndex);
            if(!(event->modifiers() & Qt::ControlModifier)){
                if(impl->numSelectedTags >= 2 || !selected){
                    impl->clearTagSelection(true);
                    selected = false;
                }
            }
            impl->setTagSelected(tagIndex, !selected, true);
            processed = true;
        }
        
    } else if(event->button() == Qt::RightButton){
        if(tagIndex >= 0 && !impl->checkTagSelected(tagIndex)){
            impl->setTagSelected(tagIndex, true, true);
        }
        event->sceneWidget()->showContextMenuAtPointerPosition();
    }

    attachPositionDragger(-1);
    
    return processed;
}


void SceneTagGroup::onFocusChanged(SceneWidgetEvent* event, bool on)
{
    if(!on){
        attachPositionDragger(-1);
    }
}


bool SceneTagGroup::onContextMenuRequest(SceneWidgetEvent* event, MenuManager* menu)
{
    int tagIndex = findPointingTagIndex(event);
    
    auto moveAction = menu->addItem(_("Move"));
    if(tagIndex < 0){
        moveAction->setEnabled(false);
    }
    moveAction->sigTriggered().connect([this, tagIndex](){ attachPositionDragger(tagIndex); });
                
    menu->addItem(_("Remove"))->sigTriggered().connect([&](){ impl->removeSelectedTags(); });
    menu->addSeparator();
    menu->addItem(_("Select all"))->sigTriggered().connect([&](){ impl->selectAllTags(); });
    menu->addItem(_("Clear selection"))->sigTriggered().connect([&](){ impl->clearTagSelection(true); });

    attachPositionDragger(-1);
    
    return true;
}


TargetLocationProxy::TargetLocationProxy(PositionTagGroupItem::Impl* impl)
    : LocationProxy(ParentRelativeLocation),
      impl(impl)
{

}


const PositionTag* TargetLocationProxy::getTargetTag() const
{
    return impl->tagGroup->tagAt(impl->locationTargetTagIndex);
}


std::string TargetLocationProxy::getName() const
{
    if(impl->locationMode == TagGroupLocation){
        return format(_("{0}: Origin"), impl->self->displayName());
    } else {
        auto tag = getTargetTag();
        return format(_("{0}: Tag {1}"), impl->self->displayName(), impl->locationTargetTagIndex);
    }
}


Item* TargetLocationProxy::getCorrespondingItem()
{
    if(impl->locationMode == TagGroupLocation){
        return impl->self;
    } else {
        return nullptr;
    }
}


Isometry3 TargetLocationProxy::getLocation() const
{
    if(impl->locationMode == TagGroupLocation){
        return impl->T_offset;
    } else {
        return getTargetTag()->position();
    }
}


bool TargetLocationProxy::setLocation(const Isometry3& T)
{
    auto tagGroup = impl->tagGroup;
    if(impl->locationMode == TagGroupLocation){
        impl->self->setOriginOffset(T, true);
    } else {
        auto primaryTag = tagGroup->tagAt(impl->locationTargetTagIndex);
        Isometry3 T_base = T * primaryTag->position().inverse(Eigen::Isometry);
        primaryTag->setPosition(T);
        for(size_t i=0; i < impl->tagSelection.size(); ++i){
            if(impl->tagSelection[i]){
                if(i != impl->locationTargetTagIndex){
                    auto tag = tagGroup->tagAt(i);
                    tag->setPosition(T_base * tag->position());
                }
                tagGroup->notifyTagPositionChange(i);
            }
        }
    }
    return true;
}


void TargetLocationProxy::finishLocationEditing()
{
    auto tagGroup = impl->tagGroup;
    if(impl->locationMode == TagGroupLocation){
        impl->self->notifyOriginOffsetUpdate(false);
    } else {
        auto primaryTag = tagGroup->tagAt(impl->locationTargetTagIndex);
        for(size_t i=0; i < impl->tagSelection.size(); ++i){
            if(impl->tagSelection[i]){
                tagGroup->notifyTagPositionUpdate(i, false);
            }
        }
    }
}


SignalProxy<void()> TargetLocationProxy::sigLocationChanged()
{
    return impl->sigTargetLocationChanged;
}


LocationProxyPtr TargetLocationProxy::getParentLocationProxy() const
{
    if(impl->locationMode == TagGroupLocation){
        return impl->groupParentLocation;
    } else {
        return impl->tagParentLocation;
    }
}


TagParentLocationProxy::TagParentLocationProxy(PositionTagGroupItem::Impl* impl)
    : LocationProxy(ParentRelativeLocation),
      impl(impl)
{

}


std::string TagParentLocationProxy::getName() const
{
    return format(_("{0}: Origin"), impl->self->name());
}


Isometry3 TagParentLocationProxy::getLocation() const
{
    return impl->T_offset;
}


SignalProxy<void()> TagParentLocationProxy::sigLocationChanged()
{
    return impl->sigTagParentLocationChanged;
}


LocationProxyPtr TagParentLocationProxy::getParentLocationProxy() const
{
    if(impl->groupParentLocation){
        return impl->groupParentLocation;
    }
    return nullptr;
}


OffsetEditRecord::OffsetEditRecord
(PositionTagGroupItem* tagGroupItem, const Isometry3& T_new, const Isometry3& T_old)
    : tagGroupItem(tagGroupItem),
      T_new(T_new),
      T_old(T_old)
{

}


OffsetEditRecord::OffsetEditRecord(const OffsetEditRecord& org)
    : EditRecord(org),
      tagGroupItem(org.tagGroupItem),
      T_new(org.T_new),
      T_old(org.T_old)
{

}


EditRecord* OffsetEditRecord::clone() const
{
    return new OffsetEditRecord(*this);
}


std::string OffsetEditRecord::label() const
{
    return _("Change the origin offset of a position tag group");
}


bool OffsetEditRecord::undo()
{
    tagGroupItem->setOriginOffset(T_old, true);
    return true;
}


bool OffsetEditRecord::redo()
{
    tagGroupItem->setOriginOffset(T_new, true);
    return true;
}


TagEditRecord::TagEditRecord
(PositionTagGroupItem::Impl* tagGroupItemImpl, TagAction action, int tagIndex,
 PositionTag* newTag, PositionTag* oldTag)
    : tagGroupItem(tagGroupItemImpl->self),
      tagGroupItemImpl(tagGroupItemImpl),
      action(action),
      tagIndex(tagIndex)
{
    setTags(newTag, oldTag);
}


TagEditRecord::TagEditRecord(const TagEditRecord& org)
    : EditRecord(org),
      tagGroupItem(org.tagGroupItem),
      tagGroupItemImpl(org.tagGroupItemImpl),
      action(org.action),
      tagIndex(org.tagIndex)
{
    setTags(org.newTag, org.oldTag);
}


void TagEditRecord::setTags(PositionTag* newTag0, PositionTag* oldTag0)
{
    if(newTag0){
        newTag = new PositionTag(*newTag0);
    }
    if(oldTag0){
        oldTag = new PositionTag(*oldTag0);
    }
}    

    
EditRecord* TagEditRecord::clone() const
{
    return new TagEditRecord(*this);
}


std::string TagEditRecord::label() const
{
    int actualAction = action;
    if(isReverse()){
        if(action == AddAction){
            actualAction = RemoveAction;
        } else if(action == RemoveAction){
            actualAction = AddAction;
        }
    }
    switch(action){
    case AddAction:
        return _("Add a position tag");
    case UpdateAction:
        return _("Update a position tag");
    case RemoveAction:
        return _("Remove a position tag");
    default:
        break;
    }
    return string();
}


bool TagEditRecord::undo()
{
    bool done = false;
    auto tagGroup = tagGroupItemImpl->tagGroup;
    tagGroupItemImpl->isDoingUndoOrRedo = true;

    switch(action){
    case AddAction:
        done = tagGroup->removeAt(tagIndex);
        break;
    case UpdateAction:
        (*tagGroup->tagAt(tagIndex)) = *oldTag;
        tagGroup->notifyTagPositionUpdate(tagIndex);
        done = true;
        break;
    case RemoveAction:
        tagGroup->insert(tagIndex, new PositionTag(*oldTag));
        done = true;
        break;
    default:
        break;
    }

    tagGroupItemImpl->isDoingUndoOrRedo = false;

    return done;
}


bool TagEditRecord::redo()
{
    bool done = false;
    auto tagGroup = tagGroupItemImpl->tagGroup;
    tagGroupItemImpl->isDoingUndoOrRedo = true;

    switch(action){
    case AddAction:
        tagGroup->insert(tagIndex, new PositionTag(*newTag));
        done = true;
        break;
    case UpdateAction:
        (*tagGroup->tagAt(tagIndex)) = *newTag;
        tagGroup->notifyTagPositionUpdate(tagIndex);
        done = true;
        break;
    case RemoveAction:
        done = tagGroup->removeAt(tagIndex);
        break;
    default:
        break;
    }

    tagGroupItemImpl->isDoingUndoOrRedo = false;

    return done;
}


ConversionDialog::ConversionDialog()
{
    setWindowTitle(_("Tag Group Conversion"));
    auto vbox = new QVBoxLayout;

    descriptionLabel.setWordWrap(true);
    vbox->addWidget(&descriptionLabel);
    
    auto descriptionLabel2 = new QLabel;
    descriptionLabel2->setWordWrap(true);
    descriptionLabel2->setText(
        _("Which type of positions do you want to keep in the coordinate system change?"));
    vbox->addWidget(descriptionLabel2);

    auto hbox = new QHBoxLayout;
    globalCoordRadio.setText(_("Global positions"));
    globalCoordRadio.setChecked(true);
    hbox->addWidget(&globalCoordRadio);
    clearOriginOffsetCheck.setText(_("Clear the origin offset"));
    clearOriginOffsetCheck.setChecked(true);
    hbox->addWidget(&clearOriginOffsetCheck);
    hbox->addStretch();
    vbox->addLayout(hbox);
    
    localCoordRadio.setText(_("Local positions"));
    vbox->addWidget(&localCoordRadio);
    vbox->addStretch();
    
    auto buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
    buttonBox->button(QDialogButtonBox::Ok)->setAutoDefault(true);
    connect(buttonBox, &QDialogButtonBox::accepted, this, &QDialog::accept);
    connect(buttonBox, &QDialogButtonBox::rejected, this, &QDialog::reject);
    vbox->addWidget(buttonBox);

    setLayout(vbox);
}


void ConversionDialog::setTargets(PositionTagGroupItem* tagGroupItem, LocationProxyPtr newParentLocation)
{
    if(newParentLocation){
        descriptionLabel.setText(
            QString(_("The parent coordinate system of \"%1\" is changed to the coordinate system of \"%2\"."))
            .arg(tagGroupItem->displayName().c_str()).arg(newParentLocation->getName().c_str()));
    } else {
        descriptionLabel.setText(
            QString(_("The parent coordinate system of \"%1\" is changed to the global coordinate system."))
            .arg(tagGroupItem->displayName().c_str()));
    }
}

}
