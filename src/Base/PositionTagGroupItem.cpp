#include "PositionTagGroupItem.h"
#include "ItemManager.h"
#include "SceneWidgetEditable.h"
#include "CoordinateFrameMarker.h"
#include "Dialog.h"
#include "Buttons.h"
#include "CheckBox.h"
#include "LazyCaller.h"
#include "Archive.h"
#include "PutPropertyFunction.h"
#include <cnoid/PositionTagGroup>
#include <cnoid/SceneDrawables>
#include <cnoid/ConnectionSet>
#include <fmt/format.h>
#include <QBoxLayout>
#include <QLabel>
#include <QDialogButtonBox>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

namespace {

enum LocationMode { TagGroupLocation, TagLocation };

class ScenePositionTagGroup : public SgPosTransform, public SceneWidgetEditable
{
public:
    PositionTagGroupItem::Impl* impl;
    SgPosTransformPtr offsetTransform;
    SgGroupPtr tagMarkerGroup;
    CoordinateFrameMarkerPtr originMarker;
    SgLineSetPtr edgeLineSet;
    SgUpdate update;
    ScopedConnectionSet tagGroupConnections;
    
    ScenePositionTagGroup(PositionTagGroupItem::Impl* itemImpl);
    void finalize();
    void addTagNode(int index, bool doUpdateEdges, bool doNotify);
    SgNode* getOrCreateTagMarker();
    void removeTagNode(int index);
    void updateTagNodePosition(int index);
    void updateEdges(bool doNotify);
    void setOriginOffset(const Position& T);
    void setParentPosition(const Position& T);
    void setOriginMarkerVisibility(bool on);
    void setEdgeVisiblility(bool on);
};

typedef ref_ptr<ScenePositionTagGroup> ScenePositionTagGroupPtr;


class TargetLocationProxy : public LocationProxy
{
public:
    PositionTagGroupItem::Impl* impl;

    TargetLocationProxy(PositionTagGroupItem::Impl* impl);
    const PositionTag* getTargetTag() const;
    virtual std::string getName() const override;
    virtual Item* getCorrespondingItem() override;
    virtual Position getLocation() const override;
    virtual bool setLocation(const Position& T) override;
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
    virtual Position getLocation() const override;
    virtual SignalProxy<void()> sigLocationChanged() override;
    virtual LocationProxyPtr getParentLocationProxy() const override;
};

typedef ref_ptr<TagParentLocationProxy> TagParentLocationProxyPtr;


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
    PositionTagGroupPtr tags;
    ScopedConnectionSet tagGroupConnections;
    LazyCaller notifyUpdateLater;
    std::vector<bool> tagSelection;
    int numSelectedTags;
    std::vector<int> selectedTagIndices;
    bool needToUpdateSelectedTagIndices;
    Signal<void()> sigTagSelectionChanged;
    LocationMode locationMode;
    int locationTargetTagIndex;
    TargetLocationProxyPtr targetLocation;
    Signal<void()> sigTargetLocationChanged;
    LocationProxyPtr groupParentLocation;
    ScopedConnection groupParentLocationConnection;
    Position T_parent;
    TagParentLocationProxyPtr tagParentLocation;
    Signal<void()> sigTagParentLocationChanged;
    ScenePositionTagGroupPtr scene;
    SgNodePtr tagMarker;
    SgFixedPixelSizeGroupPtr tagMarkerSizeGroup;
    bool originMarkerVisibility;
    bool edgeVisibility;
    
    Impl(PositionTagGroupItem* self, const Impl* org);
    void onTagUpdated(int tagIndex);
    void clearTagSelection(bool doNotify);
    void setTagSelected(int tagIndex, bool on, bool doNotify);
    void onTagSelectionChanged();
    void setParentItemLocationProxy(
        LocationProxyPtr newParentLocation, bool doCoordinateConversion, bool doClearOriginOffset);
    void convertLocalCoordinates(
        LocationProxy* currentParentLocation, LocationProxy* newParentLocation, bool doClearOriginOffset); 
    void onParentItemLocationChanged();
    void onOriginOffsetChanged();
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
      notifyUpdateLater([=](){ self->notifyUpdate(); })
{
    if(!org){
        tags = new PositionTagGroup;
    } else {
        tags = new PositionTagGroup(*org->tags);
    }

    tagGroupConnections.add(
        tags->sigTagAdded().connect(
            [&](int){ notifyUpdateLater(); }));
    tagGroupConnections.add(
        tags->sigTagRemoved().connect(
            [&](int, PositionTag*){ notifyUpdateLater(); }));
    tagGroupConnections.add(
        tags->sigTagUpdated().connect(
            [&](int index){ onTagUpdated(index); }));
    tagGroupConnections.add(
        tags->sigOriginOffsetChanged().connect(
            [&](const Position&){ onOriginOffsetChanged(); }));

    numSelectedTags = 0;
    needToUpdateSelectedTagIndices = false;

    locationMode = TagGroupLocation;
    locationTargetTagIndex = -1;
    targetLocation = new TargetLocationProxy(this);
    tagParentLocation = new TagParentLocationProxy(this);

    tagMarkerSizeGroup = new SgFixedPixelSizeGroup;
    originMarkerVisibility = false;
    edgeVisibility = false;
    
    if(!org){
        tagMarkerSizeGroup->setPixelSizeRatio(24.0f);
        originMarkerVisibility = false;
        edgeVisibility = false;
        T_parent.setIdentity();
    } else {
        tagMarkerSizeGroup->setPixelSizeRatio(org->tagMarkerSizeGroup->pixelSizeRatio());
        originMarkerVisibility = org->originMarkerVisibility;
        edgeVisibility = org->edgeVisibility;
        T_parent = org->T_parent;
    }
}


PositionTagGroupItem::~PositionTagGroupItem()
{
    delete impl;
}


Item* PositionTagGroupItem::doDuplicate() const
{
    return new PositionTagGroupItem(*this);
}


bool PositionTagGroupItem::setName(const std::string& name)
{
    if(Item::setName(name)){
        impl->tags->setName(name);
        return true;
    }
    return false;
}


const PositionTagGroup* PositionTagGroupItem::tagGroup() const
{
    return impl->tags;
}


PositionTagGroup* PositionTagGroupItem::tagGroup()
{
    return impl->tags;
}


void PositionTagGroupItem::Impl::onTagUpdated(int tagIndex)
{
    if(locationMode == TagLocation && locationTargetTagIndex == tagIndex){
        sigTargetLocationChanged();
    }
    notifyUpdateLater();
}


void PositionTagGroupItem::clearTagSelection()
{
    impl->clearTagSelection(true);
}


void PositionTagGroupItem::Impl::clearTagSelection(bool doNotify)
{
    if(!tagSelection.empty()){
        tagSelection.clear();
        numSelectedTags = 0;
        needToUpdateSelectedTagIndices = true;
        if(doNotify){
            onTagSelectionChanged();
        }
    }
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
            onTagSelectionChanged();
        }
    }
}


bool PositionTagGroupItem::checkTagSelected(int tagIndex) const
{
    if(tagIndex < impl->tagSelection.size()){
        return impl->tagSelection[tagIndex];
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
        impl->onTagSelectionChanged();
    }
}


void PositionTagGroupItem::Impl::onTagSelectionChanged()
{
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


SgNode* PositionTagGroupItem::getScene()
{
    if(!impl->scene){
        impl->scene = new ScenePositionTagGroup(impl);
    }
    return impl->scene;
}


LocationProxyPtr PositionTagGroupItem::getLocationProxy()
{
    return impl->targetLocation;
}


bool PositionTagGroupItem::onNewPositionCheck(bool isManualOperation, std::function<void()>& out_callbackWhenAdded)
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
    Position T0 = self->globalOriginOffset();

    if(doClearOriginOffset){
        tags->setOriginOffset(Position::Identity(), true);
    }
    Position T1 = tags->originOffset();
    if(newParentLocation){
        T1 = newParentLocation->getLocation() * T1;
    }

    Position Tc = T1.inverse(Eigen::Isometry) * T0;
    int n = tags->numTags();
    for(int i=0; i < n; ++i){
        auto tag = tags->tagAt(i);
        if(tag->hasAttitude()){
            tag->setPosition(Tc * tag->position());
        } else {
            tag->setTranslation(Tc * tag->translation());
        }
        tags->notifyTagUpdate(i);
    }
}


void PositionTagGroupItem::Impl::onParentItemLocationChanged()
{
    if(groupParentLocation){
        T_parent = groupParentLocation->getLocation();
    } else {
        T_parent.setIdentity();
    }
    if(scene){
        scene->setParentPosition(T_parent);
    }
    sigTargetLocationChanged();
}


void PositionTagGroupItem::Impl::onOriginOffsetChanged()
{
    sigTargetLocationChanged();
    notifyUpdateLater();
}


const Position& PositionTagGroupItem::parentCoordinateSystem() const
{
    return impl->T_parent;
}


Position PositionTagGroupItem::globalOriginOffset() const
{
    return impl->T_parent * impl->tags->originOffset();
}


double PositionTagGroupItem::tagMarkerSize() const
{
    return impl->tagMarkerSizeGroup->pixelSizeRatio();
}


void PositionTagGroupItem::setTagMarkerSize(double s)
{
    auto& sizeGroup = impl->tagMarkerSizeGroup;
    if(s != sizeGroup->pixelSizeRatio()){
        sizeGroup->setPixelSizeRatio(s);
        sizeGroup->notifyUpdate();
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
        if(impl->scene){
            impl->scene->setOriginMarkerVisibility(on);
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
        if(impl->scene){
            impl->scene->setEdgeVisiblility(on);
        }
    }
}


void PositionTagGroupItem::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("Number of tags"), impl->tags->numTags());
    putProperty(_("Origin marker"), impl->originMarkerVisibility,
                [&](bool on){ setOriginMarkerVisibility(on); return true; });
    putProperty(_("Tag marker size"), tagMarkerSize(),
                [&](double s){ setTagMarkerSize(s); return true; });
    putProperty(_("Show edges"), impl->edgeVisibility,
                [&](bool on){ setEdgeVisiblility(on); return true; });
}


bool PositionTagGroupItem::store(Archive& archive)
{
    impl->tags->write(&archive, *archive.session());
    archive.write("origin_marker", impl->originMarkerVisibility);
    archive.write("tag_marker_size", tagMarkerSize());
    archive.write("show_edges", impl->edgeVisibility);
    return true;
}


bool PositionTagGroupItem::restore(const Archive& archive)
{
    if(impl->tags->read(&archive, *archive.session())){
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
        return true;
    }
    return false;
}


ScenePositionTagGroup::ScenePositionTagGroup(PositionTagGroupItem::Impl* impl)
    : impl(impl)
{
    auto tags = impl->tags;

    setPosition(impl->T_parent);

    offsetTransform = new SgPosTransform;
    offsetTransform->setPosition(tags->originOffset());
    addChild(offsetTransform);

    tagMarkerGroup = new SgGroup;
    offsetTransform->addChild(tagMarkerGroup);

    int n = tags->numTags();
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

    tagGroupConnections.add(
        tags->sigTagAdded().connect(
            [&](int index){ addTagNode(index, true, true); }));
    tagGroupConnections.add(
        tags->sigTagRemoved().connect(
            [&](int index, PositionTag*){ removeTagNode(index); }));
    tagGroupConnections.add(
        tags->sigTagUpdated().connect(
            [&](int index){ updateTagNodePosition(index); }));
    tagGroupConnections.add(
        tags->sigOriginOffsetChanged().connect(
            [&](const Position& T){ setOriginOffset(T); }));
}


void ScenePositionTagGroup::finalize()
{
    tagGroupConnections.disconnect();
    impl = nullptr;
}


void ScenePositionTagGroup::addTagNode(int index, bool doUpdateEdges, bool doNotify)
{
    auto tag = impl->tags->tagAt(index);
    auto node = new SgPosTransform;
    node->addChild(getOrCreateTagMarker());
    node->setPosition(tag->position());
    tagMarkerGroup->insertChild(index, node);
    if(doNotify){
        update.resetAction(SgUpdate::ADDED);
        tagMarkerGroup->notifyUpdate(update);
    }

    if(doUpdateEdges){
        updateEdges(doNotify);
    }
}
    

SgNode* ScenePositionTagGroup::getOrCreateTagMarker()
{
    auto& marker = impl->tagMarker;

    if(!marker){
        auto lines = new SgLineSet;


        auto& vertices = *lines->getOrCreateVertices(4);
        constexpr float r = 3.0f;
        constexpr float offset = -1.0f; // 0.0f or -1.0f
        vertices[0] << 0.0f,     0.0f,     offset;        // Origin
        vertices[1] << 0.0f,     0.0f,     1.0f + offset; // Z direction
        vertices[2] << 1.0f / r, 0.0f,     offset;        // X direction
        vertices[3] << 0.0f,     1.0f / r, offset;        // Y direction
        
        auto& colors = *lines->getOrCreateColors(3);
        colors[0] << 1.0f, 0.0f, 0.0f; // Red
        colors[1] << 0.0f, 0.8f, 0.0f; // Green
        colors[2] << 0.0f, 0.0f, 1.0f; // Blue
        
        lines->setNumLines(5);
        lines->setLineWidth(2.0f);
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

        impl->tagMarkerSizeGroup->addChild(lines);
        marker = impl->tagMarkerSizeGroup;
    }

    return marker;
}


void ScenePositionTagGroup::removeTagNode(int index)
{
    tagMarkerGroup->removeChildAt(index);
    update.resetAction(SgUpdate::REMOVED);
    tagMarkerGroup->notifyUpdate(update);

    updateEdges(true);
}


void ScenePositionTagGroup::updateTagNodePosition(int index)
{
    auto tag = impl->tags->tagAt(index);
    auto node = static_cast<SgPosTransform*>(tagMarkerGroup->child(index));
    node->setTranslation(tag->translation());
    update.resetAction(SgUpdate::MODIFIED);
    node->notifyUpdate(update);

    updateEdges(true);
}


void ScenePositionTagGroup::updateEdges(bool doNotify)
{
    if(!impl->edgeVisibility){
        return;
    }
    
    auto& vertices = *edgeLineSet->getOrCreateVertices();
    vertices.clear();
    for(auto& tag : *impl->tags){
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
        update.resetAction(SgUpdate::MODIFIED);
        vertices.notifyUpdate(update);
    }
}


void ScenePositionTagGroup::setOriginOffset(const Position& T)
{
    offsetTransform->setPosition(T);
    update.resetAction(SgUpdate::MODIFIED);
    offsetTransform->notifyUpdate(update);
}


void ScenePositionTagGroup::setParentPosition(const Position& T)
{
    setPosition(T);
    update.resetAction(SgUpdate::MODIFIED);
    notifyUpdate(update);
}


void ScenePositionTagGroup::setOriginMarkerVisibility(bool on)
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


void ScenePositionTagGroup::setEdgeVisiblility(bool on)
{
    if(on){
        updateEdges(false);
        offsetTransform->addChild(edgeLineSet, update);
    } else {
        offsetTransform->removeChild(edgeLineSet, update);
    }
}


TargetLocationProxy::TargetLocationProxy(PositionTagGroupItem::Impl* impl)
    : LocationProxy(ParentRelativeLocation),
      impl(impl)
{

}


const PositionTag* TargetLocationProxy::getTargetTag() const
{
    return impl->tags->tagAt(impl->locationTargetTagIndex);
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


Position TargetLocationProxy::getLocation() const
{
    if(impl->locationMode == TagGroupLocation){
        return impl->tags->originOffset();
    } else {
        return getTargetTag()->position();
    }
}


bool TargetLocationProxy::setLocation(const Position& T)
{
    auto tags = impl->tags;
    if(impl->locationMode == TagGroupLocation){
        tags->setOriginOffset(T, true);
    } else {
        int index = impl->locationTargetTagIndex;
        auto tag = tags->tagAt(index);
        tag->setPosition(T);
        // impl->tagGroupConnections.block();
        tags->notifyTagUpdate(index);
        // impl->tagGroupConnections.unblock();
    }
    return true;
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


Position TagParentLocationProxy::getLocation() const
{
    return impl->tags->originOffset();
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
