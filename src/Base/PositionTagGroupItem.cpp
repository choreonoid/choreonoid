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
#include <QBoxLayout>
#include <QLabel>
#include <QDialogButtonBox>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

class ScenePositionTagGroup : public SgPosTransform, public SceneWidgetEditable
{
public:
    PositionTagGroupItem::Impl* itemImpl;
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


class PositionTagGroupLocation : public LocationProxy
{
public:
    PositionTagGroupItem::Impl* itemImpl;

    PositionTagGroupLocation(PositionTagGroupItem::Impl* itemImpl);
    virtual int getType() const override;
    virtual Item* getCorrespondingItem() override;
    virtual Position getLocation() const override;
    virtual void setLocation(const Position& T) override;
    virtual SignalProxy<void()> sigLocationChanged() override;
    virtual LocationProxyPtr getParentLocationProxy() override;
};

typedef ref_ptr<PositionTagGroupLocation> PositionTagGroupLocationPtr;


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
    PositionTagGroupLocationPtr location;
    LocationProxyPtr parentLocation;
    ScopedConnection parentLocationConnection;
    Position T_parent;
    ScenePositionTagGroupPtr scene;
    SgNodePtr tagMarker;
    SgFixedPixelSizeGroupPtr tagMarkerSizeGroup;
    bool originMarkerVisibility;
    bool edgeVisibility;
    Signal<void()> sigLocationChanged;
    
    Impl(PositionTagGroupItem* self, const Impl* org);
    void setParentLocationProxy(
        LocationProxyPtr newParentLocation, bool doCoordinateConversion, bool doClearOriginOffset);
    void convertLocalCoordinates(
        LocationProxy* currentParentLocation, LocationProxy* newParentLocation, bool doClearOriginOffset);
    void onParentLocationChanged();
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
            [&](int){ notifyUpdateLater(); }));
    tagGroupConnections.add(
        tags->sigOriginOffsetChanged().connect(
            [&](const Position&){ onOriginOffsetChanged(); }));

    tagMarkerSizeGroup = new SgFixedPixelSizeGroup;
    originMarkerVisibility = false;
    edgeVisibility = false;

    location = new PositionTagGroupLocation(this);

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


SgNode* PositionTagGroupItem::getScene()
{
    if(!impl->scene){
        impl->scene = new ScenePositionTagGroup(impl);
    }
    return impl->scene;
}


LocationProxyPtr PositionTagGroupItem::getLocationProxy()
{
    return impl->location;
}


bool PositionTagGroupItem::onNewPositionCheck(bool isManualOperation, std::function<void()>& out_callbackWhenAdded)
{
    bool accepted = true;

    LocationProxyPtr newParentLocation;
    if(auto parentLocatableItem = findOwnerItem<LocatableItem>()){
        newParentLocation = parentLocatableItem->getLocationProxy();
    }
    bool isParentLocationChanged = (newParentLocation != impl->parentLocation);
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
                impl->setParentLocationProxy(
                    newParentLocation, doCoordinateConversion, doClearOriginOffset);
        };
    }
    
    return accepted;
}


void PositionTagGroupItem::Impl::setParentLocationProxy
(LocationProxyPtr newParentLocation, bool doCoordinateConversion, bool doClearOriginOffset)
{
    parentLocationConnection.disconnect();

    if(doCoordinateConversion){
        convertLocalCoordinates(parentLocation, newParentLocation, doClearOriginOffset);
    }
        
    parentLocation = newParentLocation;

    if(parentLocation){
        parentLocationConnection =
            parentLocation->sigLocationChanged().connect(
                [&](){ onParentLocationChanged(); });
    }

    onParentLocationChanged();

    // Notify the change of the parent location proxy
    location->notifyAttributeChange();
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


void PositionTagGroupItem::Impl::onParentLocationChanged()
{
    if(parentLocation){
        T_parent = parentLocation->getLocation();
    } else {
        T_parent.setIdentity();
    }
    if(scene){
        scene->setParentPosition(T_parent);
    }
    sigLocationChanged();
}


void PositionTagGroupItem::Impl::onOriginOffsetChanged()
{
    sigLocationChanged();
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


ScenePositionTagGroup::ScenePositionTagGroup(PositionTagGroupItem::Impl* itemImpl)
    : itemImpl(itemImpl)
{
    auto tags = itemImpl->tags;

    setPosition(itemImpl->T_parent);

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
    if(itemImpl->edgeVisibility){
        setEdgeVisiblility(true);
    }

    if(itemImpl->originMarkerVisibility){
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
    itemImpl = nullptr;
}


void ScenePositionTagGroup::addTagNode(int index, bool doUpdateEdges, bool doNotify)
{
    auto tag = itemImpl->tags->tagAt(index);
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
    auto& marker = itemImpl->tagMarker;

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

        itemImpl->tagMarkerSizeGroup->addChild(lines);
        marker = itemImpl->tagMarkerSizeGroup;
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
    auto tag = itemImpl->tags->tagAt(index);
    auto node = static_cast<SgPosTransform*>(tagMarkerGroup->child(index));
    node->setTranslation(tag->translation());
    update.resetAction(SgUpdate::MODIFIED);
    node->notifyUpdate(update);

    updateEdges(true);
}


void ScenePositionTagGroup::updateEdges(bool doNotify)
{
    if(!itemImpl->edgeVisibility){
        return;
    }
    
    auto& vertices = *edgeLineSet->getOrCreateVertices();
    vertices.clear();
    for(auto& tag : *itemImpl->tags){
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


PositionTagGroupLocation::PositionTagGroupLocation(PositionTagGroupItem::Impl* itemImpl)
    : itemImpl(itemImpl)
{

}


int PositionTagGroupLocation::getType() const
{
    return ParentRelativeLocation;
}


Item* PositionTagGroupLocation::getCorrespondingItem()
{
    return itemImpl->self;
}


Position PositionTagGroupLocation::getLocation() const
{
    return itemImpl->tags->originOffset();
}


void PositionTagGroupLocation::setLocation(const Position& T)
{
    itemImpl->tags->setOriginOffset(T, true);
}


SignalProxy<void()> PositionTagGroupLocation::sigLocationChanged()
{
    return itemImpl->sigLocationChanged;
}


LocationProxyPtr PositionTagGroupLocation::getParentLocationProxy()
{
    return itemImpl->parentLocation;
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
