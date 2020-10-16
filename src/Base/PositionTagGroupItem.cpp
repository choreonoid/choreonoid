#include "PositionTagGroupItem.h"
#include "ItemManager.h"
#include "SceneWidgetEditable.h"
#include "CoordinateFrameMarker.h"
#include "Dialog.h"
#include "Buttons.h"
#include "CheckBox.h"
#include "LazyCaller.h"
#include "Archive.h"
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
    CoordinateFrameMarkerPtr originMarker;
    SgUpdate update;
    ScopedConnectionSet tagGroupConnections;
    
    ScenePositionTagGroup(PositionTagGroupItem::Impl* itemImpl);
    void finalize();
    void addTagNode(int index, bool doNotify);
    static SgNode* getOrCreateTagMarker();
    void removeTagNode(int index);
    void updateTagNodePosition(int index);
    void setOriginOffset(const Position& T);
    void setParentPosition(const Position& T);
    void setOriginMarkerVisible(bool on);
    bool isOriginMarkerVisible() const;
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
    Position parentPosition;
    ScenePositionTagGroupPtr scene;
    Signal<void()> sigLocationChanged;
    
    Impl(PositionTagGroupItem* self);
    void setParentLocationProxy(
        LocationProxyPtr newParentLocation, bool doCoordinateConversion, bool doClearOriginOffset);
    void convertLocalCoordinates(
        LocationProxy* currentParentLocation, LocationProxy* newParentLocation, bool doClearOriginOffset);
    void onParentLocationChanged();
    void onOffsetPositionChanged();
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
    impl = new Impl(this);
}


PositionTagGroupItem::PositionTagGroupItem(const PositionTagGroupItem& org)
    : Item(org)
{
    impl = new Impl(this);
}


PositionTagGroupItem::Impl::Impl(PositionTagGroupItem* self)
    : self(self),
      notifyUpdateLater([=](){ self->notifyUpdate(); })
{
    tags = new PositionTagGroup;

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
        tags->sigOffsetPositionChanged().connect(
            [&](const Position&){ onOffsetPositionChanged(); }));

    location = new PositionTagGroupLocation(this);
    parentPosition.setIdentity();
}


PositionTagGroupItem::~PositionTagGroupItem()
{
    delete impl;
}


Item* PositionTagGroupItem::doDuplicate() const
{
    return new PositionTagGroupItem(*this);
}


const PositionTagGroup* PositionTagGroupItem::tags() const
{
    return impl->tags;
}


PositionTagGroup* PositionTagGroupItem::tags()
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


void PositionTagGroupItem::setOriginMarkerVisible(bool on)
{
    if(!impl->scene){
        if(on){
            getScene();
        }
    }
    if(impl->scene){
        impl->scene->setOriginMarkerVisible(on);
    }
}


bool PositionTagGroupItem::isOriginMarkerVisible() const
{
    return impl->scene ? impl->scene->isOriginMarkerVisible() : false;
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
        parentPosition = parentLocation->getLocation();
    } else {
        parentPosition.setIdentity();
    }
    if(scene){
        scene->setParentPosition(parentPosition);
    }
    sigLocationChanged();
}


void PositionTagGroupItem::Impl::onOffsetPositionChanged()
{
    sigLocationChanged();
    notifyUpdateLater();
}


const Position& PositionTagGroupItem::parentPosition() const
{
    return impl->parentPosition;
}


Position PositionTagGroupItem::globalOriginOffset() const
{
    return impl->parentPosition * impl->tags->originOffset();
}


bool PositionTagGroupItem::store(Archive& archive)
{
    impl->tags->write(&archive);
    archive.write("origin_marker", isOriginMarkerVisible());
    return true;
}


bool PositionTagGroupItem::restore(const Archive& archive)
{
    if(impl->tags->read(&archive)){
        if(archive.get("origin_marker", false)){
            setOriginMarkerVisible(true);
        }
        return true;
    }
    return false;
}


ScenePositionTagGroup::ScenePositionTagGroup(PositionTagGroupItem::Impl* itemImpl)
    : itemImpl(itemImpl)
{
    auto tags = itemImpl->tags;

    setPosition(itemImpl->parentPosition);

    offsetTransform = new SgPosTransform;
    offsetTransform->setPosition(tags->originOffset());
    addChild(offsetTransform);
    
    int n = tags->numTags();
    for(int i=0; i < n; ++i){
        addTagNode(i, false);
    }

    tagGroupConnections.add(
        tags->sigTagAdded().connect(
            [&](int index){ addTagNode(index, true); }));
    tagGroupConnections.add(
        tags->sigTagRemoved().connect(
            [&](int index, PositionTag*){ removeTagNode(index); }));
    tagGroupConnections.add(
        tags->sigTagUpdated().connect(
            [&](int index){ updateTagNodePosition(index); }));
    tagGroupConnections.add(
        tags->sigOffsetPositionChanged().connect(
            [&](const Position& T){ setOriginOffset(T); }));
}


void ScenePositionTagGroup::finalize()
{
    tagGroupConnections.disconnect();
    itemImpl = nullptr;
}


void ScenePositionTagGroup::addTagNode(int index, bool doNotify)
{
    auto tag = itemImpl->tags->tagAt(index);
    auto node = new SgPosTransform;
    node->addChild(getOrCreateTagMarker());
    node->setPosition(tag->position());
    offsetTransform->insertChild(index, node);
    if(doNotify){
        update.resetAction(SgUpdate::ADDED);
        offsetTransform->notifyUpdate(update);
    }
}
    

SgNode* ScenePositionTagGroup::getOrCreateTagMarker()
{
    static SgNodePtr marker;

    if(!marker){
        auto lines = new SgLineSet;

        auto& vertices = *lines->getOrCreateVertices(4);
        constexpr float r = 1.0f;
        vertices[0] << 0.0f,     0.0f,     0.0f;     // Origin
        vertices[1] << 0.0f,     0.0f,     r * 3.0f; // Z direction
        vertices[2] << r * 1.0f, 0.0f,     0.0f;     // X direction
        vertices[3] << 0.0f,     r * 1.0f, 0.0f;     // Y direction
        
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

        auto fixedPixelSizeGroup = new SgFixedPixelSizeGroup;
        fixedPixelSizeGroup->setPixelSizeRatio(7.0f);
        fixedPixelSizeGroup->addChild(lines);
        
        marker = fixedPixelSizeGroup;
    }

    return marker;
}


void ScenePositionTagGroup::removeTagNode(int index)
{
    offsetTransform->removeChildAt(index);
    update.resetAction(SgUpdate::REMOVED);
    offsetTransform->notifyUpdate(update);
}


void ScenePositionTagGroup::updateTagNodePosition(int index)
{
    auto tag = itemImpl->tags->tagAt(index);
    auto node = static_cast<SgPosTransform*>(offsetTransform->child(index));
    node->setTranslation(tag->translation());
    update.resetAction(SgUpdate::MODIFIED);
    node->notifyUpdate(update);
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


void ScenePositionTagGroup::setOriginMarkerVisible(bool on)
{
    if(!originMarker){
        originMarker = new CoordinateFrameMarker;
    }
    int last = offsetTransform->numChildren() - 1;
    if(on){
        if(last < 0 || offsetTransform->child(last) != originMarker){
            offsetTransform->addChild(originMarker);
            update.resetAction(SgUpdate::ADDED);
            offsetTransform->notifyUpdate(update);
        }
    } else {
        if(last >= 0 && offsetTransform->child(last) == originMarker){
            offsetTransform->removeChildAt(last);
            update.resetAction(SgUpdate::REMOVED);
            offsetTransform->notifyUpdate(update);
        }
    }
}


bool ScenePositionTagGroup::isOriginMarkerVisible() const
{
    int last = offsetTransform->numChildren() - 1;
    return (last >= 0) && (offsetTransform->child(last) == originMarker);
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
