#include "LinkPositionWidget.h"
#include "KinematicsBar.h"
#include <cnoid/PositionWidget>
#include <cnoid/BodyItem>
#include <cnoid/Body>
#include <cnoid/Link>
#include <cnoid/JointPath>
#include <cnoid/JointSpaceConfigurationHandler>
#include <cnoid/CompositeBodyIK>
#include <cnoid/BodyItemKinematicsKit>
#include <cnoid/CoordinateFrameList>
#include <cnoid/BodyState>
#include <cnoid/MenuManager>
#include <cnoid/Archive>
#include <cnoid/Buttons>
#include <cnoid/CheckBox>
#include <cnoid/ComboBox>
#include <cnoid/ButtonGroup>
#include <cnoid/Dialog>
#include <cnoid/TreeWidget>
#include <cnoid/LineEdit>
#include <cnoid/ConnectionSet>
#include <cnoid/EigenUtil>
#include <cnoid/Selection>
#include <QScrollArea>
#include <QLabel>
#include <QGridLayout>
#include <QStyle>
#include <QDialogButtonBox>
#include <fmt/format.h>
#include <unordered_set>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

namespace {

const QString normalStyle("font-weight: normal");
const QString errorStyle("font-weight: bold; color: red");

enum FrameType { BaseFrame, OffsetFrame };

class ConfTreeWidget : public TreeWidget
{
public:
    virtual QSize sizeHint() const override;
};

class JointSpaceConfigurationDialog : public Dialog
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    LinkPositionWidget::Impl* baseImpl;
    BodyPtr body;
    shared_ptr<JointPath> jointPath;
    shared_ptr<JointSpaceConfigurationHandler> configuration;
    Isometry3 T0;
    BodyState bodyState0;
    ConfTreeWidget treeWidget;
    LineEdit searchBox;
    CheckBox feasibleCheck;
    int lastSortedSection;
    Qt::SortOrder lastSortOrder;
    QRect lastPosition;
    PushButton* applyButton;

    JointSpaceConfigurationDialog(LinkPositionWidget::Impl* baseImpl);
    void setUserInputEnabled(bool on);
    void reset();
    bool updateConfigurationTypes();
    void updateConfigurationStates();
    void updateItemDisplay();
    void onSectionClicked(int index);
    void applyConfiguration(int id);
    void onCanceled();
    virtual void hideEvent(QHideEvent* event) override;
};

}

namespace cnoid {

class LinkPositionWidget::Impl
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    LinkPositionWidget* self;

    ScopedConnectionSet targetConnections;
    BodyItemPtr targetBodyItem;
    LinkPtr targetLink;
    int targetLinkType;
    BodyItemKinematicsKitPtr kinematicsKit;
    ScopedConnectionSet kinematicsKitConnections;
    CoordinateFramePtr identityFrame;
    CoordinateFramePtr baseFrame;
    ScopedConnection baseFrameConnection;
    CoordinateFramePtr offsetFrame;
    ScopedConnection offsetFrameConnection;
    FrameLabelFunction frameLabelFunction[2];

    ScopedConnection kinematicsBarConnection;

    bool isUserInputEnabled;

    QLabel resultLabel;
    PushButton fetchButton;
    PushButton applyButton;
    QLabel coordLabel;

    enum CoordinateMode { WorldCoordinateMode, BaseFrameCoordinateMode, LocalCoordinateMode, NumCoordinateModes };
    Selection coordinateModeSelection;
    int coordinateMode;
    int preferredCoordinateMode;
    ButtonGroup coordinateModeGroup;
    RadioButton worldCoordRadio;
    RadioButton baseCoordRadio;
    RadioButton localCoordRadio;

    // Temporary origin of the local coordinate
    Isometry3 T_local0;
    Matrix3 R_local0;

    PositionWidget* positionWidget;

    QLabel frameComboLabel[2];
    ComboBox frameCombo[2];
    bool isFrameSelectionEnabled[2];
    
    QLabel configurationLabel;
    vector<int> currentConfigurationTypes;
    string configurationString;
    std::unordered_set<string> tmpConfigurationLabelSet;
    ToolButton configurationButton;
    vector<QWidget*> configurationWidgets;
    JointSpaceConfigurationDialog* configurationDialog;
    
    Impl(LinkPositionWidget* self);
    void createPanel();
    void setUserInputEnabled(bool on);
    void onAttachedMenuRequest(MenuManager& menuManager);
    void setCoordinateMode(int mode, bool doUpdatePreferredMode, bool doUpdateDisplay);
    void setTargetBodyAndLink(BodyItem* bodyItem, Link* link);
    void updateTargetLink(Link* link);
    void onKinematicsModeChanged();
    void setMainInterfaceEnabled(bool on);
    void setCoordinateFrameInterfaceEnabled(int frameComboIndex, bool isEnabled);
    void updateCoordinateFrameCandidates();
    void updateCoordinateFrameCandidates(int frameComboIndex);
    void updateCoordinateFrameComboItems(
        QComboBox& combo, CoordinateFrameList* frames, const GeneralId& currentId,
        const FrameLabelFunction& frameLabelFunction);
    void updateCurrentCoordinateFrameLabel(int frameComboIndex);
    string defaultFrameLabelFunction(CoordinateFrame* frame, bool isDefaultFrame, const char* defaultFrameNote);
    void onFrameComboActivated(int frameComboIndex, int index);
    void onFrameSetChanged();
    void setBaseFrame(CoordinateFrame* frame);
    void setOffsetFrame(CoordinateFrame* frame);
    void onFrameUpdated(int flags, int frameComboIndex);
    void setConfigurationInterfaceEnabled(bool on);
    void initializeConfigurationInterface();
    void showConfigurationDialog();
    void applyConfiguration(int id);
    void onKinematicsKitPositionError(const Isometry3& T_frameCoordinate);
    void updateDisplay();
    void updateDisplayWithPosition(const Isometry3& position);
    void updateDisplayWithGlobalLinkPosition(const Isometry3& Ta_global);
    void updateDisplayWithCurrentLinkPosition();
    void updateConfigurationDisplay();
    bool applyPositionInput(const Isometry3& T);
    bool findBodyIkSolution(const Isometry3& T_input, bool isRawT);
    void finishPositionEditing();
    bool storeState(Archive* archive);
    bool restoreState(const Archive* archive);
};

}


LinkPositionWidget::LinkPositionWidget(QWidget* parent)
    : QWidget(parent)
{
    impl = new Impl(this);
}


LinkPositionWidget::Impl::Impl(LinkPositionWidget* self)
    : self(self),
      coordinateModeSelection(NumCoordinateModes, CNOID_GETTEXT_DOMAIN_NAME)
{
    createPanel();
    
    targetLinkType = IkLink;
    identityFrame = new CoordinateFrame;
    setBaseFrame(identityFrame);
    setOffsetFrame(identityFrame);
    isUserInputEnabled = true;

    kinematicsBarConnection =
        KinematicsBar::instance()->sigKinematicsModeChanged().connect(
            [this](){ onKinematicsModeChanged(); });
}


LinkPositionWidget::~LinkPositionWidget()
{
    delete impl;
}


void LinkPositionWidget::Impl::createPanel()
{
    auto vbox = new QVBoxLayout;
    auto s = self->style();
    int lmargin = s->pixelMetric(QStyle::PM_LayoutLeftMargin);
    int rmargin = s->pixelMetric(QStyle::PM_LayoutRightMargin);
    //vbox->setContentsMargins(lmargin / 2.0, 0, rmargin / 2, 0);
    vbox->setContentsMargins(lmargin, 0, rmargin, 0);
    self->setLayout(vbox);

    auto hbox = new QHBoxLayout;
    resultLabel.setFrameStyle(QFrame::Box | QFrame::Sunken);
    resultLabel.setAlignment(Qt::AlignCenter);
    hbox->addWidget(&resultLabel, 1);
    fetchButton.setText(_("Fetch"));
    fetchButton.sigClicked().connect([this](){ updateDisplay(); });
    hbox->addWidget(&fetchButton);
    applyButton.setText(_("Apply"));
    applyButton.sigClicked().connect([this](){ positionWidget->applyPositionInput(); });
    hbox->addWidget(&applyButton);
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout;
    coordLabel.setText(_("Coord:"));
    hbox->addWidget(&coordLabel);
    
    coordinateModeSelection.setSymbol(WorldCoordinateMode, "world");
    worldCoordRadio.setText(_("World"));
    worldCoordRadio.setChecked(true);
    hbox->addWidget(&worldCoordRadio);
    coordinateModeGroup.addButton(&worldCoordRadio, WorldCoordinateMode);

    coordinateModeSelection.setSymbol(BaseFrameCoordinateMode, "base");
    baseCoordRadio.setText(_("Base"));
    hbox->addWidget(&baseCoordRadio);
    coordinateModeGroup.addButton(&baseCoordRadio, BaseFrameCoordinateMode);

    coordinateModeSelection.setSymbol(LocalCoordinateMode, "local");
    localCoordRadio.setText(_("Local"));
    hbox->addWidget(&localCoordRadio);
    coordinateModeGroup.addButton(&localCoordRadio, LocalCoordinateMode);

    coordinateMode = WorldCoordinateMode;
    preferredCoordinateMode = BaseFrameCoordinateMode;

    coordinateModeGroup.sigButtonToggled().connect(
        [this](int mode, bool on){
            if(on){
                setCoordinateMode(mode, true, true);
            }
        });
    
    hbox->addStretch();
    vbox->addLayout(hbox);

    positionWidget = new PositionWidget(self);
    positionWidget->setUserInputValuePriorityMode(true);
    positionWidget->setCallbacks(
        [this](const Isometry3& T){ return applyPositionInput(T); },
        [this](){ finishPositionEditing(); });
    vbox->addWidget(positionWidget);

    setMainInterfaceEnabled(false);

    auto grid = new QGridLayout;
    int row = 0;
    grid->setColumnStretch(1, 1);

    frameComboLabel[BaseFrame].setText(_("Base"));
    frameLabelFunction[BaseFrame] =
        [this](BodyItemKinematicsKit* kit, CoordinateFrame* frame, bool isDefaultFrame){
            return defaultFrameLabelFunction(frame, isDefaultFrame, _("Body Origin")); };

    frameComboLabel[OffsetFrame].setText(_("Offset"));
    frameLabelFunction[OffsetFrame] =
        [this](BodyItemKinematicsKit* kit, CoordinateFrame* frame, bool isDefaultFrame){
            return defaultFrameLabelFunction(frame, isDefaultFrame, _("Link Origin")); };

    for(int i=0; i < 2; ++i){
        grid->addWidget(&frameComboLabel[i], row + i, 0, Qt::AlignLeft /* Qt::AlignJustify */);
        
        frameCombo[i].sigAboutToShowPopup().connect(
            [=](){ updateCoordinateFrameCandidates(i); });
        
        frameCombo[i].sigActivated().connect(
            [=](int index){ onFrameComboActivated(i, index); });
        
        grid->addWidget(&frameCombo[i], row + i, 1, 1, 2);

        setCoordinateFrameInterfaceEnabled(i, false);
        isFrameSelectionEnabled[i] = false;
    }
    row += 2;

    auto configTitle = new QLabel(_("Config"));
    grid->addWidget(configTitle, row, 0, Qt::AlignLeft);
    configurationWidgets.push_back(configTitle);
    grid->addWidget(&configurationLabel, row, 1,  Qt::AlignLeft);
    configurationWidgets.push_back(&configurationLabel);
    configurationButton.setText(_("Set"));
    configurationButton.sigClicked().connect([this](){ showConfigurationDialog(); });
    grid->addWidget(&configurationButton, row, 2);
    configurationWidgets.push_back(&configurationButton);
    configurationDialog = nullptr;
    vbox->addLayout(grid);

    setConfigurationInterfaceEnabled(false);

    vbox->addStretch();
}


void LinkPositionWidget::customizeCoordinateModeLabels
(const char* worldModeLabel, const char* baseModeLabel, const char* localModeLabel)
{
    impl->worldCoordRadio.setText(worldModeLabel);
    impl->baseCoordRadio.setText(baseModeLabel);
    impl->localCoordRadio.setText(localModeLabel);
}


void LinkPositionWidget::customizeBaseFrameLabels(const char* caption, FrameLabelFunction labelFunction)
{
    impl->frameComboLabel[BaseFrame].setText(caption);
    impl->frameLabelFunction[BaseFrame] = labelFunction;
}


void LinkPositionWidget::customizeOffsetFrameLabels(const char* caption, FrameLabelFunction labelFunction)
{
    impl->frameComboLabel[OffsetFrame].setText(caption);
    impl->frameLabelFunction[OffsetFrame] = labelFunction;
}


void LinkPositionWidget::Impl::setUserInputEnabled(bool on)
{
    if(on != isUserInputEnabled){
        applyButton.setEnabled(on);
        positionWidget->setEditable(on);
        if(configurationDialog){
            configurationDialog->setUserInputEnabled(on);
        }
        isUserInputEnabled = on;
    }
}


void LinkPositionWidget::setOptionMenuTo(MenuManager& menuManager)
{
    impl->positionWidget->setOptionMenuTo(menuManager);

    menuManager.addSeparator();

    auto disableCustomIkCheck = menuManager.addCheckItem(_("Disable custom IK"));
    disableCustomIkCheck->setChecked(!isCustomIkEnabled());
    disableCustomIkCheck->sigToggled().connect(
        [this](bool on){ setCustomIkEnabled(on); });
}


void LinkPositionWidget::Impl::setCoordinateMode(int mode, bool doUpdatePreferredMode, bool doUpdateDisplay)
{
    bool isValid = false;
    
    coordinateModeGroup.blockSignals(true);

    if(mode == LocalCoordinateMode){
        localCoordRadio.setChecked(true);
        if(targetLink){
            T_local0 = targetLink->T() * offsetFrame->T();
            R_local0 = T_local0.linear();
        }
        positionWidget->setReferenceRpy(Vector3::Zero());
        isValid = true;

    } else if(mode == BaseFrameCoordinateMode){
        if(baseCoordRadio.isEnabled()){
            baseCoordRadio.setChecked(true);
            isValid = true;
        }
    }
    if(!isValid){
        if(mode != WorldCoordinateMode){
            mode = WorldCoordinateMode;
            doUpdatePreferredMode = false;
        }
        worldCoordRadio.setChecked(true);
        isValid = true;
    }

    coordinateModeGroup.blockSignals(false);

    if(isValid){
        coordinateMode = mode;
        if(doUpdatePreferredMode){
            preferredCoordinateMode = mode;
        }
        if(doUpdateDisplay){
            updateDisplay();
        }
    }
}


void LinkPositionWidget::setTargetLinkType(int type)
{
    if(type != impl->targetLinkType){
        impl->targetLinkType = type;
        impl->setTargetBodyAndLink(impl->targetBodyItem, impl->targetLink);
    }
}


int LinkPositionWidget::targetLinkType() const
{
    return impl->targetLinkType;
}


void LinkPositionWidget::setCustomIkEnabled(bool on)
{
    if(impl->kinematicsKit){
        impl->kinematicsKit->setCustomIkDisabled(on);
        impl->initializeConfigurationInterface();
        impl->updateDisplay();
    }
}


bool LinkPositionWidget::isCustomIkEnabled() const
{
    if(impl->kinematicsKit){
        return !impl->kinematicsKit->isCustomIkDisabled();
    }
    return false;
}


BodyItem* LinkPositionWidget::targetBodyItem()
{
    return impl->targetBodyItem;
}


Link* LinkPositionWidget::targetLink()
{
    return impl->targetLink;
}


void LinkPositionWidget::setTargetBodyAndLink(BodyItem* bodyItem, Link* link)
{
    impl->setTargetBodyAndLink(bodyItem, link);
}


void LinkPositionWidget::Impl::setTargetBodyAndLink(BodyItem* bodyItem, Link* link)
{
    if(bodyItem && link){
        // Sub body's root link is recognized as the parent body's end link
        if(link->isRoot() && bodyItem->isAttachedToParentBody()){
            if(auto parentBodyItem = bodyItem->parentBodyItem()){
                link = bodyItem->body()->parentBodyLink();
                bodyItem = parentBodyItem;
            }
        }
        bool isIkLinkRequired = (targetLinkType != AnyLink);
        if(targetLinkType == RootOrIkLink){
            isIkLinkRequired = !link->isRoot();
        }
        if(isIkLinkRequired){
            if(!bodyItem->findPresetIK(link)){
                auto orgLink = link;
                link = nullptr;
                LinkTraverse traverse(orgLink, false, true);
                for(auto& node : traverse){
                    if(bodyItem->findPresetIK(node)){
                        link = node;
                        break;
                    }
                }
                if(!link){
                    traverse.find(orgLink, true, false);
                    for(auto& node : traverse){
                        if(bodyItem->findPresetIK(node)){
                            link = node;
                            break;
                        }
                    }
                }
            }
        }
    }

    if(bodyItem != targetBodyItem){
        positionWidget->clearPosition();
        targetConnections.disconnect();

        targetBodyItem = bodyItem;

        if(bodyItem){
            targetConnections.add(
                bodyItem->sigNameChanged().connect(
                    [this](const std::string&){ updateTargetLink(targetLink); }));

            targetConnections.add(
                bodyItem->sigKinematicStateChanged().connect(
                    [this](){ updateDisplayWithCurrentLinkPosition(); }));

            targetConnections.add(
                bodyItem->sigUpdated().connect(
                    [this](){ updateTargetLink(targetLink); }));

            targetConnections.add(
                bodyItem->sigContinuousKinematicUpdateStateChanged().connect(
                    [this](bool on){ setUserInputEnabled(!on); }));

            setUserInputEnabled(!bodyItem->isDoingContinuousKinematicUpdate());
        }
    }

    if(link != targetLink){
        updateTargetLink(link);
        updateDisplayWithCurrentLinkPosition();
    }
}


void LinkPositionWidget::Impl::updateTargetLink(Link* link)
{
    targetLink = link;
    kinematicsKit.reset();
    kinematicsKitConnections.disconnect();
    setBaseFrame(identityFrame);
    setOffsetFrame(identityFrame);
    bool isBaseCoordinateModeEnabled = false;
    
    if(targetLink){
        auto body = targetBodyItem->body();

        kinematicsKit = targetBodyItem->getCurrentKinematicsKit(targetLink);
        if(kinematicsKit){
            kinematicsKitConnections.add(
                kinematicsKit->sigFrameSetChanged().connect(
                    [this](){ onFrameSetChanged(); }));
            kinematicsKitConnections.add(
                kinematicsKit->sigPositionError().connect(
                    [this](const Isometry3& T_frameCoordinate){
                        onKinematicsKitPositionError(T_frameCoordinate); }));

            if(kinematicsKit->baseLink() && link != kinematicsKit->baseLink()){
                isBaseCoordinateModeEnabled = true;
            }
            setBaseFrame(kinematicsKit->currentBaseFrame());
            setOffsetFrame(kinematicsKit->currentOffsetFrame());
        }
    }

    bool isEditable = kinematicsKit != nullptr;

    setMainInterfaceEnabled(isEditable);
    baseCoordRadio.setEnabled(isBaseCoordinateModeEnabled);
    isFrameSelectionEnabled[BaseFrame] = isBaseCoordinateModeEnabled;
    isFrameSelectionEnabled[OffsetFrame] = true;
    updateCoordinateFrameCandidates();
    setCoordinateMode(preferredCoordinateMode, false, false);
    resultLabel.setText("");

    initializeConfigurationInterface();
}


void LinkPositionWidget::Impl::onKinematicsModeChanged()
{
    if(targetLink){
        updateTargetLink(targetLink);
    }
}


void LinkPositionWidget::Impl::setMainInterfaceEnabled(bool on)
{
    resultLabel.setEnabled(on);
    fetchButton.setEnabled(on);
    applyButton.setEnabled(on);
    coordLabel.setEnabled(on);
    worldCoordRadio.setEnabled(on);
    baseCoordRadio.setEnabled(on);
    localCoordRadio.setEnabled(on);
    positionWidget->setEnabled(on);
}


void LinkPositionWidget::Impl::setCoordinateFrameInterfaceEnabled(int frameComboIndex, bool on)
{
    auto& combo = frameCombo[frameComboIndex];
    if(on != combo.isEnabled()){
        combo.setEnabled(on);
        frameComboLabel[frameComboIndex].setEnabled(on);
    }
}


void LinkPositionWidget::Impl::updateCoordinateFrameCandidates()
{
    for(int i=0; i < 2; ++i){
        updateCoordinateFrameCandidates(i);
    }
}


void LinkPositionWidget::Impl::updateCoordinateFrameCandidates(int frameComboIndex)
{
    CoordinateFrameList* frames = nullptr;
    GeneralId currentFrameId = GeneralId::defaultId();

    if(kinematicsKit){
        if(frameComboIndex == BaseFrame){
            frames = kinematicsKit->baseFrames();
            currentFrameId = kinematicsKit->currentBaseFrameId();
        } else if(frameComboIndex == OffsetFrame){
            frames = kinematicsKit->offsetFrames();
            currentFrameId = kinematicsKit->currentOffsetFrameId();
        }
    }
    bool hasCandidates = false;
    auto& combo = frameCombo[frameComboIndex];
    combo.clear();
    if(isFrameSelectionEnabled[frameComboIndex] && frames){
        auto& labelFunction = frameLabelFunction[frameComboIndex];
        if(frames->numFrames() > 0){
            updateCoordinateFrameComboItems(combo, frames, currentFrameId, labelFunction);
            hasCandidates = true;
        } else if(frameComboIndex == BaseFrame){
            // It is better to show the body origin label on the base frame combo
            string label = labelFunction(kinematicsKit, baseFrame, true);
            combo.addItem(label.c_str(), 0);
            hasCandidates = true;
        }
    }

    setCoordinateFrameInterfaceEnabled(frameComboIndex, hasCandidates);
}


void LinkPositionWidget::Impl::updateCoordinateFrameComboItems
(QComboBox& combo, CoordinateFrameList* frames, const GeneralId& currentId,
 const FrameLabelFunction& frameLabelFunction)
{
    int currentIndex = 0;
    const int n = frames->numFrames();
    for(int i=0; i < n; ++i){
        int index = combo.count();
        if(auto frame = frames->frameAt(i)){
            auto& id = frame->id();
            bool isDefaultFrame = (i == 0 && frames->hasFirstElementAsDefaultFrame());
            string label = frameLabelFunction(kinematicsKit, frame, isDefaultFrame);
            if(id.isInt()){
                combo.addItem(label.c_str(), id.toInt());
            } else {
                combo.addItem(label.c_str(), id.toString().c_str());
            }
            if(id == currentId){
                currentIndex = index;
            }
        }
    }
    combo.setCurrentIndex(currentIndex);
}


void LinkPositionWidget::Impl::updateCurrentCoordinateFrameLabel(int frameComboIndex)
{
    CoordinateFrameList* frames = nullptr;
    if(kinematicsKit){
        if(frameComboIndex == BaseFrame){
            frames = kinematicsKit->baseFrames();
        } else if(frameComboIndex == OffsetFrame){
            frames = kinematicsKit->offsetFrames();
        }
    }
    if(frames){
        auto& combo = frameCombo[frameComboIndex];
        int index = combo.currentIndex();
        //! Do not update the label of the default frame with index 0
        if(index < frames->numFrames()){
            auto frame = frames->frameAt(index);
            auto& labelFunction = frameLabelFunction[frameComboIndex];
            bool isDefaultFrame = (index == 0 && frames->hasFirstElementAsDefaultFrame());
            string label = labelFunction(kinematicsKit, frame, isDefaultFrame);
            combo.setItemText(index, label.c_str());
            auto& id = frame->id();
            if(id.isInt()){
                combo.setItemData(index, id.toInt());
            } else {
                combo.setItemData(index, id.toString().c_str());
            }
        }
    }
}


string LinkPositionWidget::Impl::defaultFrameLabelFunction
(CoordinateFrame* frame, bool isDefaultFrame, const char* defaultFrameNote)
{
    string label;
    string note = frame->note();
    if(note.empty() && isDefaultFrame){
        note = defaultFrameNote;
    }
    const auto& id = frame->id();
    if(!id.isValid()){
        return note;
    }
    if(note.empty()){
        return id.label();
    }
    return format("{0}: {1}", id.label(), note.c_str());
}


void LinkPositionWidget::Impl::onFrameComboActivated(int frameComboIndex, int index)
{
    GeneralId id;
    auto idValue = frameCombo[frameComboIndex].itemData(index);
    if(idValue.userType() == QMetaType::Int){
        id = idValue.toInt();
    } else if(idValue.userType() == QMetaType::QString){
        id = idValue.toString().toStdString();
    }
    if(id.isValid()){
        if(!kinematicsKit){
            updateDisplay();
        } else {
            if(frameComboIndex == BaseFrame){
                kinematicsKit->setCurrentBaseFrame(id);
                setBaseFrame(kinematicsKit->currentBaseFrame());
            } else {
                kinematicsKit->setCurrentOffsetFrame(id);
                setOffsetFrame(kinematicsKit->currentOffsetFrame());
            }
            // onFrameSetChanged is called and updateDisplay is then called
            kinematicsKit->notifyFrameSetChange();
        }
    }
}


void LinkPositionWidget::Impl::onFrameSetChanged()
{
    bool coordinateModeUpdated = false;

    updateCoordinateFrameCandidates();

    for(int i=0; i < 2; ++i){
        GeneralId newId;
        if(i == BaseFrame){
            newId = kinematicsKit->currentBaseFrameId();
        } else {
            newId = kinematicsKit->currentOffsetFrameId();
        }
        auto& combo = frameCombo[i];
        int currentIndex = combo.currentIndex();
        for(int j=0; j < combo.count(); ++j){
            GeneralId id;
            auto idValue = combo.itemData(j);
            if(idValue.userType() == QMetaType::Int){
                id = idValue.toInt();
            } else if(idValue.userType() == QMetaType::QString){
                id = idValue.toString().toStdString();
            }
            if(id == newId){
                currentIndex = j;
                break;
            }
        }
        if(currentIndex != combo.currentIndex()){
            combo.setCurrentIndex(currentIndex);
        }
    }

    setBaseFrame(kinematicsKit->currentBaseFrame());
    setOffsetFrame(kinematicsKit->currentOffsetFrame());
    
    updateDisplay();
}


void LinkPositionWidget::Impl::setBaseFrame(CoordinateFrame* frame)
{
    if(frame != baseFrame){
        baseFrame = frame;
        if(frame == identityFrame){
            baseFrameConnection.disconnect();
        } else {
            baseFrameConnection =
                baseFrame->sigUpdated().connect(
                    [this](int flags){ onFrameUpdated(flags, BaseFrame); });
        }
    }
}


void LinkPositionWidget::Impl::setOffsetFrame(CoordinateFrame* frame)
{
    if(frame != offsetFrame){
        offsetFrame = frame;
        if(frame == identityFrame){
            offsetFrameConnection.disconnect();
        } else {
            offsetFrameConnection =
                offsetFrame->sigUpdated().connect(
                    [this](int flags){ onFrameUpdated(flags, OffsetFrame); });
        }
    }
}


void LinkPositionWidget::Impl::onFrameUpdated(int flags, int frameComboIndex)
{
    bool doUpdateDisplay = false;

    if(flags & CoordinateFrame::IdUpdate){
        updateCoordinateFrameCandidates(frameComboIndex);
        doUpdateDisplay = true;
        
    } else if(flags & CoordinateFrame::NoteUpdate){
        updateCurrentCoordinateFrameLabel(frameComboIndex);
    }

    if(flags & (CoordinateFrame::ModeUpdate | CoordinateFrame::PositionUpdate)){
        doUpdateDisplay = true;
    }

    if(doUpdateDisplay){
        updateDisplay();
    }
}


void LinkPositionWidget::Impl::setConfigurationInterfaceEnabled(bool on)
{
    for(auto& widget : configurationWidgets){
        widget->setEnabled(on);
    }
    if(!on){
        configurationLabel.setText("-----");
        configurationLabel.setToolTip(QString::Null());
    }
}
    

void LinkPositionWidget::Impl::initializeConfigurationInterface()
{
    currentConfigurationTypes.clear();

    bool isConfigurationValid = false;
    if(kinematicsKit && !kinematicsKit->isCustomIkDisabled()){
        if(auto configurationHandler = kinematicsKit->configurationHandler()){
            isConfigurationValid = true;
        }
    }

    setConfigurationInterfaceEnabled(isConfigurationValid);

    if(configurationDialog){
        if(isConfigurationValid){
            configurationDialog->updateConfigurationTypes();
        } else {
            configurationDialog->reset();
        }
    }

}


void LinkPositionWidget::Impl::showConfigurationDialog()
{
    if(!configurationDialog){
        configurationDialog = new JointSpaceConfigurationDialog(this);
        if(targetBodyItem){
            configurationDialog->setUserInputEnabled(!targetBodyItem->isDoingContinuousKinematicUpdate());
        }
    }
    
    if(configurationDialog->updateConfigurationTypes()){
        if(configurationDialog->isHidden()){
            configurationDialog->show();
            if(!configurationDialog->lastPosition.isNull()){
                configurationDialog->treeWidget.setSizeAdjustPolicy(
                    QAbstractScrollArea::AdjustIgnored);
                configurationDialog->setGeometry(configurationDialog->lastPosition);
            }
        }
    }
}


JointSpaceConfigurationDialog::JointSpaceConfigurationDialog(LinkPositionWidget::Impl* baseImpl)
    : Dialog(baseImpl->self, Qt::Tool),
      baseImpl(baseImpl)
{
    setSizeGripEnabled(true);
    
    auto vbox = new QVBoxLayout;

    auto hbox = new QHBoxLayout;
    hbox->addWidget(new QLabel(_("Search")));
    searchBox.setEnabled(false);
    hbox->addWidget(&searchBox, 1);
    feasibleCheck.setText(_("Feasible"));
    feasibleCheck.sigToggled().connect([this](bool){ updateItemDisplay(); });
    hbox->addWidget(&feasibleCheck);
    vbox->addLayout(hbox);

    auto header = treeWidget.header();
    header->setMinimumSectionSize(1);
    header->setSectionResizeMode(QHeaderView::ResizeToContents);
    header->setSectionsClickable(true);
    QObject::connect(header, &QHeaderView::sectionClicked,
                     [this](int index){ onSectionClicked(index); });

    treeWidget.setSizeAdjustPolicy(QAbstractScrollArea::AdjustToContentsOnFirstShow);
    treeWidget.setRootIsDecorated(false);

    treeWidget.sigCurrentItemChanged().connect(
        [this](QTreeWidgetItem* item, QTreeWidgetItem*){
            if(item){
                int id = item->data(0, Qt::UserRole).toInt();
                applyConfiguration(id);
            }
        });
    vbox->addWidget(&treeWidget);

    hbox = new QHBoxLayout;
    auto updateButton = new PushButton(_("&Update"));
    updateButton->sigClicked().connect([this](){ updateConfigurationStates(); });
    hbox->addWidget(updateButton);
    hbox->addStretch();

    applyButton = new PushButton(_("&Apply"));
    applyButton->setDefault(true);
    auto cancelButton = new PushButton(_("&Cancel"));
    auto buttonBox = new QDialogButtonBox(this);
    buttonBox->addButton(applyButton, QDialogButtonBox::AcceptRole);
    buttonBox->addButton(cancelButton, QDialogButtonBox::RejectRole);
    connect(buttonBox, SIGNAL(accepted()), this, SLOT(accept()));
    connect(buttonBox, &QDialogButtonBox::rejected, [this](){ onCanceled(); });
    hbox->addWidget(buttonBox);

    vbox->addLayout(hbox);
    
    setLayout(vbox);
}


void JointSpaceConfigurationDialog::setUserInputEnabled(bool on)
{
    treeWidget.setEnabled(on);
    applyButton->setEnabled(on);
}


void JointSpaceConfigurationDialog::reset()
{
    body.reset();
    jointPath.reset();
    configuration.reset();
    treeWidget.clear();
    lastSortedSection = -1;
}


bool JointSpaceConfigurationDialog::updateConfigurationTypes()
{
    reset();

    auto& kinematicsKit = baseImpl->kinematicsKit;
    if(!kinematicsKit){
        return false;
    }
    body = kinematicsKit->body();
    jointPath = kinematicsKit->jointPath();
    configuration = kinematicsKit->configurationHandler();
    if(!jointPath || !configuration){
        return false;
    }

    T0 = jointPath->endLink()->T();
    bodyState0.storePositions(*body);
    
    auto name = jointPath->name();
    if(name.empty()){
        setWindowTitle(_("Joint-space configuration"));
    } else {
        setWindowTitle(format(_("{} configuration"), name).c_str());
    }

    int n = configuration->getNumConfigurationTypes();
    auto targetNames = configuration->getConfigurationTargetNames();
    treeWidget.setColumnCount(targetNames.size() + 1);
    QStringList headerLabels;
    headerLabels.append("No");
    for(auto& label : targetNames){
        headerLabels.append(label.c_str());
    }
    treeWidget.setHeaderLabels(headerLabels);
    
    for(int i=0; i < n; ++i){
        int typeId = configuration->getConfigurationTypeId(i);
        auto labels = configuration->getConfigurationStateNames(typeId);
        auto item = new QTreeWidgetItem;
        item->setData(0, Qt::DisplayRole, i + 1);
        item->setTextAlignment(0, Qt::AlignCenter);
        item->setData(0, Qt::UserRole, typeId);
        for(size_t j=0; j < labels.size(); ++j){
            item->setText(j + 1, labels[j].c_str());
        }
        treeWidget.addTopLevelItem(item);
    }

    updateConfigurationStates();

    return true;
}


void JointSpaceConfigurationDialog::updateConfigurationStates()
{
    int n = treeWidget.topLevelItemCount();
    if(n == 0){
        return;
    }
    
    T0 = jointPath->endLink()->T();
    bodyState0.storePositions(*body);

    for(int i=0; i < n; ++i){
        auto item = treeWidget.topLevelItem(i);
        int id = item->data(0, Qt::UserRole).toInt();
        configuration->setPreferredConfigurationType(id);
        bool solved = jointPath->calcInverseKinematics(T0);
        if(solved){
            for(auto& joint : jointPath->joints()){
                if(joint->q() > joint->q_upper() || joint->q() < joint->q_lower()){
                    solved = false;
                    break;
                }
            }
        }
        item->setData(0, Qt::UserRole + 1, solved);
    }

    configuration->resetPreferredConfigurationType();
    bodyState0.restorePositions(*body);
    jointPath->endLink()->T() = T0;

    updateItemDisplay();
}


void JointSpaceConfigurationDialog::updateItemDisplay()
{
    int n = treeWidget.topLevelItemCount();
    if(n == 0){
        return;
    }
    QBrush red(Qt::red);
    QVariant none;
    for(int i=0; i < n; ++i){
        auto item = treeWidget.topLevelItem(i);
        bool solved = item->data(0, Qt::UserRole + 1).toBool();
        for(int j=0; j < item->columnCount(); ++j){
            if(solved){
                item->setData(j, Qt::ForegroundRole, none);
            } else {
                item->setForeground(j, red);
            }
            if(feasibleCheck.isChecked()){
                item->setHidden(!solved);
            } else {
                item->setHidden(false);
            }
        }
    }
}


void JointSpaceConfigurationDialog::onSectionClicked(int index)
{
    Qt::SortOrder order;
    if(index != lastSortedSection){
        order = Qt::AscendingOrder;
    } else {
        order = (lastSortOrder != Qt::AscendingOrder) ? Qt::AscendingOrder : Qt::DescendingOrder;
    }
    treeWidget.sortByColumn(index, order);
    
    lastSortedSection = index;
    lastSortOrder = order;
}


void JointSpaceConfigurationDialog::applyConfiguration(int id)
{
    configuration->setPreferredConfigurationType(id);
    baseImpl->findBodyIkSolution(baseImpl->kinematicsKit->endLink()->T(), true);
    configuration->resetPreferredConfigurationType();
    baseImpl->updateConfigurationDisplay();
}


void JointSpaceConfigurationDialog::onCanceled()
{
    if(body){
        bodyState0.restorePositions(*body);
        jointPath->endLink()->T() = T0;
        baseImpl->targetBodyItem->notifyKinematicStateChange();
    }
        
    hide();
}


void JointSpaceConfigurationDialog::hideEvent(QHideEvent* event)
{
    lastPosition = geometry();
    Dialog::hideEvent(event);
}


QSize ConfTreeWidget::sizeHint() const
{
    int c = topLevelItemCount();
    if(c == 0){
        return TreeWidget::sizeHint();
    }
    const int frameWidth = style()->pixelMetric(QStyle::PM_DefaultFrameWidth);
    auto header_ = header();
    auto r = visualItemRect(topLevelItem(c - 1));
    return QSize(-1, r.bottom() + header_->height() + frameWidth * 2 + r.height() / 2);
}


void LinkPositionWidget::Impl::onKinematicsKitPositionError(const Isometry3& T_frameCoordinate)
{
    Isometry3 T_base;
    if(coordinateMode == WorldCoordinateMode){
        T_base.setIdentity();
    } else {
        T_base = kinematicsKit->globalBasePosition();
    }
    const auto& T_end = kinematicsKit->currentOffsetFrame()->T();
    Isometry3 Ta_global = T_base * T_frameCoordinate * T_end.inverse(Eigen::Isometry);
    updateDisplayWithGlobalLinkPosition(Ta_global);
    positionWidget->setErrorHighlight(true);
    resultLabel.setText(_("Not Solved"));
    resultLabel.setStyleSheet(errorStyle);
}


void LinkPositionWidget::Impl::updateDisplay()
{
    updateDisplayWithCurrentLinkPosition();

    resultLabel.setText(_("Actual State"));
    resultLabel.setStyleSheet(normalStyle);
}


void LinkPositionWidget::Impl::updateDisplayWithGlobalLinkPosition(const Isometry3& Ta_global)
{
    Isometry3 T;

    if(coordinateMode == LocalCoordinateMode){
        Isometry3 T_end = Ta_global * offsetFrame->T();
        Matrix3 R_prev = T_local0.linear();
        Matrix3 R_current = T_end.linear();
        // When the rotation is changed, the translation origin is reset
        if(!R_current.isApprox(R_prev, 1e-6)){
            T_local0 = T_end;
        }
        T.translation() = (T_local0.inverse(Eigen::Isometry) * T_end).translation();
        T.linear() = R_local0.transpose() * T_end.linear();

    } else {
        T = Ta_global * offsetFrame->T(); // World coordinate

        if(coordinateMode == BaseFrameCoordinateMode){
            bool isBodyCoordinate = baseFrame->isLocal();
            if(isBodyCoordinate && kinematicsKit && kinematicsKit->baseLink()){
                T = kinematicsKit->baseLink()->T().inverse(Eigen::Isometry) * T;
            }
            T = baseFrame->T().inverse(Eigen::Isometry) * T;
        }
        
        if(kinematicsKit){
            positionWidget->setReferenceRpy(kinematicsKit->referenceRpy());
        }
    }
    
    positionWidget->setPosition(T);
    updateConfigurationDisplay();
}


void LinkPositionWidget::Impl::updateDisplayWithCurrentLinkPosition()
{
    if(targetLink){
        updateDisplayWithGlobalLinkPosition(targetLink->T());
    }
}


void LinkPositionWidget::Impl::updateConfigurationDisplay()
{
    if(kinematicsKit){
        if(auto configuration = kinematicsKit->configurationHandler()){
            configurationString.clear();
            auto types = configuration->getCurrentConfigurationTypes();
            if(types != currentConfigurationTypes){
                size_t numTypes = types.size();
                vector<vector<string>> allLabels(numTypes);
                int maxNumLabels = 0;
                for(int i=0; i < numTypes; ++i){
                    allLabels[i] = configuration->getConfigurationStateNames(types[i]);
                    int n = allLabels[i].size();
                    if(n > maxNumLabels){
                        maxNumLabels = n;
                    }
                }
                for(size_t i=0; i < maxNumLabels; ++i){
                    for(size_t j=0; j < numTypes; ++j){
                        auto& labels = allLabels[j];
                        if(i < labels.size()){
                            auto& label = labels[i];
                            auto p = tmpConfigurationLabelSet.insert(label);
                            if(p.second){ // newly inserted
                                if(!configurationString.empty()){
                                    if(j == 0){
                                        configurationString.append("-");
                                    } else {
                                        configurationString.append("/");
                                    }
                                }
                                configurationString.append(label);
                            }
                        }
                    }
                }
                configurationLabel.setText(configurationString.c_str());
                configurationLabel.setToolTip(configurationLabel.text());
                currentConfigurationTypes = types;
            }
            tmpConfigurationLabelSet.clear();
        }
    }
}


bool LinkPositionWidget::Impl::applyPositionInput(const Isometry3& T)
{
    return findBodyIkSolution(T, false);
}


bool LinkPositionWidget::Impl::findBodyIkSolution(const Isometry3& T_input, bool isRawT)
{
    shared_ptr<InverseKinematics> ik;
    if(kinematicsKit){
        ik = kinematicsKit->inverseKinematics();
    }
    if(!ik){
        return false;
    }
    
    bool solved = false;
    
    kinematicsKit->setReferenceRpy(positionWidget->getRpyInput());

    if(isRawT){
        solved = ik->calcInverseKinematics(T_input);
        
    } else {
        Isometry3 T;
        Isometry3 T_end_origin = T_input * offsetFrame->T().inverse(Eigen::Isometry);

        if(coordinateMode == WorldCoordinateMode){
            T = T_end_origin;

        } else if(coordinateMode == BaseFrameCoordinateMode){
            T = baseFrame->T() * T_end_origin;
            bool isBodyCoordinate = baseFrame->isLocal();
            if(isBodyCoordinate && kinematicsKit->baseLink()){
                T = kinematicsKit->baseLink()->T() * T;
            }

        } else if(coordinateMode == LocalCoordinateMode){
            T.linear() = R_local0 * T_end_origin.linear();
            T.translation() = T_local0 * T_end_origin.translation();

            if(!T.linear().isApprox(T_local0.linear(), 1e-6)){
                T_local0 = T;
                Isometry3 T_input_fixed = T_input;
                T_input_fixed.translation().setZero();
                positionWidget->setPosition(T_input_fixed);
            }
        }

        normalizeRotation(T);
        solved = ik->calcInverseKinematics(T);
    }

    if(solved){
        ik->calcRemainingPartForwardKinematicsForInverseKinematics();

        targetConnections.block();
        targetBodyItem->notifyKinematicStateChange();
        targetConnections.unblock();

        int nearSingularPointState = 0;
        auto configuration = kinematicsKit->configurationHandler();
        if(configuration){
            nearSingularPointState = configuration->getCurrentNearSingularPointState();
        }
        if(nearSingularPointState == 0){
            resultLabel.setText(_("Solved"));
            resultLabel.setStyleSheet(normalStyle);
        } else {
            resultLabel.setText(
                configuration->getNearSingularPointFactorString(nearSingularPointState).c_str());
            resultLabel.setStyleSheet(errorStyle);
        }
    } else {
        resultLabel.setText(_("Not Solved"));
        resultLabel.setStyleSheet(errorStyle);
    }

    return solved;
}


void LinkPositionWidget::Impl::finishPositionEditing()
{
    if(targetBodyItem){
        targetBodyItem->notifyKinematicStateUpdate(false);
    }
}


bool LinkPositionWidget::storeState(Archive* archive)
{
    return impl->storeState(archive);
}


bool LinkPositionWidget::Impl::storeState(Archive* archive)
{
    coordinateModeSelection.select(coordinateMode);
    archive->write("coordinate_mode", coordinateModeSelection.selectedSymbol());
    coordinateModeSelection.select(preferredCoordinateMode);
    archive->write("preferred_coordinate_mode", coordinateModeSelection.selectedSymbol());

    positionWidget->storeState(archive);

    return true;
}


bool LinkPositionWidget::restoreState(const Archive* archive)
{
    return impl->restoreState(archive);
}


bool LinkPositionWidget::Impl::restoreState(const Archive* archive)
{
    string symbol;

    if(archive->read("preferred_coordinate_mode", symbol)){
        if(coordinateModeSelection.select(symbol)){
            preferredCoordinateMode = coordinateModeSelection.which();
        }
    }
    if(archive->read("coordinate_mode", symbol)){
        if(coordinateModeSelection.select(symbol)){
            setCoordinateMode(coordinateModeSelection.which(), false, true);
        }
    }

    positionWidget->restoreState(archive);

    return true;
}
