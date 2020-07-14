#include "LinkPositionWidget.h"
#include <cnoid/PositionWidget>
#include <cnoid/BodyItem>
#include <cnoid/Body>
#include <cnoid/Link>
#include <cnoid/JointPath>
#include <cnoid/JointSpaceConfigurationHandler>
#include <cnoid/CompositeBodyIK>
#include <cnoid/LinkKinematicsKit>
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
    /*
    QModelIndex indexFromItem(const QTreeWidgetItem* item, int column = 0) const {
        return QTreeWidget::indexFromItem(item, column);
    }
    */
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
    Position T0;
    BodyState bodyState0;
    ConfTreeWidget treeWidget;
    LineEdit searchBox;
    CheckBox feasibleCheck;
    int lastSortedSection;
    Qt::SortOrder lastSortOrder;
    QRect lastPosition;

    JointSpaceConfigurationDialog(LinkPositionWidget::Impl* baseImpl);
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
    LinkPositionWidget* self;

    ScopedConnectionSet targetConnections;
    BodyItemPtr targetBodyItem;
    LinkPtr targetLink;
    int targetLinkType;
    LinkKinematicsKitPtr kinematicsKit;
    ScopedConnectionSet kinematicsKitConnections;
    CoordinateFramePtr identityFrame;
    CoordinateFramePtr baseFrame;
    CoordinateFramePtr offsetFrame;
    FrameLabelFunction frameLabelFunction[2];
    
    QLabel resultLabel;

    enum CoordinateMode { WorldCoordinateMode, BodyCoordinateMode, LocalCoordinateMode, NumCoordinateModes };
    Selection coordinateModeSelection;
    int coordinateMode;
    int preferredCoordinateMode;
    ButtonGroup coordinateModeGroup;
    RadioButton worldCoordRadio;
    RadioButton bodyCoordRadio;
    RadioButton localCoordRadio;
    vector<QWidget*> coordinateModeWidgets;

    PositionWidget* positionWidget;

    string defaultCoordName[2];
    QLabel frameComboLabel[2];
    ComboBox frameCombo[2];
    
    QLabel configurationLabel;
    vector<int> currentConfigurationTypes;
    string configurationString;
    std::unordered_set<string> tmpConfigurationLabelSet;
    ToolButton configurationButton;
    vector<QWidget*> configurationWidgets;
    JointSpaceConfigurationDialog* configurationDialog;
    
    ScopedConnectionSet userInputConnections;

    Impl(LinkPositionWidget* self);
    void createPanel();
    void onAttachedMenuRequest(MenuManager& menuManager);
    void setCoordinateModeInterfaceEnabled(bool on);
    void setCoordinateMode(int mode, bool doUpdateDisplay);
    void setBodyCoordinateModeEnabled(bool on);
    void onCoordinateModeRadioToggled(int mode);
    void setTargetBodyAndLink(BodyItem* bodyItem, Link* link);
    void updateTargetLink(Link* link);
    void setCoordinateFrameInterfaceEnabled(bool isBaseEnabled, bool isOffsetEnabled);
    void updateCoordinateFrameCandidates();
    void updateCoordinateFrameCandidates(int frameComboIndex);
    void updateCoordinateFrameComboItems(
        QComboBox& combo, CoordinateFrameList* frames, const GeneralId& currentId,
        const FrameLabelFunction& frameLabelFunction);
    string getFrameLabel(CoordinateFrame* frame, bool isDefaultFrame, const char* defaultFrameNote);
    void onFrameComboActivated(int frameComboIndex, int index);
    void onFrameUpdate();
    void setConfigurationInterfaceEnabled(bool on);
    void initializeConfigurationInterface();
    void showConfigurationDialog();
    void applyConfiguration(int id);
    void onKinematicsKitPositionError(const Position& T_frameCoordinate);
    void updateDisplay();
    void updateDisplayWithPosition(const Position& position);
    void updateDisplayWithGlobalLinkPosition(const Position& Ta_global);
    void updateDisplayWithCurrentLinkPosition();
    void updateConfigurationDisplay();
    bool applyPositionInput(const Position& T);
    bool findBodyIkSolution(const Position& T_input, bool isRawT);
    bool storeState(Archive& archive);
    bool restoreState(const Archive& archive);
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
    self->setEnabled(false);
    
    targetLinkType = IkLink;
    
    identityFrame = new CoordinateFrame;
    baseFrame = identityFrame;
    offsetFrame = identityFrame;
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
    auto actualButton = new PushButton(_("Fetch"));
    actualButton->sigClicked().connect([&](){ updateDisplay(); });
    hbox->addWidget(actualButton);
    auto applyButton = new PushButton(_("Apply"));
    applyButton->sigClicked().connect([&](){ positionWidget->applyPositionInput(); });
    hbox->addWidget(applyButton);
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout;
    auto coordLabel = new QLabel(_("Coord:"));
    hbox->addWidget(coordLabel);
    coordinateModeWidgets.push_back(coordLabel);
    
    coordinateModeSelection.setSymbol(WorldCoordinateMode, "world");
    worldCoordRadio.setText(_("World"));
    worldCoordRadio.setChecked(true);
    hbox->addWidget(&worldCoordRadio);
    coordinateModeGroup.addButton(&worldCoordRadio, WorldCoordinateMode);
    coordinateModeWidgets.push_back(&worldCoordRadio);

    coordinateModeSelection.setSymbol(BodyCoordinateMode, "base");
    bodyCoordRadio.setText(_("Base"));
    hbox->addWidget(&bodyCoordRadio);
    coordinateModeGroup.addButton(&bodyCoordRadio, BodyCoordinateMode);
    coordinateModeWidgets.push_back(&bodyCoordRadio);

    coordinateModeSelection.setSymbol(LocalCoordinateMode, "local");
    localCoordRadio.setText(_("Local"));
    hbox->addWidget(&localCoordRadio);
    coordinateModeGroup.addButton(&localCoordRadio, LocalCoordinateMode);
    coordinateModeWidgets.push_back(&localCoordRadio);
    localCoordRadio.setEnabled(false);

    coordinateMode = WorldCoordinateMode;
    preferredCoordinateMode = BodyCoordinateMode;

    coordinateModeGroup.sigButtonToggled().connect(
        [&](int id, bool checked){
            if(checked) onCoordinateModeRadioToggled(id);
        });
    
    hbox->addStretch();
    vbox->addLayout(hbox);

    positionWidget = new PositionWidget(self);
    positionWidget->setUserInputValuePriorityMode(true);
    positionWidget->setPositionCallback(
        [&](const Position& T){ return applyPositionInput(T); });
    vbox->addWidget(positionWidget);

    auto grid = new QGridLayout;
    int row = 0;
    grid->setColumnStretch(1, 1);

    frameComboLabel[BaseFrame].setText(_("Base"));
    frameLabelFunction[BaseFrame] =
        [&](LinkKinematicsKit* kit, CoordinateFrame* frame, bool isDefaultFrame){
            return getFrameLabel(frame, isDefaultFrame, _("Body Origin")); };

    frameComboLabel[OffsetFrame].setText(_("Offset"));
    frameLabelFunction[OffsetFrame] =
        [&](LinkKinematicsKit* kit, CoordinateFrame* frame, bool isDefaultFrame){
            return getFrameLabel(frame, isDefaultFrame, _("Link Origin")); };

    for(int i=0; i < 2; ++i){
        grid->addWidget(&frameComboLabel[i], row + i, 0, Qt::AlignLeft /* Qt::AlignJustify */);
        
        frameCombo[i].sigAboutToShowPopup().connect(
            [=](){ updateCoordinateFrameCandidates(i); });
        
        frameCombo[i].sigActivated().connect(
            [=](int index){ onFrameComboActivated(i, index); });
        
        grid->addWidget(&frameCombo[i], row + i, 1, 1, 2);
    }
    row += 2;

    auto configTitle = new QLabel(_("Config"));
    grid->addWidget(configTitle, row, 0, Qt::AlignLeft);
    configurationWidgets.push_back(configTitle);
    grid->addWidget(&configurationLabel, row, 1,  Qt::AlignLeft);
    configurationWidgets.push_back(&configurationLabel);
    configurationButton.setText(_("Set"));
    userInputConnections.add(
        configurationButton.sigClicked().connect(
            [this](){ showConfigurationDialog(); }));
    grid->addWidget(&configurationButton, row, 2);
    configurationWidgets.push_back(&configurationButton);
    configurationDialog = nullptr;
    vbox->addLayout(grid);

    vbox->addStretch();
}


void LinkPositionWidget::customizeCoordinateModeLabels
(const char* worldModeLabel, const char* baseModeLabel, const char* localModeLabel)
{
    impl->worldCoordRadio.setText(worldModeLabel);
    impl->bodyCoordRadio.setText(baseModeLabel);
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


void LinkPositionWidget::setOptionMenuTo(MenuManager& menuManager)
{
    impl->positionWidget->setOptionMenuTo(menuManager);

    menuManager.addSeparator();

    auto disableCustomIkCheck = menuManager.addCheckItem(_("Disable custom IK"));
    disableCustomIkCheck->setChecked(!isCustomIkEnabled());
    disableCustomIkCheck->sigToggled().connect(
        [&](bool on){ setCustomIkEnabled(on); });
}


void LinkPositionWidget::Impl::setCoordinateModeInterfaceEnabled(bool on)
{
    for(auto& widget : coordinateModeWidgets){
        widget->setEnabled(on);
    }
    localCoordRadio.setEnabled(false);
}


void LinkPositionWidget::Impl::setCoordinateMode(int mode, bool doUpdateDisplay)
{
    coordinateModeGroup.blockSignals(true);
    
    if(mode == WorldCoordinateMode){
        worldCoordRadio.setEnabled(true);
        worldCoordRadio.setChecked(true);

    } else if(mode == BodyCoordinateMode){
        bodyCoordRadio.setEnabled(true);
        bodyCoordRadio.setChecked(true);

    } else if(mode == LocalCoordinateMode){
        localCoordRadio.setEnabled(true);
        localCoordRadio.setChecked(true);
    }

    coordinateModeGroup.blockSignals(false);

    if(mode != coordinateMode){
        coordinateMode = mode;
        updateCoordinateFrameCandidates();
    }

    if(doUpdateDisplay){
        updateDisplay();
    }
}


void LinkPositionWidget::Impl::setBodyCoordinateModeEnabled(bool on)
{
    bodyCoordRadio.setEnabled(on);
    if(!on && coordinateMode == BodyCoordinateMode){
        setCoordinateMode(WorldCoordinateMode, false);
    }
}


void LinkPositionWidget::Impl::onCoordinateModeRadioToggled(int mode)
{
    setCoordinateMode(mode, true);
    preferredCoordinateMode = mode;
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
        if(link->isBodyRoot() && bodyItem->isAttachedToParentBody()){
            if(auto parentBodyItem = bodyItem->parentBodyItem()){
                link = bodyItem->body()->parentBodyLink();
                bodyItem = parentBodyItem;
            }
        }
        bool isIkLinkRequired = (targetLinkType != AnyLink);
        if(targetLinkType == RootOrIkLink){
            isIkLinkRequired = !link->isBodyRoot();
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

    if(!link){
        return;
    }

    if(bodyItem != targetBodyItem){
        positionWidget->clearPosition();
        targetConnections.disconnect();
            
        targetBodyItem = bodyItem;
    
        if(bodyItem){
            targetConnections.add(
                bodyItem->sigNameChanged().connect(
                    [&](const std::string&){ updateTargetLink(targetLink); }));

            targetConnections.add(
                bodyItem->sigKinematicStateChanged().connect(
                    [&](){ updateDisplayWithCurrentLinkPosition(); }));
        }
    }

    if(link != targetLink){
        updateTargetLink(link);
        updateDisplayWithCurrentLinkPosition();
    }
}


void LinkPositionWidget::Impl::updateTargetLink(Link* link)
{
    setCoordinateModeInterfaceEnabled(true);
    
    targetLink = link;
    kinematicsKit.reset();
    kinematicsKitConnections.disconnect();
    bool hasBaseFrames = false;
    bool hasOffsetFrames = false;
    bool isBodyCoordinateModeEnabled = false;
    
    if(targetLink){

        auto body = targetBodyItem->body();

        if(defaultCoordName[BaseFrame].empty()){
            defaultCoordName[BaseFrame] = _("Origin");
        }
        if(defaultCoordName[OffsetFrame].empty()){
            defaultCoordName[OffsetFrame] = _("No Offset");
        }

        kinematicsKit = targetBodyItem->getCurrentLinkKinematicsKit(targetLink);
        if(kinematicsKit){
            kinematicsKitConnections.add(
                kinematicsKit->sigFrameUpdate().connect(
                    [&](){ onFrameUpdate(); }));
            kinematicsKitConnections.add(
                kinematicsKit->sigPositionError().connect(
                    [&](const Position& T_frameCoordinate){
                        onKinematicsKitPositionError(T_frameCoordinate); }));

            hasBaseFrames = kinematicsKit->baseFrames();
            hasOffsetFrames = kinematicsKit->offsetFrames();

            if(kinematicsKit->baseLink() && link != kinematicsKit->baseLink() && hasBaseFrames){
                isBodyCoordinateModeEnabled = true;
            }

            baseFrame = kinematicsKit->currentBaseFrame();
            offsetFrame = kinematicsKit->currentOffsetFrame();
        }
    }

    self->setEnabled(kinematicsKit != nullptr);
    setCoordinateFrameInterfaceEnabled(hasBaseFrames, hasOffsetFrames);
    resultLabel.setText("");

    updateCoordinateFrameCandidates();
    setCoordinateMode(preferredCoordinateMode, false);
    setBodyCoordinateModeEnabled(isBodyCoordinateModeEnabled);

    initializeConfigurationInterface();
}


void LinkPositionWidget::Impl::setCoordinateFrameInterfaceEnabled(bool isBaseEnabled, bool isOffsetEnabled)
{
    bool on[] = { isBaseEnabled, isOffsetEnabled };
    for(int i=0; i < 2; ++i){
        frameComboLabel[i].setEnabled(on[i]);
        frameCombo[i].setEnabled(on[i]);
        if(!on[i]){
            frameCombo[i].clear();
        }
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
    updateCoordinateFrameComboItems(
        frameCombo[frameComboIndex], frames, currentFrameId, frameLabelFunction[frameComboIndex]);
}


void LinkPositionWidget::Impl::updateCoordinateFrameComboItems
(QComboBox& combo, CoordinateFrameList* frames, const GeneralId& currentId,
 const FrameLabelFunction& frameLabelFunction)
{
    combo.clear();

    int currentIndex = 0;

    if(frames){
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
    }

    combo.setCurrentIndex(currentIndex);
}


string LinkPositionWidget::Impl::getFrameLabel
(CoordinateFrame* frame, bool isDefaultFrame, const char* defaultFrameNote)
{
    string label;
    string note = frame->note();
    if(note.empty() && isDefaultFrame){
        note = defaultFrameNote;
    }
    if(note.empty()){
        label = frame->id().label();
    } else {
        label = format("{0}: {1}", frame->id().label(), note.c_str());
    }
    return label;
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
        if(kinematicsKit){
            if(frameComboIndex == BaseFrame){
                kinematicsKit->setCurrentBaseFrame(id);
                baseFrame = kinematicsKit->currentBaseFrame();
            } else {
                kinematicsKit->setCurrentOffsetFrame(id);
                offsetFrame = kinematicsKit->currentOffsetFrame();
            }
            kinematicsKit->notifyFrameUpdate();
        }
        updateDisplay();
    }
}


void LinkPositionWidget::Impl::onFrameUpdate()
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

    baseFrame = kinematicsKit->currentBaseFrame();
    offsetFrame = kinematicsKit->currentOffsetFrame();
    
    updateDisplay();
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
    feasibleCheck.sigToggled().connect([&](bool){ updateItemDisplay(); });
    hbox->addWidget(&feasibleCheck);
    vbox->addLayout(hbox);

    auto header = treeWidget.header();
    header->setMinimumSectionSize(1);
    header->setSectionResizeMode(QHeaderView::ResizeToContents);
    header->setSectionsClickable(true);
    QObject::connect(header, &QHeaderView::sectionClicked,
                     [&](int index){ onSectionClicked(index); });

    treeWidget.setSizeAdjustPolicy(QAbstractScrollArea::AdjustToContentsOnFirstShow);
    treeWidget.setRootIsDecorated(false);

    treeWidget.sigCurrentItemChanged().connect(
        [&](QTreeWidgetItem* item, QTreeWidgetItem*){
            if(item){
                int id = item->data(0, Qt::UserRole).toInt();
                applyConfiguration(id);
            }
        });
    vbox->addWidget(&treeWidget);

    hbox = new QHBoxLayout;
    auto updateButton = new PushButton(_("&Update"));
    updateButton->sigClicked().connect([&](){ updateConfigurationStates(); });
    hbox->addWidget(updateButton);
    hbox->addStretch();

    auto applyButton = new PushButton(_("&Apply"));
    applyButton->setDefault(true);
    auto cancelButton = new PushButton(_("&Cancel"));
    auto buttonBox = new QDialogButtonBox(this);
    buttonBox->addButton(applyButton, QDialogButtonBox::AcceptRole);
    buttonBox->addButton(cancelButton, QDialogButtonBox::RejectRole);
    connect(buttonBox, SIGNAL(accepted()), this, SLOT(accept()));
    connect(buttonBox, &QDialogButtonBox::rejected, [&](){ onCanceled(); });
    hbox->addWidget(buttonBox);

    vbox->addLayout(hbox);
    
    setLayout(vbox);
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
    /*
    auto index1 = treeWidget.indexFromItem(treeWidget.topLevelItem(0), 0);
    auto index2 = treeWidget.indexFromItem(treeWidget.topLevelItem(n - 1), treeWidget.columnCount() - 1);
    treeWidget.model()->dataChanged(index1, index2);
    */
    // treeWidget.update();
    // treeWidget.repaint();
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
    baseImpl->findBodyIkSolution(baseImpl->kinematicsKit->link()->T(), true);
    configuration->resetPreferredConfigurationType();
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


void LinkPositionWidget::Impl::onKinematicsKitPositionError(const Position& T_frameCoordinate)
{
    Position T_base;
    if(coordinateMode == WorldCoordinateMode){
        T_base.setIdentity();
    } else {
        T_base = kinematicsKit->globalBasePosition();
    }
    const auto& T_end = kinematicsKit->currentOffsetFrame()->T();
    Position Ta_global = T_base * T_frameCoordinate * T_end.inverse(Eigen::Isometry);
    updateDisplayWithGlobalLinkPosition(Ta_global);
    positionWidget->setErrorHighlight(true);
    resultLabel.setText(_("Not Solved"));
    resultLabel.setStyleSheet(errorStyle);
}


void LinkPositionWidget::Impl::updateDisplay()
{
    userInputConnections.block();
    
    updateDisplayWithCurrentLinkPosition();

    userInputConnections.unblock();

    resultLabel.setText(_("Actual State"));
    resultLabel.setStyleSheet(normalStyle);
}


void LinkPositionWidget::Impl::updateDisplayWithGlobalLinkPosition(const Position& Ta_global)
{
    Position T;
    if(coordinateMode == WorldCoordinateMode){
        T = Ta_global * offsetFrame->T();
    } else if(coordinateMode == BodyCoordinateMode){
        T = baseFrame->T().inverse(Eigen::Isometry) * Ta_global * offsetFrame->T();
        if(kinematicsKit && kinematicsKit->baseLink()){
            T = kinematicsKit->baseLink()->T().inverse(Eigen::Isometry) * T;
        }
    }
    if(kinematicsKit){
        positionWidget->setReferenceRpy(kinematicsKit->referenceRpy());
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


bool LinkPositionWidget::Impl::applyPositionInput(const Position& T)
{
    return findBodyIkSolution(T, false);
}


bool LinkPositionWidget::Impl::findBodyIkSolution(const Position& T_input, bool isRawT)
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

    targetBodyItem->beginKinematicStateEdit();

    if(isRawT){
        solved = ik->calcInverseKinematics(T_input);
    } else {
        Position T;
        if(coordinateMode == WorldCoordinateMode){
            T = T_input * offsetFrame->T().inverse(Eigen::Isometry);
        } else if(coordinateMode == BodyCoordinateMode){
            T = baseFrame->T() * T_input * offsetFrame->T().inverse(Eigen::Isometry);
            if(kinematicsKit->baseLink()){
                T = kinematicsKit->baseLink()->T() * T;
            }
        }
        solved = ik->calcInverseKinematics(T);
    }

    if(solved){
        ik->calcRemainingPartForwardKinematicsForInverseKinematics();
        targetBodyItem->notifyKinematicStateChange();
        targetBodyItem->acceptKinematicStateEdit();
        resultLabel.setText(_("Solved"));
        resultLabel.setStyleSheet(normalStyle);
    } else {
        targetBodyItem->cancelKinematicStateEdit();
        resultLabel.setText(_("Not Solved"));
        resultLabel.setStyleSheet(errorStyle);
    }

    return solved;
}


bool LinkPositionWidget::storeState(Archive& archive)
{
    return impl->storeState(archive);
}


bool LinkPositionWidget::Impl::storeState(Archive& archive)
{
    coordinateModeSelection.select(coordinateMode);
    archive.write("coordinate_mode", coordinateModeSelection.selectedSymbol());
    coordinateModeSelection.select(preferredCoordinateMode);
    archive.write("preferred_coordinate_mode", coordinateModeSelection.selectedSymbol());

    positionWidget->storeState(archive);

    return true;
}


bool LinkPositionWidget::restoreState(const Archive& archive)
{
    return impl->restoreState(archive);
}


bool LinkPositionWidget::Impl::restoreState(const Archive& archive)
{
    userInputConnections.block();
    
    string symbol;

    if(archive.read("preferred_coordinate_mode", symbol)){
        if(coordinateModeSelection.select(symbol)){
            preferredCoordinateMode = coordinateModeSelection.which();
        }
    }
    if(archive.read("coordinate_mode", symbol)){
        if(coordinateModeSelection.select(symbol)){
            setCoordinateMode(coordinateModeSelection.which(), true);
        }
    }

    positionWidget->restoreState(archive);
    
    userInputConnections.unblock();

    return true;
}
