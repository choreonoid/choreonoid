/** \file
    \author Shin'ichiro Nakaoka
*/

#include "LinkPositionView.h"
#include "BodySelectionManager.h"
#include <cnoid/PositionWidget>
#include <cnoid/BodyItem>
#include <cnoid/Body>
#include <cnoid/Link>
#include <cnoid/JointPath>
#include <cnoid/JointPathConfigurationHandler>
#include <cnoid/CompositeBodyIK>
#include <cnoid/LinkCoordinateFrameSet>
#include <cnoid/LinkKinematicsKit>
#include <cnoid/EigenUtil>
#include <cnoid/ConnectionSet>
#include <cnoid/ViewManager>
#include <cnoid/MenuManager>
#include <cnoid/PositionEditManager>
#include <cnoid/Archive>
#include <cnoid/Buttons>
#include <cnoid/CheckBox>
#include <cnoid/ComboBox>
#include <cnoid/ButtonGroup>
#include <cnoid/ActionGroup>
#include <cnoid/Selection>
#include <QLabel>
#include <QGridLayout>
#include <QStyle>
#include <fmt/format.h>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

namespace {

const char* normalStyle = "font-weight: normal";
const char* errorStyle = "font-weight: bold; color: red";

enum FrameType {
    WorldFrame = LinkKinematicsKit::WorldFrame,
    BodyFrame = LinkKinematicsKit::BodyFrame,
    EndFrame = LinkKinematicsKit::EndFrame
};

enum FrameComboType { BaseFrameCombo, EndFrameCombo };

}

namespace cnoid {

class LinkPositionView::Impl
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    LinkPositionView* self;

    ScopedConnectionSet managerConnections;
    ScopedConnectionSet targetConnections;
    enum TargetType { LinkTarget, PositionEditTarget } targetType;
    BodyItemPtr targetBodyItem;
    LinkPtr targetLink;
    enum TargetLinkType { AnyLink, RootOrIkLink, IkLink, NumTargetLinkTypes };
    Selection targetLinkTypeSelection;
    LinkKinematicsKitPtr kinematicsKit;
    LinkKinematicsKitPtr dummyKinematicsKit;
    ScopedConnection kinematicsKitConnection;
    bool isCustomIkDisabled;
    CoordinateFramePtr identityFrame;
    CoordinateFramePtr baseFrame;
    CoordinateFramePtr endFrame;
    std::function<std::tuple<std::string,std::string,std::string>(LinkKinematicsKit*)> functionToGetDefaultFrameNames;
    AbstractPositionEditTarget* positionEditTarget;
    
    QLabel targetLabel;
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

    string defaultCoordName[3];
    QLabel frameComboLabel[2];
    ComboBox frameCombo[2];
    
    QLabel configurationLabel;
    ComboBox configurationCombo;
    CheckBox requireConfigurationCheck;
    vector<QWidget*> configurationWidgets;

    ScopedConnectionSet userInputConnections;

    Impl(LinkPositionView* self);
    void createPanel();
    void onActivated();
    void onAttachedMenuRequest(MenuManager& menuManager);
    void setCoordinateModeInterfaceEnabled(bool on);
    void setCoordinateMode(int mode);
    void setBodyCoordinateModeEnabled(bool on);
    void onCoordinateModeRadioToggled(int mode);
    void setTargetLinkType(int type);
    void setTargetBodyAndLink(BodyItem* bodyItem, Link* link);
    void updateTargetLink(Link* link);
    void updateIkMode();
    void setCoordinateFrameInterfaceEnabled(bool on);
    void updateCoordinateFrameCandidates();
    void updateCoordinateFrameCandidates(int frameComboIndex);
    void updateCoordinateFrameComboItems(
        QComboBox& combo, CoordinateFrameSet* frames, const GeneralId& currentId, const std::string& originLabel,
         bool isBaseLinkBodyFrame);
    void onFrameComboActivated(int frameComboIndex, int index);
    void onCurrentFrameChanged();
    void setConfigurationInterfaceEnabled(bool on);
    void updateConfigurationCandidates();
    bool setPositionEditTarget(AbstractPositionEditTarget* target);
    void onPositionEditTargetExpired();
    void updatePanel();
    void updatePanelWithCurrentLinkPosition();
    void updatePanelWithPositionEditTarget();
    void updateConfigurationPanel();
    void setFramesToCombo(CoordinateFrameSet* frames, QComboBox& combo);
    void onConfigurationInput(int index);
    bool applyPositionInput(const Position& T);
    bool findBodyIkSolution(const Position& T_input);
    bool applyInputToPositionEditTarget(const Position& T_input);
    bool storeState(Archive& archive);
    bool restoreState(const Archive& archive);
};

}


void LinkPositionView::initializeClass(ExtensionManager* ext)
{
    ext->viewManager().registerClass<LinkPositionView>(
        "LinkPositionView", N_("Link Position"), ViewManager::SINGLE_OPTIONAL);
}


LinkPositionView* LinkPositionView::instance()
{
    static LinkPositionView* instance_ = ViewManager::getOrCreateView<LinkPositionView>();
    return instance_;
}


LinkPositionView::LinkPositionView()
{
    impl = new Impl(this);
}


LinkPositionView::Impl::Impl(LinkPositionView* self)
    : self(self),
      targetLinkTypeSelection(NumTargetLinkTypes, CNOID_GETTEXT_DOMAIN_NAME),
      coordinateModeSelection(NumCoordinateModes, CNOID_GETTEXT_DOMAIN_NAME)
{
    self->setDefaultLayoutArea(View::CENTER);
    createPanel();
    self->setEnabled(false);
    
    targetType = LinkTarget;

    targetLinkTypeSelection.setSymbol(AnyLink, "anyLink");    
    targetLinkTypeSelection.setSymbol(RootOrIkLink, "rootOrIkLink");    
    targetLinkTypeSelection.setSymbol(IkLink, "ikLink");    
    targetLinkTypeSelection.select(RootOrIkLink);
    
    dummyKinematicsKit = new LinkKinematicsKit(nullptr);
    dummyKinematicsKit->setFrameSets(new LinkCoordinateFrameSet);
    kinematicsKit = dummyKinematicsKit;

    identityFrame = new CoordinateFrame;
    baseFrame = identityFrame;
    endFrame = identityFrame;

    positionEditTarget = nullptr;
}


LinkPositionView::~LinkPositionView()
{
    delete impl;
}


void LinkPositionView::Impl::createPanel()
{
    self->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);

    auto vbox = new QVBoxLayout;

    auto hbox = new QHBoxLayout;
    hbox->addStretch(2);
    targetLabel.setStyleSheet("font-weight: bold");
    targetLabel.setAlignment(Qt::AlignLeft);
    hbox->addWidget(&targetLabel);
    hbox->addStretch(10);
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout;
    resultLabel.setFrameStyle(QFrame::Box | QFrame::Sunken);
    resultLabel.setAlignment(Qt::AlignCenter);
    hbox->addWidget(&resultLabel, 1);
    auto actualButton = new PushButton(_("Fetch"));
    actualButton->sigClicked().connect([&](){ updatePanel(); });
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

    coordinateModeSelection.setSymbol(BodyCoordinateMode, "body");
    bodyCoordRadio.setText(_("Body"));
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
    positionWidget->setPositionCallback(
        [&](const Position& T){ return applyPositionInput(T); });
    vbox->addWidget(positionWidget);

    isCustomIkDisabled = false;

    auto grid = new QGridLayout;
    int row = 0;
    grid->setColumnStretch(1, 1);

    frameComboLabel[BaseFrameCombo].setText(_("Base"));
    frameComboLabel[EndFrameCombo].setText(_("End"));

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
    grid->addWidget(&configurationLabel, row, 1, 1, 2, Qt::AlignLeft);
    configurationWidgets.push_back(&configurationLabel);
    row += 1;

    auto configComboTitle = new QLabel(_("Pref."));
    grid->addWidget(configComboTitle, row, 0, Qt::AlignLeft);
    configurationWidgets.push_back(configComboTitle);
    grid->addWidget(&configurationCombo, row, 1);
    configurationWidgets.push_back(&configurationCombo);
    userInputConnections.add(
        configurationCombo.sigActivated().connect(
            [this](int index){ onConfigurationInput(index); }));

    requireConfigurationCheck.setText(_("Req."));
    grid->addWidget(&requireConfigurationCheck, row, 2);
    configurationWidgets.push_back(&requireConfigurationCheck);
    userInputConnections.add(
        requireConfigurationCheck.sigToggled().connect(
            [this](bool){ onConfigurationInput(configurationCombo.currentIndex()); }));

    vbox->addLayout(grid);
    vbox->addStretch();
    self->setLayout(vbox, 0.5);
}


void LinkPositionView::setCoordinateModeLabels
(const char* worldModeLabel, const char* baseModeLabel, const char* localModeLabel)
{
    impl->worldCoordRadio.setText(worldModeLabel);
    impl->bodyCoordRadio.setText(baseModeLabel);
    impl->localCoordRadio.setText(localModeLabel);
}


void LinkPositionView::setCoordinateOffsetLabels(const char* baseOffsetLabel, const char* endOffsetLabel)
{
    impl->frameComboLabel[BaseFrameCombo].setText(baseOffsetLabel);
    impl->frameComboLabel[EndFrameCombo].setText(endOffsetLabel);
}


void LinkPositionView::customizeDefaultCoordinateFrameNames
(std::function<std::tuple<std::string,std::string,std::string>(LinkKinematicsKit*)> getNames)
{
    impl->functionToGetDefaultFrameNames = getNames;
}


void LinkPositionView::onActivated()
{
    impl->onActivated();
}


void LinkPositionView::Impl::onActivated()
{
    auto bsm = BodySelectionManager::instance();
    auto pem = PositionEditManager::instance();
    
    managerConnections.add(
        bsm->sigCurrentSpecified().connect(
            [&](BodyItem* bodyItem, Link* link){
                setTargetBodyAndLink(bodyItem, link); }));

    managerConnections.add(
        pem->sigPositionEditRequest().connect(
            [&](AbstractPositionEditTarget* target){
                return setPositionEditTarget(target); }));
                
    setTargetBodyAndLink(bsm->currentBodyItem(), bsm->currentLink());

    if(!targetBodyItem){
        if(auto positionEditTarget = pem->lastPositionEditTarget()){
            setPositionEditTarget(positionEditTarget);
        }
    }
}


void LinkPositionView::onDeactivated()
{
    impl->managerConnections.disconnect();
}


void LinkPositionView::onAttachedMenuRequest(MenuManager& menuManager)
{
    impl->onAttachedMenuRequest(menuManager);
}


void LinkPositionView::Impl::onAttachedMenuRequest(MenuManager& menu)
{
    menu.setPath("/").setPath(_("Target link type"));
    auto checkGroup = new ActionGroup(menu.topMenu());
    menu.addRadioItem(checkGroup, _("Any links"));
    menu.addRadioItem(checkGroup, _("IK priority link and root link"));
    menu.addRadioItem(checkGroup, _("IK priority link"));
    checkGroup->actions()[targetLinkTypeSelection.which()]->setChecked(true);
    checkGroup->sigTriggered().connect(
        [=](QAction* check){ setTargetLinkType(checkGroup->actions().indexOf(check)); });
                                           
    menu.setPath("/");
    menu.addSeparator();
    
    positionWidget->setOptionMenu(menu);

    menu.addSeparator();

    auto disableCustomIkCheck = menu.addCheckItem(_("Disable custom IK"));
    disableCustomIkCheck->setChecked(isCustomIkDisabled);
    disableCustomIkCheck->sigToggled().connect(
        [&](bool on){
            isCustomIkDisabled = on;
            updateIkMode();
            updatePanel();
        });
}


void LinkPositionView::Impl::setCoordinateModeInterfaceEnabled(bool on)
{
    for(auto& widget : coordinateModeWidgets){
        widget->setEnabled(on);
    }
    localCoordRadio.setEnabled(false);
}


void LinkPositionView::Impl::setCoordinateMode(int mode)
{
    coordinateModeGroup.blockSignals(true);
    
    if(mode == WorldCoordinateMode){
        worldCoordRadio.setEnabled(true);
        worldCoordRadio.setChecked(true);
        kinematicsKit->setCurrentBaseFrameType(LinkKinematicsKit::WorldFrame);
    } else if(mode == BodyCoordinateMode){
        bodyCoordRadio.setEnabled(true);
        bodyCoordRadio.setChecked(true);
        kinematicsKit->setCurrentBaseFrameType(LinkKinematicsKit::BodyFrame);
    } else if(mode == LocalCoordinateMode){
        localCoordRadio.setEnabled(true);
        localCoordRadio.setChecked(true);
    }

    coordinateModeGroup.blockSignals(false);

    if(mode != coordinateMode){
        coordinateMode = mode;
        updateCoordinateFrameCandidates();
    }

    updatePanel();
}


void LinkPositionView::Impl::setBodyCoordinateModeEnabled(bool on)
{
    bodyCoordRadio.setEnabled(on);
    if(!on && coordinateMode == BodyCoordinateMode){
        setCoordinateMode(WorldCoordinateMode);
    }
}


void LinkPositionView::Impl::onCoordinateModeRadioToggled(int mode)
{
    setCoordinateMode(mode);
    preferredCoordinateMode = mode;
}


void LinkPositionView::Impl::setTargetLinkType(int type)
{
    targetLinkTypeSelection.select(type);
    setTargetBodyAndLink(targetBodyItem, targetLink);
}


void LinkPositionView::Impl::setTargetBodyAndLink(BodyItem* bodyItem, Link* link)
{
    // Sub body's root link is recognized as the parent body's end link
    if(bodyItem && link){
        if(link->hasParentBody()){
            if(auto parentBodyItem = bodyItem->parentBodyItem()){
                link = bodyItem->body()->parentBodyLink();
                bodyItem = parentBodyItem;
            }
        }
        bool isIkLinkRequired = !targetLinkTypeSelection.is(AnyLink);
        if(targetLinkTypeSelection.is(RootOrIkLink)){
            isIkLinkRequired = !link->isRoot();
        }
        if(isIkLinkRequired){
            if(!bodyItem->getDefaultIK(link)){
                LinkTraverse traverse(link);
                link = nullptr;
                for(int i=1; i < traverse.numLinks(); ++i){
                    if(bodyItem->getDefaultIK(traverse[i])){
                        link = traverse[i];
                        break;
                    }
                }
            }
        }
    }
                
    bool isTargetTypeChanged = (targetType != LinkTarget);
    bool isBodyItemChanged = isTargetTypeChanged || (bodyItem != targetBodyItem);
    bool isLinkChanged = isTargetTypeChanged || (link != targetLink);
    
    if(isBodyItemChanged || isLinkChanged){

        if(isBodyItemChanged){
            positionWidget->clearPosition();
            targetConnections.disconnect();
            
            targetBodyItem = bodyItem;
    
            if(bodyItem){
                targetConnections.add(
                    bodyItem->sigNameChanged().connect(
                        [&](const std::string&){ updateTargetLink(targetLink); }));

                targetConnections.add(
                    bodyItem->sigKinematicStateChanged().connect(
                        [&](){ updatePanelWithCurrentLinkPosition(); }));
            }
        }

        targetType = LinkTarget;
        updateTargetLink(link);
        updatePanelWithCurrentLinkPosition();
    }
}


void LinkPositionView::Impl::updateTargetLink(Link* link)
{
    setCoordinateModeInterfaceEnabled(true);
    
    if(targetType != LinkTarget){
        return;
    }
    
    targetLink = link;
    
    if(!targetLink){
        kinematicsKit = dummyKinematicsKit;
        kinematicsKitConnection.reset();
        targetLabel.setText("------");
        self->setEnabled(false);

    } else {
        auto body = targetBodyItem->body();

        targetLabel.setText(format("{0} / {1}", body->name(), targetLink->name()).c_str());

        kinematicsKit = targetBodyItem->getLinkKinematicsKit(targetLink);
        kinematicsKitConnection =
            kinematicsKit->sigCurrentFrameChanged().connect(
                [&](){ onCurrentFrameChanged(); });

        if(functionToGetDefaultFrameNames){
            tie(defaultCoordName[WorldFrame], defaultCoordName[BodyFrame], defaultCoordName[EndFrame]) =
                functionToGetDefaultFrameNames(kinematicsKit);
        }
        if(defaultCoordName[WorldFrame].empty()){
            defaultCoordName[WorldFrame] = _("World Origin");
        }
        for(int i=0; i < 2; ++i){
            if(defaultCoordName[i].empty()){
                defaultCoordName[i] = _("Origin");
            }
        }
        
        updateIkMode();

        if(coordinateMode == WorldCoordinateMode){
            kinematicsKit->setCurrentBaseFrameType(LinkKinematicsKit::WorldFrame);
        } else {
            kinematicsKit->setCurrentBaseFrameType(LinkKinematicsKit::BodyFrame);
        }
        baseFrame = kinematicsKit->currentBaseFrame();
        endFrame = kinematicsKit->currentEndFrame();
    }

    bool isValid = kinematicsKit->inverseKinematics() != nullptr;
    self->setEnabled(isValid);
    setCoordinateFrameInterfaceEnabled(isValid);
    resultLabel.setText("");

    updateCoordinateFrameCandidates();
    updateConfigurationCandidates();

    setCoordinateMode(preferredCoordinateMode);

    setBodyCoordinateModeEnabled(
        !(!kinematicsKit->baseLink() || link == kinematicsKit->baseLink()));
}


void LinkPositionView::Impl::updateIkMode()
{
    if(auto jointPath = kinematicsKit->jointPath()){
        jointPath->setNumericalIKenabled(isCustomIkDisabled);
    }
}


void LinkPositionView::Impl::setCoordinateFrameInterfaceEnabled(bool on)
{
    for(int i=0; i < 2; ++i){
        frameComboLabel[i].setEnabled(on);
        frameCombo[i].setEnabled(on);
        if(!on){
            frameCombo[i].clear();
        }
    }
}
    

void LinkPositionView::Impl::updateCoordinateFrameCandidates()
{
    for(int i=0; i < 2; ++i){
        updateCoordinateFrameCandidates(i);
    }
}


void LinkPositionView::Impl::updateCoordinateFrameCandidates(int frameComboIndex)
{
    int frameType;
    bool isBaseLinkBodyFrame = false;

    if(frameComboIndex == EndFrameCombo){
        frameType = EndFrame;
    } else {
        if(kinematicsKit->baseLink() == targetLink){
            isBaseLinkBodyFrame = true;
        }
        if(coordinateMode == WorldCoordinateMode){
            frameType = WorldFrame;
        } else {
            frameType = BodyFrame;
        }
    }
    
    updateCoordinateFrameComboItems(
        frameCombo[frameComboIndex],
        kinematicsKit->frameSet(frameType),
        kinematicsKit->currentFrameId(frameType),
        defaultCoordName[frameType],
        isBaseLinkBodyFrame);
}


void LinkPositionView::Impl::updateCoordinateFrameComboItems
(QComboBox& combo, CoordinateFrameSet* frames, const GeneralId& currentId, const std::string& originLabel,
 bool isBaseLinkBodyFrame)
{
    constexpr bool EnableToolTip = false;
    
    combo.clear();
    combo.addItem(QString("0: %1").arg(originLabel.c_str()), 0);
    if(EnableToolTip){
        combo.setItemData(0, QString(), Qt::ToolTipRole);
    }
    int currentIndex = 0;

    if(frames){
        auto candidates = frames->getFindableFrameLists();
        const int n = candidates.size();
        for(int i=0; i < n; ++i){
            int index = combo.count();
            auto frame = candidates[i];
            auto& id = frame->id();
            if(id.isInt()){
                combo.addItem(QString("%1: %2").arg(id.toInt()).arg(frame->note().c_str()), id.toInt());
                if(EnableToolTip){
                    combo.setItemData(index, QString(), Qt::ToolTipRole);
                }
            } else {
                combo.addItem(id.label().c_str(), id.toString().c_str());
                if(EnableToolTip){
                    combo.setItemData(index, frame->note().c_str(), Qt::ToolTipRole);
                }
            }
            if(id == currentId){
                currentIndex = index;
            }
        }
    }

    combo.setCurrentIndex(currentIndex);
}


void LinkPositionView::Impl::onFrameComboActivated(int frameComboIndex, int index)
{
    GeneralId id;
    auto idValue = frameCombo[frameComboIndex].itemData(index);
    if(idValue.userType() == QMetaType::Int){
        id = idValue.toInt();
    } else if(idValue.userType() == QMetaType::QString){
        id = idValue.toString().toStdString();
    }
    if(id.isValid()){
        int frameType;
        if(frameComboIndex == BaseFrameCombo){
            if(coordinateMode == WorldCoordinateMode){
                frameType = WorldFrame;
            } else {
                frameType = BodyFrame;
            }
        } else {
            frameType = EndFrame;
        }
        kinematicsKit->setCurrentFrame(frameType, id);
        if(frameComboIndex == BaseFrameCombo){
            baseFrame = kinematicsKit->currentFrame(frameType);
        } else {
            endFrame = kinematicsKit->currentEndFrame();
        }
        updatePanel();
    }
}


void LinkPositionView::Impl::onCurrentFrameChanged()
{
    bool coordinateModeUpdated = false;
    auto baseFrameType = kinematicsKit->currentBaseFrameType();
    
    if(baseFrameType == LinkCoordinateFrameSet::WorldFrame && coordinateMode != WorldCoordinateMode){
        setCoordinateMode(WorldCoordinateMode);
        coordinateModeUpdated = true;

    } else if(baseFrameType == LinkCoordinateFrameSet::BodyFrame && coordinateMode != BodyCoordinateMode){
        setCoordinateMode(BodyCoordinateMode);
        coordinateModeUpdated = true;
    }

    if(coordinateModeUpdated){
        preferredCoordinateMode = coordinateMode;

    } else {
        for(int i=0; i < 2; ++i){
            GeneralId newId;
            if(i == BaseFrameCombo){
                newId = kinematicsKit->currentBaseFrameId();
            } else {
                newId = kinematicsKit->currentEndFrameId();
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
    }
    
    baseFrame = kinematicsKit->currentBaseFrame();
    endFrame = kinematicsKit->currentEndFrame();
    
    updatePanel();
}


void LinkPositionView::Impl::setConfigurationInterfaceEnabled(bool on)
{
    for(auto& widget : configurationWidgets){
        widget->setEnabled(on);
    }
    if(!on){
        configurationLabel.setText("-----");
        configurationCombo.clear();
    }
}
    

void LinkPositionView::Impl::updateConfigurationCandidates()
{
    bool isConfigurationComboActive = false;
    configurationCombo.clear();

    if(auto configurationHandler = kinematicsKit->configurationHandler()){
        int n = configurationHandler->getNumConfigurations();
        for(int i=0; i < n; ++i){
            configurationCombo.addItem(
                configurationHandler->getConfigurationName(i).c_str());
        }
        if(configurationHandler->checkConfiguration(0)){
            configurationCombo.setCurrentIndex(0);
        } else {
            configurationCombo.setCurrentIndex(
                configurationHandler->getCurrentConfiguration());
        }
        isConfigurationComboActive = !isCustomIkDisabled;
    }

    setConfigurationInterfaceEnabled(isConfigurationComboActive);
}


bool LinkPositionView::Impl::setPositionEditTarget(AbstractPositionEditTarget* target)
{
    positionWidget->clearPosition();
    targetConnections.disconnect();

    targetType = PositionEditTarget;
    positionEditTarget = target;
    baseFrame = identityFrame;
    endFrame = identityFrame;

    targetConnections.add(
        target->sigPositionChanged().connect(
            [&](const Position&){ updatePanelWithPositionEditTarget(); }));

    targetConnections.add(
        target->sigPositionEditTargetExpired().connect(
            [&](){ onPositionEditTargetExpired(); }));

    targetLabel.setText(target->getPositionName().c_str());
    self->setEnabled(target->isEditable());
    setCoordinateFrameInterfaceEnabled(false);
    setConfigurationInterfaceEnabled(false);
    setBodyCoordinateModeEnabled(false);
    setCoordinateModeInterfaceEnabled(false);

    updatePanelWithPositionEditTarget();

    return true;
}


void LinkPositionView::Impl::onPositionEditTargetExpired()
{

}


void LinkPositionView::Impl::updatePanel()
{
    userInputConnections.block();
    
    if(targetType == LinkTarget){
        updatePanelWithCurrentLinkPosition();

    } else if(targetType == PositionEditTarget){
        updatePanelWithPositionEditTarget();
    }

    userInputConnections.unblock();

    resultLabel.setText(_("Actual State"));
    resultLabel.setStyleSheet(normalStyle);
}


void LinkPositionView::Impl::updatePanelWithCurrentLinkPosition()
{
    if(targetLink){
        Position T = baseFrame->T().inverse(Eigen::Isometry) * targetLink->Ta() * endFrame->T();
        if(coordinateMode == BodyCoordinateMode && kinematicsKit->baseLink()){
            T = kinematicsKit->baseLink()->Ta().inverse(Eigen::Isometry) * T;
        }
        positionWidget->setReferenceRpy(kinematicsKit->referenceRpy());
        positionWidget->updatePosition(T);

        updateConfigurationPanel();
    }
}


void LinkPositionView::Impl::updatePanelWithPositionEditTarget()
{
    if(positionEditTarget){
        positionWidget->setReferenceRpy(Vector3::Zero());
        positionWidget->updatePosition(positionEditTarget->getPosition());
    }
}


void LinkPositionView::Impl::updateConfigurationPanel()
{
    auto configurationHandler = kinematicsKit->configurationHandler();
    
    if(!configurationHandler){
        configurationLabel.setText("-----");
        
    } else {
        int preferred = configurationCombo.currentIndex();
        if(requireConfigurationCheck.isChecked() &&
           !configurationHandler->checkConfiguration(preferred)){
            configurationCombo.setStyleSheet(errorStyle);
        } else {
            configurationCombo.setStyleSheet("font-weight: normal");
        }
        int actual = configurationHandler->getCurrentConfiguration();
        configurationLabel.setText(configurationCombo.itemText(actual));
    }
}


void LinkPositionView::Impl::onConfigurationInput(int index)
{
    if(auto configurationHandler = kinematicsKit->configurationHandler()){
        configurationHandler->setPreferredConfiguration(index);
    }
    positionWidget->applyPositionInput();
}


bool LinkPositionView::Impl::applyPositionInput(const Position& T)
{
    bool accepted = false;
    
    if(targetType == LinkTarget){
        accepted = findBodyIkSolution(T);

    } else if(targetType == PositionEditTarget){
        accepted = applyInputToPositionEditTarget(T);
    }

    return accepted;
}


bool LinkPositionView::Impl::findBodyIkSolution(const Position& T_input)
{
    bool solved = false;
    
    if(auto ik = kinematicsKit->inverseKinematics()){

        kinematicsKit->setReferenceRpy(positionWidget->getRpyInput());

        targetBodyItem->beginKinematicStateEdit();
        
        Position T = baseFrame->T() * T_input * endFrame->T().inverse(Eigen::Isometry);
        if(coordinateMode == BodyCoordinateMode && kinematicsKit->baseLink()){
            T = kinematicsKit->baseLink()->Ta() * T;
        }
        T.linear() = targetLink->calcRfromAttitude(T.linear());
        
        solved = ik->calcInverseKinematics(T);

        if(solved){
            if(requireConfigurationCheck.isChecked() && !isCustomIkDisabled){
                if(auto configurationHandler = kinematicsKit->configurationHandler()){
                    int preferred = configurationCombo.currentIndex();
                    if(!configurationHandler->checkConfiguration(preferred)){
                        configurationCombo.setStyleSheet(errorStyle);
                        solved = false;
                    }
                }
            }
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
            resultLabel.setStyleSheet("font-weight: bold; color: red");
        }
    }

    return solved;
}


bool LinkPositionView::Impl::applyInputToPositionEditTarget(const Position& T_input)
{
    bool accepted = false;
    
    if(positionEditTarget){

        targetConnections.block();
        accepted = positionEditTarget->setPosition(T_input);
        targetConnections.unblock();

        if(accepted){
            resultLabel.setText(_("Accepted"));
            resultLabel.setStyleSheet(normalStyle);
        } else {
            resultLabel.setText(_("Not Accepted"));
            resultLabel.setStyleSheet("font-weight: bold; color: red");
        }
    }

    return accepted;
}


bool LinkPositionView::storeState(Archive& archive)
{
    return impl->storeState(archive);
}


bool LinkPositionView::Impl::storeState(Archive& archive)
{
    archive.write("targetLinkType", targetLinkTypeSelection.selectedSymbol());
    coordinateModeSelection.select(coordinateMode);
    archive.write("coordinateMode", coordinateModeSelection.selectedSymbol());
    coordinateModeSelection.select(preferredCoordinateMode);
    archive.write("preferredCoordinateMode", coordinateModeSelection.selectedSymbol());
    archive.write("disableCustomIK", isCustomIkDisabled);

    positionWidget->storeState(archive);

    return true;
}


bool LinkPositionView::restoreState(const Archive& archive)
{
    return impl->restoreState(archive);
}


bool LinkPositionView::Impl::restoreState(const Archive& archive)
{
    userInputConnections.block();
    
    string symbol;

    if(archive.read("targetLinkType", symbol)){
        targetLinkTypeSelection.select(symbol);
    }
    if(archive.read("preferredCoordinateMode", symbol)){
        if(coordinateModeSelection.select(symbol)){
            preferredCoordinateMode = coordinateModeSelection.which();
        }
    }
    if(archive.read("coordinateMode", symbol)){
        if(coordinateModeSelection.select(symbol)){
            setCoordinateMode(coordinateModeSelection.which());
        }
    }

    positionWidget->restoreState(archive);
    
    archive.read("disableCustomIK", isCustomIkDisabled);

    userInputConnections.unblock();

    return true;
}
