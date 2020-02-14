/*! @file
  @author Shin'ichiro Nakaoka
*/

#include "PoseSeqViewBase.h"
#include "BodyMotionGenerationBar.h"
#include "PronunSymbol.h"
#include "PoseFilters.h"
#include "BodyMotionGenerationBar.h"
#include <cnoid/RootItem>
#include <cnoid/LeggedBodyHelper>
#include <cnoid/LinkSelectionView>
#include <cnoid/MessageView>
#include <cnoid/InfoBar>
#include <cnoid/Dialog>
#include <fmt/format.h>
#include <QDialogButtonBox>
#include <QHeaderView>
#include <QMouseEvent>
#include <QScrollBar>
#include <QLineEdit>
#include <iostream>
#include "gettext.h"

using namespace std;
using namespace std::placeholders;
using namespace cnoid;
using fmt::format;

namespace {

const bool TRACE_FUNCTIONS = false;

inline double degree(double rad) { return (180.0 * rad / 3.14159265358979); }
inline double radian(double deg) { return (3.14159265358979 * deg / 180.0); }

}

class ColumnCheckBox : public CheckBox
{
public:
    ColumnCheckBox(std::function<void(Qt::CheckState)> slotOnClicked)
        : slotOnClicked(slotOnClicked) {
    }
    virtual void nextCheckState(){
        slotOnClicked(checkState());
    };
    std::function<void(Qt::CheckState)> slotOnClicked;        
};

class LinkTreeWidgetEx : public LinkTreeWidget
{
public:
    LinkTreeWidgetEx(QWidget* parent) : LinkTreeWidget(parent) {
        header()->setSectionResizeMode(nameColumn(), QHeaderView::ResizeToContents);
    }
    virtual QSize sizeHint() const {
        QSize size = QTreeWidget::sizeHint();
        int width = header()->length();
        size.setWidth(width);
        return size;
    }
};

namespace cnoid {

class LinkPositionAdjustmentDialog : public Dialog
{
public:
    RadioButton absoluteRadio;
    RadioButton relativeRadio;
    CheckBox targetAxisCheck[3];
    DoubleSpinBox positionSpin[3];

    LinkPositionAdjustmentDialog(View* parentView)
        : Dialog(parentView) {

        setWindowTitle(_("Link Position Adjustment"));

        QVBoxLayout* vbox = new QVBoxLayout();
        QHBoxLayout* hbox = new QHBoxLayout();

        vbox->addLayout(hbox);

        absoluteRadio.setText(_("Absolute"));
        hbox->addWidget(&absoluteRadio);
        relativeRadio.setText(_("Relative"));
        relativeRadio.setChecked(true);
        hbox->addWidget(&relativeRadio);

        hbox = new QHBoxLayout();
        vbox->addLayout(hbox);

        const char* axisLabel[] = { "X", "Y", "Z" };
            
        for(int i=0; i < 3; ++i){
            targetAxisCheck[i].setText(axisLabel[i]);
            hbox->addWidget(&targetAxisCheck[i]);
            positionSpin[i].setDecimals(3);
            positionSpin[i].setRange(-99.999, 99.999);
            positionSpin[i].setSingleStep(0.001);
            positionSpin[i].setValue(0.0);
            hbox->addWidget(&positionSpin[i]);
        }

        QDialogButtonBox* buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok);
        connect(buttonBox, SIGNAL(accepted()), this, SLOT(accept()));
        vbox->addWidget(buttonBox);

        setLayout(vbox);
    }

    bool storeState(Archive& archive) {
        return true;
    }

    bool restoreState(const Archive& archive) {
        return true;
    }
};


class YawOrientationRotationDialog : public Dialog
{
public:
    DoubleSpinBox angleSpin;
    DoubleSpinBox centerPosSpins[2];

    YawOrientationRotationDialog(View* parentView)
        : Dialog(parentView) {

        setWindowTitle(_("Yaw Orientation Rotation"));

        QVBoxLayout* vbox = new QVBoxLayout();
        QHBoxLayout* hbox = new QHBoxLayout();
        vbox->addLayout(hbox);

        hbox->addWidget(new QLabel(_("Center:")));
        hbox->addSpacing(8);
        const char* axisLabel[] = { "X", "Y" };
        for(int i=0; i < 2; ++i){
            hbox->addWidget(new QLabel(axisLabel[i]));
            centerPosSpins[i].setDecimals(3);
            centerPosSpins[i].setRange(-99.999, 99.999);
            centerPosSpins[i].setSingleStep(0.001);
            hbox->addWidget(&centerPosSpins[i]);
        }

        hbox = new QHBoxLayout();
        vbox->addLayout(hbox);

        hbox->addWidget(new QLabel(_("Angle")));
        angleSpin.setDecimals(1);
        angleSpin.setRange(0.1, 90.0);
        angleSpin.setSingleStep(0.1);
        hbox->addWidget(&angleSpin);
        hbox->addWidget(new QLabel(_("[deg]")));

        QDialogButtonBox* buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok);
        connect(buttonBox, SIGNAL(accepted()), this, SLOT(accept()));
        vbox->addWidget(buttonBox);
            
        setLayout(vbox);
    }

    bool storeState(Archive& archive) {
        return true;
    }

    bool restoreState(const Archive& archive) {
        return true;
    }
};
    

class PoseSelectionDialog : public Dialog
{
public:
    DoubleSpinBox startTimeSpin;
    DoubleSpinBox endTimeSpin;
    RadioButton allPartRadio;
    RadioButton selectedPartRadio;
    RadioButton justSelectedPartRadio;

    PoseSelectionDialog(View* parentView)
        : Dialog(parentView) {

        setWindowTitle(_("Select Specified Key Poses"));

        QVBoxLayout* vbox = new QVBoxLayout();

        QHBoxLayout* hbox = new QHBoxLayout();
        vbox->addLayout(hbox);

        hbox->addWidget(new QLabel(_("Start")));
        startTimeSpin.setDecimals(2);
        startTimeSpin.setRange(0.00, 999.99);
        startTimeSpin.setSingleStep(0.01);
        hbox->addWidget(&startTimeSpin);
        hbox->addWidget(new QLabel(_("[s]")));

        hbox->addWidget(new QLabel(_("End")));
        endTimeSpin.setDecimals(2);
        endTimeSpin.setRange(0.00, 999.99);
        endTimeSpin.setSingleStep(0.01);
        hbox->addWidget(&endTimeSpin);
        hbox->addWidget(new QLabel(_("[s]")));
            
        hbox = new QHBoxLayout();
        vbox->addLayout(hbox);

        allPartRadio.setText(_("all parts"));
        hbox->addWidget(&allPartRadio);
        selectedPartRadio.setText(_("having selected parts"));
        selectedPartRadio.setChecked(true);
        hbox->addWidget(&selectedPartRadio);
        justSelectedPartRadio.setText(_("just selected parts"));
        hbox->addWidget(&justSelectedPartRadio);

        QDialogButtonBox* buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok);
        connect(buttonBox, SIGNAL(accepted()), this, SLOT(accept()));
        vbox->addWidget(buttonBox);
            
        setLayout(vbox);
    }

    bool storeState(Archive& archive) {
        return true;
    }

    bool restoreState(const Archive& archive) {
        return true;
    }
};
}


PoseSeqViewBase::PoseSeqViewBase(View* view)
    : view(view),
      os(MessageView::mainInstance()->cout()),
      textForEmptyName("----------"),
      menuManager(&popupMenu)
{
    view->sigActivated().connect(std::bind(&PoseSeqViewBase::onViewActivated, this));
    view->sigDeactivated().connect(std::bind(&PoseSeqViewBase::onViewDeactivated, this));

    currentTime = 0.0;

    timeBar = TimeBar::instance();

    BodyMotionGenerationBar* generationBar = BodyMotionGenerationBar::instance();
    timeScale = generationBar->timeScaleRatio();
    staticConnections.add(
        generationBar->sigInterpolationParametersChanged().connect(
            std::bind(&PoseSeqViewBase::onInterpolationParametersChanged, this)));

    setupOperationParts();
    setupLinkTreeWidget();

    staticConnections.add(
        RootItem::instance()->sigSelectedItemsChanged().connect(
            std::bind(&PoseSeqViewBase::onSelectedItemsChanged, this, _1)));

    isSelectedPoseMoving = false;

    copiedPoses = new PoseSeq();
    
    poseSelectionDialog = new PoseSelectionDialog(view);
    poseSelectionDialog->sigAccepted().connect(
        std::bind(&PoseSeqViewBase::onPoseSelectionDialogAccepted, this));

    linkPositionAdjustmentDialog = new LinkPositionAdjustmentDialog(view);
    linkPositionAdjustmentDialog->sigAccepted().connect(
        std::bind(&PoseSeqViewBase::onLinkPositionAdjustmentDialogAccepted, this));

    yawOrientationRotationDialog = new YawOrientationRotationDialog(view);
    yawOrientationRotationDialog->sigAccepted().connect(
        std::bind(&PoseSeqViewBase::onYawOrientationRotationDialogAccepted, this));

    menuManager.addItem(_("Select all poses after current position"))->sigTriggered().connect
        (std::bind(&PoseSeqViewBase::selectAllPosesAfterCurrentPosition, this));
    menuManager.addItem(_("Select all poses before current position"))->sigTriggered().connect
        (std::bind(&PoseSeqViewBase::selectAllPosesBeforeCurrentPosition, this));
    menuManager.addItem(_("Adjust step positions"))->sigTriggered().connect
        (std::bind(&PoseSeqViewBase::onAdjustStepPositionsActivated, this));
    menuManager.addItem(_("Count selected key poses"))->sigTriggered().connect(
        std::bind(&PoseSeqViewBase::countSelectedKeyPoses, this));
}
        


PoseSeqViewBase::~PoseSeqViewBase()
{
    staticConnections.disconnect();
    poseSeqConnections.disconnect();
    connectionOfBodyKinematicStateEdited.disconnect();
}


void PoseSeqViewBase::onViewActivated()
{
    if(TRACE_FUNCTIONS){
        cout << "PoseSeqViewBase::onViewActivated()" << endl;
    }
    if(timeSyncCheck.isChecked()){
        if(!connectionOfTimeChanged.connected()){
            connectionOfTimeChanged = timeBar->sigTimeChanged().connect(
                std::bind(&PoseSeqViewBase::onTimeChanged, this, _1));
        }
        onTimeChanged(timeBar->time());
    }
}


void PoseSeqViewBase::onViewDeactivated()
{
    if(TRACE_FUNCTIONS){
        cout << "PoseSeqViewBase::onViewDeactivated()" << endl;
    }
    connectionOfTimeChanged.disconnect();
}


void PoseSeqViewBase::onTimeSyncCheckToggled()
{
    if(timeSyncCheck.isChecked()){
        if(!connectionOfTimeChanged.connected()){
            connectionOfTimeChanged = timeBar->sigTimeChanged().connect(
                std::bind(&PoseSeqViewBase::onTimeChanged, this, _1));
        }
    } else {
        connectionOfTimeChanged.disconnect();
    }
}


void PoseSeqViewBase::setupOperationParts()
{
    currentItemLabel.setText(textForEmptyName);
    currentItemLabel.setAlignment(Qt::AlignCenter);

    insertPoseButton.setText(_(" Insert "));
    insertPoseButton.setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
    insertPoseButton.setToolTip(_("Insert a new pose at the current time position"));
    insertPoseButton.sigClicked().connect(
        std::bind(&PoseSeqViewBase::onInsertPoseButtonClicked, this));

    transitionTimeSpin.setToolTip(_("Transition time of a newly inserted pose"));
    transitionTimeSpin.setAlignment(Qt::AlignCenter);
    transitionTimeSpin.setDecimals(3);
    transitionTimeSpin.setRange(0.0, 9.999);
    transitionTimeSpin.setSingleStep(0.005);
    transitionTimeSpin.sigEditingFinished().connect(
        std::bind(&PoseSeqViewBase::onInsertPoseButtonClicked, this));

    updateButton.setText(_("Update"));
    updateButton.setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
    updateButton.setToolTip(_("Update the selected pose with the current robot state"));
    updateButton.sigClicked().connect(
        std::bind(&PoseSeqViewBase::onUpdateButtonClicked, this));

    updateAllToggle.setText(_("All"));
    updateAllToggle.setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
    updateAllToggle.setToolTip(_("The update button updates all the element of the selected pose."));
    updateAllToggle.setChecked(true);
    
    autoUpdateModeCheck.setText(_("Auto"));
    autoUpdateModeCheck.setToolTip(_("The selected pose is automatically updated when the robot state changes."));
    autoUpdateModeCheck.setChecked(false);

    deleteButton.setText(_("Delete"));
    deleteButton.setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
    deleteButton.sigClicked().connect(
        std::bind(&PoseSeqViewBase::onDeleteButtonClicked, this));
    
    timeSyncCheck.setText(_("Time sync"));
    timeSyncCheck.setChecked(true);
    timeSyncCheck.sigToggled().connect(
        std::bind(&PoseSeqViewBase::onTimeSyncCheckToggled, this));
}

        
void PoseSeqViewBase::setupLinkTreeWidget()
{
    linkTreeWidget = new LinkTreeWidgetEx(view);

    //linkTreeWidget->setStyleSheet("QTreeView::item { border-right: 1px solid black }");
    
    QHeaderView* header = linkTreeWidget->header();
    header->hideSection(linkTreeWidget->jointIdColumn());

    poseForDefaultStateSetting = new Pose();
    
    baseLinkColumn = linkTreeWidget->addColumn("BL");
    linkTreeWidget->moveVisualColumnIndex(baseLinkColumn, 0);
    baseLinkRadioGroup = 0;

    validPartColumn = linkTreeWidget->addColumn("ON");
    stationaryPointColumn = linkTreeWidget->addColumn("SP");
    ikPartColumn = linkTreeWidget->addColumn("IK");
    
    zmpRow = new LinkTreeItem("ZMP");
    linkTreeWidget->addCustomRow(zmpRow);

    linkTreeWidget->sigUpdateRequest().connect(
        std::bind(&PoseSeqViewBase::onLinkTreeUpdateRequest, this, _1));

    linkTreeWidget->setFrameShape(QFrame::NoFrame);
    linkTreeWidget->setDefaultExpansionLevel(1);
    linkTreeWidget->enableCache(true);
    linkTreeWidget->setListingMode(LinkTreeWidget::PART_TREE);
    linkTreeWidget->fixListingMode(true);

    MenuManager& mm = linkTreeWidget->popupMenuManager();
    mm.addItem(_("Select key poses having the selected links"))->sigTriggered().connect(
        std::bind(&PoseSeqViewBase::selectPosesHavingSelectedLinks, this));
    mm.addItem(_("Select key poses just having the selected links"))->sigTriggered().connect(
        std::bind(&PoseSeqViewBase::selectPosesJustHavingSelectedLinks, this));
    mm.addItem(_("Remove the selected parts from the selected poses"))->sigTriggered().connect(
        std::bind(&PoseSeqViewBase::removeSelectedPartsFromKeyPoses, this));
}


bool PoseSeqViewBase::isChecked(LinkTreeItem* item, int column)
{
    QAbstractButton* check = dynamic_cast<QAbstractButton*>(linkTreeWidget->alignedItemWidget(item, column));
    return check ? check->isChecked() : Qt::Unchecked;
}


void PoseSeqViewBase::setChecked(LinkTreeItem* item, int column, bool checked)
{
    QAbstractButton* check = dynamic_cast<QAbstractButton*>(linkTreeWidget->alignedItemWidget(item, column));
    if(check){
        check->setChecked(checked);
    }
}


void PoseSeqViewBase::setCheckState(LinkTreeItem* item, int column, Qt::CheckState state)
{
    QCheckBox* check = dynamic_cast<QCheckBox*>(linkTreeWidget->alignedItemWidget(item, column));
    if(check){
        check->setCheckState(state);
    }
}


void PoseSeqViewBase::onLinkTreeUpdateRequest(bool isInitialCreation)
{
    if(!linkTreeWidget->bodyItem()){
        setCurrentPoseSeqItem(0);
    } else {
        if(isInitialCreation){
            initializeLinkTree();
        }
        updateLinkTreeModel();
    }
}


void PoseSeqViewBase::initializeLinkTree()
{
    poseForDefaultStateSetting->clear();

    if(baseLinkRadioGroup){
        delete baseLinkRadioGroup;
    }
    baseLinkRadioGroup = new ButtonGroup(linkTreeWidget);
    baseLinkRadioGroup->sigButtonClicked().connect(
        std::bind(&PoseSeqViewBase::onBaseLinkRadioClicked, this));

    initializeLinkTreeIkLinkColumn();

    Link* rootLink = body->rootLink();
    poseForDefaultStateSetting->setBaseLink(rootLink->index(), rootLink->p(), rootLink->R());

    initializeLinkTreeTraverse(linkTreeWidget->invisibleRootItem());

}


void PoseSeqViewBase::initializeLinkTreeIkLinkColumn()
{
    const Mapping& info = *body->info();

    possibleIkLinkFlag.clear();
    possibleIkLinkFlag.resize(body->numLinks(), false);

    if(body->numLinks() > 0){
        const Listing& possibleIkLinks = *info.findListing("possibleIkInterpolationLinks");
        if(possibleIkLinks.isValid()){
            for(int i=0; i < possibleIkLinks.size(); ++i){
                Link* link = body->link(possibleIkLinks[i]);
                if(link){
                    possibleIkLinkFlag[link->index()] = true;
                    LinkTreeItem* item = linkTreeWidget->itemOfLink(link->index());
                    if(item){
                        ColumnCheckBox* checkBox = new ColumnCheckBox(
                            std::bind(&PoseSeqViewBase::onIkPartCheckClicked, this, item, _1));
                        linkTreeWidget->setAlignedItemWidget(item, ikPartColumn, checkBox);
                    }
                }
            }
        }
        if(!body->isFixedRootModel()){
            possibleIkLinkFlag[0] = true;
        }
    }
    
    const Listing& defaultIkLinks = *info.findListing("defaultIkInterpolationLinks");
    if(defaultIkLinks.isValid()){
        for(int i=0; i < defaultIkLinks.size(); ++i){
            Link* link = body->link(defaultIkLinks[i]);
            if(link){
                poseForDefaultStateSetting->addIkLink(link->index());
            }
        }
    }
}


void PoseSeqViewBase::initializeLinkTreeTraverse(QTreeWidgetItem* parentItem)
{
    int n = parentItem->childCount();
    for(int i=0; i < n; ++i){
        LinkTreeItem* item = dynamic_cast<LinkTreeItem*>(parentItem->child(i));
        if(!item){
            continue;
        }
        Link* link = item->link();

        if(!link || link->parent()){

            ColumnCheckBox* checkBox = new ColumnCheckBox(
                std::bind(&PoseSeqViewBase::onValidPartCheckClicked, this, item, _1));
            
            if(link && link->jointId() >= 0){
                poseForDefaultStateSetting->setJointPosition(link->jointId(), 0.0);
            } else if(item->isLinkGroup()){
                checkBox->setTristate(true);
            }
            linkTreeWidget->setAlignedItemWidget(item, validPartColumn, checkBox);
        }
        if(link){
            RadioButton* radioButton = new RadioButton();
            baseLinkRadioGroup->addButton(radioButton, link->index());
            linkTreeWidget->setAlignedItemWidget(item, baseLinkColumn, radioButton);
        }

        ColumnCheckBox* spCheck = new ColumnCheckBox(
            std::bind(&PoseSeqViewBase::onStationaryPointCheckClicked, this, item, _1));
        linkTreeWidget->setAlignedItemWidget(item, stationaryPointColumn, spCheck);
        
        initializeLinkTreeTraverse(item);
    }
}


void PoseSeqViewBase::togglePoseAttribute(std::function<bool(PosePtr& pose)> toggleFunction)
{
    if(selectedPoseIters.empty()){
        if(toggleFunction(poseForDefaultStateSetting)){
            updateLinkTreeModel();
        }

    } else {
        currentPoseSeqItem->beginEditing();
        bool modified = false;
        for(PoseIterSet::iterator p = selectedPoseIters.begin(); p != selectedPoseIters.end(); ++p){
            PosePtr pose = (*p)->get<Pose>();
            if(pose){
                seq->beginPoseModification(*p);
                modified = toggleFunction(pose);
                if(modified){
                    seq->endPoseModification(*p);
                }
            }
        }
        currentPoseSeqItem->endEditing(modified);

        //! \todo This should be processed by PoseSeq slots
        if(modified){
            doAutomaticInterpolationUpdate();
        }
    }
}


void PoseSeqViewBase::onBaseLinkRadioClicked()
{
    if(TRACE_FUNCTIONS){
        cout << "PoseSeqViewBase::onBaseLinkRadioClicked()" << endl;
    }

    int linkIndex = baseLinkRadioGroup->checkedId();
    Link* link = (linkIndex >= 0) ? body->link(linkIndex) : 0;
    togglePoseAttribute(std::bind(&PoseSeqViewBase::setBaseLink, this, _1, link));
}


bool PoseSeqViewBase::setBaseLink(PosePtr& pose, Link* link)
{
    if(TRACE_FUNCTIONS){
        cout << "PoseSeqViewBase::toggleBaseLink()" << endl;
    }
    
    bool modified = false;
    if(link){
        if(pose->baseLinkIndex() != link->index()){
            pose->setBaseLink(link->index(), link->p(), link->R());
            modified = true;
        }
    } else {
        if(pose->baseLinkInfo()){
            pose->invalidateBaseLink();
            modified = true;
        }
    }
    return modified;
}


void PoseSeqViewBase::onValidPartCheckClicked(LinkTreeItem* item, Qt::CheckState checkState)
{
    bool on = ((checkState == Qt::Unchecked) || (checkState == Qt::PartiallyChecked));
    
    if(item == zmpRow){
        togglePoseAttribute(std::bind(&PoseSeqViewBase::toggleZmp, this, _1, on));
    } else {
        Link* link = item->link();
        if(link){
            bool isIkPartChecked = isChecked(item, ikPartColumn);
            togglePoseAttribute(std::bind(&PoseSeqViewBase::toggleLink, this, _1, item, link, on, isIkPartChecked));
        } else {
            togglePoseAttribute(std::bind(&PoseSeqViewBase::togglePart, this, _1, item, on));
        }
    }
}


bool PoseSeqViewBase::toggleZmp(PosePtr& pose, bool on)
{
    bool modified = false;
    if(on){
        const Vector3& zmp = currentBodyItem->zmp();
        if(!pose->isZmpValid() || zmp != pose->zmp()){
            pose->setZmp(currentBodyItem->zmp());
            modified = true;
        }
    } else {
        if(pose->isZmpValid()){
            pose->invalidateZmp();
            modified = true;
        }
    }
    return modified;
}



bool PoseSeqViewBase::toggleLink(PosePtr& pose, LinkTreeItem* item, Link* link, bool partOn, bool ikOn)
{
    bool modified = false;
    int jId = link->jointId();
    if(partOn){
        if(jId >= 0){
            bool isSpChecked = isChecked(item, stationaryPointColumn);
            if(!pose->isJointValid(jId) || pose->jointPosition(jId) != link->q() ||
               pose->isJointStationaryPoint(jId) != isSpChecked){
                pose->setJointPosition(jId, link->q());
                pose->setJointStationaryPoint(jId, isSpChecked);
                modified = true;
            }
        }
        if(possibleIkLinkFlag[link->index()]){
            Pose::LinkInfo* info = pose->ikLinkInfo(link->index());
            if(!info){
                info = pose->addIkLink(link->index());
                modified = true;
            }
            if(setCurrentLinkStateToIkLink(link, info)){
                modified = true;
            }
            bool isSlave = !ikOn;
            if(info->isSlave() != isSlave){
                info->setSlave(isSlave);
                modified = true;
            }
        }
    } else {
        if(pose->isJointValid(jId)){
            pose->invalidateJoint(jId);
            modified = true;
        }
        if(pose->removeIkLink(link->index())){
            modified = true;
        }
    }
    return modified;
}


bool PoseSeqViewBase::togglePart(PosePtr& pose, LinkTreeItem* item, bool on)
{
    bool modified = false;
    
    Link* link = item->link();
    if(link){
        bool ikOn = false;
        if(possibleIkLinkFlag[link->index()]){
            if(isChecked(item, validPartColumn)){
                ikOn = isChecked(item, ikPartColumn);
            } else {
                ikOn = on;
            }
        }
        modified = toggleLink(pose, item, link, on, ikOn);
    }

    int n = item->childCount();
    for(int i=0; i < n; ++i){
        LinkTreeItem* childItem = dynamic_cast<LinkTreeItem*>(item->child(i));
        if(childItem){
            modified |= togglePart(pose, childItem, on);
        }
    }

    return modified;
}


void PoseSeqViewBase::onStationaryPointCheckClicked(LinkTreeItem* item, Qt::CheckState checkState)
{
    bool on = (checkState == Qt::Unchecked);
    if(item == zmpRow){
        togglePoseAttribute(std::bind(&PoseSeqViewBase::toggleZmpStationaryPoint, this, _1, on));
    } else {
        Link* link = item->link();
        if(link){
            togglePoseAttribute(std::bind(&PoseSeqViewBase::toggleStationaryPoint, this, _1, link, on));
        } else {
            if(checkState == Qt::PartiallyChecked){
                on = true;
            }
            togglePoseAttribute(std::bind(&PoseSeqViewBase::togglePartStationaryPoints, this, _1, item, on));
        }
    }
}


bool PoseSeqViewBase::toggleZmpStationaryPoint(PosePtr& pose, bool on)
{
    bool modified = false;
    if(on){
        if(!pose->isZmpStationaryPoint()){
            pose->setZmpStationaryPoint(on);
            modified = true;
        }
    } else {
        if(pose->isZmpStationaryPoint()){
            pose->setZmpStationaryPoint(on);
            modified = true;
        }
    }
    return modified;
}


bool PoseSeqViewBase::toggleStationaryPoint(PosePtr& pose, Link* link, bool on)
{
    bool modified = false;

    int id = link->jointId();
    if(pose->isJointValid(id)){
        pose->setJointStationaryPoint(id, on);
        modified = true;
    }
    Pose::LinkInfo* info = pose->ikLinkInfo(link->index());
    if(info){
        info->setStationaryPoint(on);
        modified = true;
    }
    return modified;
}


bool PoseSeqViewBase::togglePartStationaryPoints(PosePtr& pose, LinkTreeItem* item, bool on)
{
    bool modified = false;
    
    Link* link = item->link();
    if(link){
        modified = toggleStationaryPoint(pose, link, on);
    }
    int n = item->childCount();
    for(int i=0; i < n; ++i){
        LinkTreeItem* childItem = dynamic_cast<LinkTreeItem*>(item->child(i));
        if(childItem){
            modified |= togglePartStationaryPoints(pose, childItem, on);
        }
    }
    return modified;
}


void PoseSeqViewBase::onIkPartCheckClicked(LinkTreeItem* item, Qt::CheckState checkState)
{
    Link* link = item->link();
    if(link){
        bool ikOn = (checkState == Qt::Unchecked);
        bool partOn = ikOn | isChecked(item, validPartColumn);
        togglePoseAttribute(std::bind(&PoseSeqViewBase::toggleLink, this, _1, item, link, partOn, ikOn));
    }
}


void PoseSeqViewBase::onInterpolationParametersChanged()
{
    double newTimeScale = BodyMotionGenerationBar::instance()->timeScaleRatio();
    if(newTimeScale != timeScale){
        timeScale = newTimeScale;
        onTimeScaleChanged();
    }
}


void PoseSeqViewBase::onTimeScaleChanged()
{
    onSelectedPosesModified();
}


void PoseSeqViewBase::onSelectedItemsChanged(ItemList<PoseSeqItem> selectedItems)
{
    if(auto item = selectedItems.toSingle()){
        setCurrentPoseSeqItem(item);
    }
}


void PoseSeqViewBase::setCurrentPoseSeqItem(PoseSeqItemPtr poseSeqItem)
{
    if(poseSeqItem == currentPoseSeqItem){
        return;
    }
    
    poseSeqConnections.disconnect();

    currentPoseSeqItem = poseSeqItem;

    setCurrentItemName(poseSeqItem);

    connectionOfBodyKinematicStateEdited.disconnect();

    seq = 0;
    currentBodyItem = 0;
    body.reset();
    selectedPoseIters.clear();

    if(!poseSeqItem){
        if(linkTreeWidget->bodyItem()){
            linkTreeWidget->setBodyItem(0);
        }

    } else {
        poseSeqConnections.add(
            poseSeqItem->sigNameChanged().connect(
                std::bind(&PoseSeqViewBase::setCurrentItemName, this, poseSeqItem)));

        seq = currentPoseSeqItem->poseSeq();
        currentPoseIter = seq->end();

        currentBodyItem = poseSeqItem->findOwnerItem<BodyItem>();
        if(currentBodyItem){
            body = currentBodyItem->body();
        }
        linkTreeWidget->setBodyItem(currentBodyItem);
        if(currentBodyItem){
            connectionOfBodyKinematicStateEdited = currentBodyItem->sigKinematicStateEdited().connect(
                std::bind(&PoseSeqViewBase::onBodyKinematicStateEdited, this));
        }
            
        poseSeqConnections.add(
            seq->connectSignalSet(
                std::bind(&PoseSeqViewBase::onPoseInserted, this, _1, _2),
                std::bind(&PoseSeqViewBase::onPoseRemoving, this, _1, _2),
                std::bind(&PoseSeqViewBase::onPoseModified, this, _1)));

        poseSeqConnections.add(
            poseSeqItem->sigDetachedFromRoot().connect(
                std::bind(&PoseSeqViewBase::setCurrentPoseSeqItem, this, PoseSeqItemPtr())));
    }
}


PoseSeqViewBase::PoseIterSet::iterator PoseSeqViewBase::findPoseIterInSelected(PoseSeq::iterator poseIter)
{
    pair<PoseIterSet::iterator, PoseIterSet::iterator> range = selectedPoseIters.equal_range(poseIter);
    for(PoseIterSet::iterator p = range.first; p != range.second; ++p){
        if((*p) == poseIter){
            return p;
        }
    }
    return selectedPoseIters.end();
}


bool PoseSeqViewBase::toggleSelection(PoseSeq::iterator poseIter, bool adding, bool changeTime)
{
    if(!(selectedPoseIters.size() == 1 && *selectedPoseIters.begin() == poseIter)){ // Skip same single selection
    
        if(poseIter == seq->end()){
            if(selectedPoseIters.empty()){
                return false;
            }
            selectedPoseIters.clear();

        } else {
            PoseIterSet::iterator p = findPoseIterInSelected(poseIter);
            if(p == selectedPoseIters.end()){
                if(!adding){
                    selectedPoseIters.clear();
                }
                selectedPoseIters.insert(poseIter);
            } else {
                if(adding){
                    selectedPoseIters.erase(p);
                }
            }
        }
        
        updateLinkTreeModel();
        onSelectedPosesModified();
    }
    
    if(changeTime && (poseIter != seq->end())){
        double time = timeScale * poseIter->time();
        if(timeSyncCheck.isChecked()){
            timeBar->setTime(time);
        } else {
            onTimeChanged(time);
        }
    }

    return true;
}


void PoseSeqViewBase::selectAllPoses()
{
    selectedPoseIters.clear();
    for(PoseSeq::iterator p = seq->begin(); p != seq->end(); ++p){
        selectedPoseIters.insert(p);
    }
    updateLinkTreeModel();
    onSelectedPosesModified();
}


void PoseSeqViewBase::selectAllPosesAfterCurrentPosition()
{
    selectedPoseIters.clear();
    PoseSeq::iterator p = seq->seek(seq->begin(), currentTime);
    while(p != seq->end()){
        selectedPoseIters.insert(p++);
    }
    updateLinkTreeModel();
    onSelectedPosesModified();
}


void PoseSeqViewBase::selectAllPosesBeforeCurrentPosition()
{
    selectedPoseIters.clear();
    if(!seq->empty()){
        PoseSeq::iterator p = seq->seek(seq->begin(), currentTime);
        if(p != seq->end() && (p->time() == currentTime)){
            ++p;
        }
        do {
            selectedPoseIters.insert(--p);
        } while(p != seq->begin());
    }
    updateLinkTreeModel();
    onSelectedPosesModified();
}


void PoseSeqViewBase::selectPosesHavingSelectedLinks()
{
    if(!body || !seq){
        return;
    }
    
    const vector<int> selectedLinkIndices = linkTreeWidget->selectedLinkIndices();

    selectedPoseIters.clear();
    for(PoseSeq::iterator p = seq->begin(); p != seq->end(); ++p){
        PosePtr pose = p->get<Pose>();
        if(pose){
            bool match = true;
            for(size_t i=0; i < selectedLinkIndices.size(); ++i){
                int linkIndex = selectedLinkIndices[i];
                if(!pose->isJointValid(body->link(linkIndex)->jointId())){
                    if(!pose->ikLinkInfo(linkIndex)){
                        match = false;
                        break;
                    }
                }
            }
            if(match){
                selectedPoseIters.insert(p);
            }
        }
    }
    updateLinkTreeModel();
    onSelectedPosesModified();
}


void PoseSeqViewBase::selectPosesJustHavingSelectedLinks()
{
    if(!body || !seq){
        return;
    }
    
    const auto& linkSelection = linkTreeWidget->linkSelection();

    selectedPoseIters.clear();
    for(PoseSeq::iterator p = seq->begin(); p != seq->end(); ++p){
        PosePtr pose = p->get<Pose>();
        if(pose){
            bool match = true;
            for(size_t linkIndex=0; linkIndex < linkSelection.size(); ++linkIndex){
                bool hasLink = pose->isJointValid(body->link(linkIndex)->jointId()) || pose->ikLinkInfo(linkIndex);
                if((linkSelection[linkIndex] && !hasLink) || (!linkSelection[linkIndex] && hasLink)){
                    match = false;
                    break;
                }
            }
            if(match){
                selectedPoseIters.insert(p);
            }
        }
    }
    updateLinkTreeModel();
    onSelectedPosesModified();
}


void PoseSeqViewBase::removeSelectedPartsFromKeyPoses()
{
    if(!body || !seq || selectedPoseIters.empty()){
        return;
    }
    
    const std::vector<int>& selected = linkTreeWidget->selectedLinkIndices();
    bool doRemoveZmp = zmpRow->isSelected();

    if(selected.empty() && !doRemoveZmp){
        return;
    }

    PoseIterSet orgSelected(selectedPoseIters);
    currentPoseSeqItem->beginEditing();
    bool removed = false;

    for(PoseIterSet::iterator p = orgSelected.begin(); p != orgSelected.end(); ++p){
        PosePtr pose = (*p)->get<Pose>();
        if(pose){
            seq->beginPoseModification(*p);
            bool modified = false;
            for(size_t i=0; i < selected.size(); ++i){
                int linkIndex = selected[i];
                int jointId = body->link(linkIndex)->jointId();
                if(jointId >= 0){
                    modified |= pose->invalidateJoint(jointId);
                }
                modified |= pose->removeIkLink(linkIndex);
            }
            if(doRemoveZmp){
                modified |= pose->invalidateZmp();
            }
            if(pose->empty()){
                seq->erase(*p);
            } else if(modified){
                seq->endPoseModification(*p);
            }
            removed |= modified;
        }
    }
    
    if(currentPoseSeqItem->endEditing(removed)){
        doAutomaticInterpolationUpdate();
    }
}


bool PoseSeqViewBase::deleteSelectedPoses()
{
    if(!selectedPoseIters.empty()){
        PoseIterSet orgSelected(selectedPoseIters);
        currentPoseSeqItem->beginEditing();
        for(PoseIterSet::iterator p = orgSelected.begin(); p != orgSelected.end(); ++p){
            seq->erase(*p);
        }
        currentPoseSeqItem->endEditing();
        doAutomaticInterpolationUpdate();
        return true;
    }
    return false;
}


bool PoseSeqViewBase::cutSelectedPoses()
{
    if(copySelectedPoses()){
        return deleteSelectedPoses();
    }
    return false;
}


bool PoseSeqViewBase::copySelectedPoses()
{
    if(!selectedPoseIters.empty()){
        copiedPoses = new PoseSeq();
        PoseSeq::iterator destIter = copiedPoses->begin();
        double offset = - (*selectedPoseIters.begin())->time();
        for(PoseIterSet::iterator p = selectedPoseIters.begin(); p != selectedPoseIters.end(); ++p){
            PoseSeq::iterator srcIter = *p;
            destIter = copiedPoses->copyElement(destIter, srcIter, offset);
        }    
        return true;
    }
    return false;
}


bool PoseSeqViewBase::pasteCopiedPoses(double timeToPaste)
{
    if(!copiedPoses->empty()){
        currentPoseSeqItem->beginEditing();
        PoseSeq::iterator dest = seq->seek(currentPoseIter, timeToPaste, true);
        for(PoseSeq::iterator p = copiedPoses->begin(); p != copiedPoses->end(); ++p){
            dest = seq->copyElement(dest, p, timeToPaste);
        }
        currentPoseIter = dest;
        currentPoseSeqItem->endEditing();
        doAutomaticInterpolationUpdate();        
        return true;
    }
    return false;
}


/**
   @ret true if actually modified
*/
bool PoseSeqViewBase::moveSelectedPoses(double time0)
{
    bool modified = false;

    if(!selectedPoseIters.empty()){
        time0 = std::max(0.0, time0);
        double diff = time0 - (*selectedPoseIters.begin())->time();
        if(diff != 0.0){
            // Copy is needed because selectedPoseIters may change during the following loops
            PoseIterSet tmpSelectedPoseIters(selectedPoseIters);
            if(diff > 0.0){
                for(PoseIterSet::reverse_iterator p = tmpSelectedPoseIters.rbegin(); p != tmpSelectedPoseIters.rend(); ++p){
                    seq->changeTime(*p, (*p)->time() + diff);
                }
            } else {
                for(PoseIterSet::iterator p = tmpSelectedPoseIters.begin(); p != tmpSelectedPoseIters.end(); ++p){
                    seq->changeTime(*p, (*p)->time() + diff);
                }
            }
            modified = true;
        }
    }

    return modified;
}


bool PoseSeqViewBase::modifyTransitionTimeOfSelectedPoses(double ttime)
{
    bool modified = false;

    if(!selectedPoseIters.empty()){
        for(PoseIterSet::iterator p = selectedPoseIters.begin(); p != selectedPoseIters.end(); ++p){
            seq->beginPoseModification(*p);
            (*p)->setMaxTransitionTime(ttime);
            seq->endPoseModification(*p);
        }
        modified = true;
    }

    return modified;
}


void PoseSeqViewBase::popupContextMenu(QMouseEvent* event)
{
    popupMenu.popup(event->globalPos());
}


void PoseSeqViewBase::onSelectSpecifiedKeyPosesActivated()
{
    poseSelectionDialog->show();
}


void PoseSeqViewBase::onPoseSelectionDialogAccepted()
{
    if(!body || !seq){
        return;
    }

    selectedPoseIters.clear();
    const vector<int> selectedLinkIndices = linkTreeWidget->selectedLinkIndices();
    const double t0 = poseSelectionDialog->startTimeSpin.value();
    const double t1 = poseSelectionDialog->endTimeSpin.value();

    PoseSeq::iterator p = seq->seek(seq->begin(), t0);

    while(p != seq->end()){

        if(p->time() > t1){
            break;
        }

        if(poseSelectionDialog->selectedPartRadio.isChecked()){
            PosePtr pose = p->get<Pose>();
            if(pose){
                bool match = false;
                for(size_t i=0; i < selectedLinkIndices.size(); ++i){
                    int linkIndex = selectedLinkIndices[i];
                    if(pose->isJointValid(body->link(linkIndex)->jointId()) || pose->ikLinkInfo(linkIndex)){
                        match = true;
                        break;
                    }
                }
                if(match){
                    selectedPoseIters.insert(p);
                }
            }
        } else {
            selectedPoseIters.insert(p);
        }
        
        ++p;
    }

    updateLinkTreeModel();
    onSelectedPosesModified();
}


void PoseSeqViewBase::onAdjustStepPositionsActivated()
{
    if(currentPoseSeqItem && currentBodyItem){
        PoseSeq::iterator origin;
        if(selectedPoseIters.size() == 1){
            origin = *selectedPoseIters.begin();
        } else {
            origin = seq->begin();
        }

        LeggedBodyHelperPtr legged = getLeggedBodyHelper(body);
        if(legged->isValid()){
            int n = legged->numFeet();
            vector<int> footLinkIndices(n);
            for(int i=0; i < n; ++i){
                footLinkIndices[i] = legged->footLink(i)->index();
            }
            adjustStepPositions(seq, footLinkIndices, origin);
            doAutomaticInterpolationUpdate();
        }
    }
}


void PoseSeqViewBase::onRotateYawOrientationsActivated()
{
    yawOrientationRotationDialog->show();
}


void PoseSeqViewBase::onYawOrientationRotationDialogAccepted()
{
    if(currentPoseSeqItem && selectedPoseIters.size() == 1){

        PoseSeq::iterator poseIter = *selectedPoseIters.begin();
        Vector3 center(yawOrientationRotationDialog->centerPosSpins[0].value(),
                       yawOrientationRotationDialog->centerPosSpins[1].value(),
                       0.0);
        double angle = radian(yawOrientationRotationDialog->angleSpin.value());
        rotateYawOrientations(seq, poseIter, center, angle);

        /*
          PosePtr pose = poseIter->get<Pose>();
          if(pose){
          const std::vector<int>& selectedLinkIndices =
          LinkSelectionView::mainInstance()->getSelectedLinkIndices(currentBodyItem);

          if(selectedLinkIndices.size() == 1){
          Pose::LinkInfo* linkInfo = pose->ikLinkInfo(selectedLinkIndices.front());
          if(linkInfo){
          double angle = radian(yawOrientationRotationDialog->angleSpin.value());
          rotateYawOrientations(seq, ++poseIter, linkInfo->p, angle);
          }
          }
          }
        */
    }
}



void PoseSeqViewBase::onAdjustWaistPositionActivated()
{
    linkPositionAdjustmentDialog->show();
}


void PoseSeqViewBase::onLinkPositionAdjustmentDialogAccepted()
{
    if(currentPoseSeqItem && currentBodyItem && !selectedPoseIters.empty()){

        LeggedBodyHelperPtr legged = getLeggedBodyHelper(body);
        if(legged->isValid()){

            int waistLinkIndex = currentBodyItem->body()->rootLink()->index();
        
            int n = legged->numFeet();
            vector<int> footLinkIndices(n);
            for(int i=0; i < n; ++i){
                footLinkIndices[i] = legged->footLink(i)->index();
            }
        
            currentPoseSeqItem->beginEditing();
        
            for(PoseIterSet::iterator p = selectedPoseIters.begin(); p != selectedPoseIters.end(); ++p){
                PosePtr pose = (*p)->get<Pose>();
                if(pose){
                    seq->beginPoseModification(*p);
                    
                    /// \todo arbitrar selected link should be processed here
                    Pose::LinkInfo* waistInfo = pose->ikLinkInfo(waistLinkIndex);
                    if(waistInfo){
                        for(int i=0; i < 3; ++i){
                            if(linkPositionAdjustmentDialog->targetAxisCheck[i].isChecked()){
                                double p = linkPositionAdjustmentDialog->positionSpin[i].value();
                                if(linkPositionAdjustmentDialog->absoluteRadio.isChecked()){
                                    waistInfo->p[i] = p;
                                } else {
                                    waistInfo->p[i] += p;
                                }
                            }
                        }
                    }
                    seq->endPoseModification(*p);
                }
            }
            
            currentPoseSeqItem->endEditing();
            doAutomaticInterpolationUpdate();
        }
    }
}


void PoseSeqViewBase::onUpdateKeyposesWithBalancedTrajectoriesActivated()
{
    if(currentPoseSeqItem){
        ostringstream mout;
        if(currentPoseSeqItem->updateKeyPosesWithBalancedTrajectories(mout)){
            MessageView::mainInstance()->notify(
                _("Original key poses have been updated to be balanced ones."));
        } else {
            MessageView::mainInstance()->notify(
                _("Operation failed ! Key poses cannot be updated."));
        }
        if(!mout.str().empty()){
            os << mout.str() << endl;
        }
    }
}


void PoseSeqViewBase::onFlipPosesActivated()
{
    if(currentPoseSeqItem && currentBodyItem){
        MessageView::mainInstance()->notify(_("flipping all the poses against x-z plane ..."));
        flipPoses(seq, body);
        doAutomaticInterpolationUpdate();
    }
}


void PoseSeqViewBase::countSelectedKeyPoses()
{
    MessageView::mainInstance()->notify(
        format(_("The number of selected key poses is {}"), selectedPoseIters.size()));
}


void PoseSeqViewBase::onSelectedPosesModified()
{
    if(selectedPoseIters.empty()){
        updateButton.setEnabled(false);
        deleteButton.setEnabled(false);
    } else {
        updateButton.setEnabled(true);
        deleteButton.setEnabled(true);
    }
}


void PoseSeqViewBase::setCurrentItemName(ItemPtr item)
{
    if(!item || item->name().empty()){
        currentItemLabel.setText(textForEmptyName);
    } else {
        currentItemLabel.setText(item->name().c_str());
    }
}


/**
   \todo Show visual effect to emphasize the key poses beging updated
*/
void PoseSeqViewBase::onBodyKinematicStateEdited()
{
    if(TRACE_FUNCTIONS){
        cout << "PoseSeqViewBase::onBodyKinematicStateEdited()" << endl;
    }
    
    if(autoUpdateModeCheck.isChecked()){
        if(!selectedPoseIters.empty()){
            bool timeMatches = true;
            for(PoseIterSet::iterator p = selectedPoseIters.begin(); p != selectedPoseIters.end(); ++p){
                double qtime = quantizedTime((*p)->time());
                if(qtime != timeBar->time()){
                    timeMatches = false;
                    break;
                }
            }
            if(timeMatches){
                setCurrentBodyStateToSelectedPoses(!updateAllToggle.isChecked());
                InfoBar::instance()->notify(_("Selected key poses have been updated."));
            }
        }
    }
}


PoseSeq::iterator PoseSeqViewBase::insertPose()
{
    PoseSeq::iterator poseIter = seq->end();
        
    PosePtr pose = new Pose(body->numJoints());

    bool hasValidPart = false;

    for(int i=0; i < body->numLinks(); ++i){
        Link* link = body->link(i);
        LinkTreeItem* item = linkTreeWidget->itemOfLink(link->index());
        if(item){
            if(link->jointId() >= 0 && isChecked(item, validPartColumn)){
                pose->setJointPosition(link->jointId(), link->q());
                hasValidPart = true;
                if(isChecked(item, stationaryPointColumn)){
                    pose->setJointStationaryPoint(link->jointId());
                }
            }
            if(possibleIkLinkFlag[link->index()]){
                if(isChecked(item, validPartColumn) || isChecked(item, ikPartColumn)){
                    Pose::LinkInfo* info = pose->addIkLink(link->index());
                    info->setStationaryPoint(isChecked(item, stationaryPointColumn));
                    setCurrentLinkStateToIkLink(link, info);
                    if(isChecked(item, baseLinkColumn)){
                        pose->setBaseLink(link->index(), link->p(), link->R());
                    }
                    if(!isChecked(item, ikPartColumn)){
                        info->setSlave(true);
                    }
                    hasValidPart = true;
                }
            }
        }
    }

    if(isChecked(zmpRow, validPartColumn)){
        pose->setZmp(currentBodyItem->zmp());
        hasValidPart = true;
        if(isChecked(zmpRow, stationaryPointColumn)){
            pose->setZmpStationaryPoint();
        }
    }

    if(hasValidPart){
        poseIter = insertPoseUnit(pose);
    } else {
        showWarningDialog(_("Please check parts needed for making a desired pose."));
    }

    return poseIter;
}


PoseSeq::iterator PoseSeqViewBase::insertPronunSymbol()
{
    PronunSymbolPtr pronun(new PronunSymbol());
    return insertPoseUnit(pronun);
}


PoseSeq::iterator PoseSeqViewBase::insertPoseUnit(PoseUnitPtr poseUnit)
{
    PoseSeq::iterator poseIter = seq->insert(currentPoseIter, currentTime / timeScale, poseUnit);
    poseIter->setMaxTransitionTime(transitionTimeSpin.value() / timeScale);
    doAutomaticInterpolationUpdate();
    toggleSelection(poseIter, false, false);

    currentPoseIter = poseIter;

    return poseIter;
}


double PoseSeqViewBase::quantizedTime(double time)
{
    double r = timeBar->frameRate();
    double frame = nearbyint(r * time);
    return frame / r;
}


void PoseSeqViewBase::onUpdateButtonClicked()
{
    setCurrentBodyStateToSelectedPoses(!updateAllToggle.isChecked());
}


void PoseSeqViewBase::setCurrentBodyStateToSelectedPoses(bool onlySelected)
{
    if(TRACE_FUNCTIONS){
        cout << "PoseSeqViewBase::setCurrentBodyStateToSelectedPoses()" << endl;
    }
    if(body){
        if(!selectedPoseIters.empty()){

            // quantize selected pose times
            PoseIterSet prevSelected(selectedPoseIters);
            selectedPoseIters.clear();
            for(PoseIterSet::iterator p = prevSelected.begin(); p != prevSelected.end(); ++p){
                double qtime = quantizedTime((*p)->time());
                selectedPoseIters.insert(seq->changeTime(*p, qtime));
            }
            
            bool updated = false;
            currentPoseSeqItem->beginEditing();
            for(PoseIterSet::iterator p = selectedPoseIters.begin(); p != selectedPoseIters.end(); ++p){
                PosePtr pose = (*p)->get<Pose>();
                if(pose){
                    seq->beginPoseModification(*p);
                    if(setCurrentBodyStateToPose(pose, onlySelected)){
                        updated = true;
                        seq->endPoseModification(*p);
                    }
                }
            }
            currentPoseSeqItem->endEditing(updated);

            //! \todo This should be processed by PoseSeq slots
            if(updated){
                doAutomaticInterpolationUpdate();
            }
        }
    }
}


bool PoseSeqViewBase::setCurrentBodyStateToPose(PosePtr& pose, bool onlySelected)
{
    const auto& linkSelection =
        LinkSelectionView::instance()->linkSelection(currentBodyItem);
            
    bool updated = false;
            
    int n = pose->numJoints();
    for(int i=0; i < n; ++i){
        if(pose->isJointValid(i)){
            Link* joint = body->joint(i);
            if(!onlySelected || linkSelection[joint->index()]){
                const double q = body->joint(i)->q();
                if(q != pose->jointPosition(i)){
                    pose->setJointPosition(i, q);
                    updated = true;
                }
            }
        }
    }

    for(Pose::LinkInfoMap::iterator it = pose->ikLinkBegin(); it != pose->ikLinkEnd(); ++it){
        const int linkIndex = it->first;
        Link* link = body->link(linkIndex);
        if(link && (!onlySelected || linkSelection[link->index()])){
            updated |= setCurrentLinkStateToIkLink(link, &it->second);
        }
    }

    if(pose->isZmpValid()){
        const Vector3& zmp = currentBodyItem->zmp();
        if(zmp != pose->zmp()){
            pose->setZmp(zmp);
            updated = true;
        }
    }

    return updated;
}


/**
   @return true if state is modified
*/
bool PoseSeqViewBase::setCurrentLinkStateToIkLink(Link* link, Pose::LinkInfo* linkInfo)
{
    bool updated = false;
    
    if(linkInfo->p != link->p()){
        linkInfo->p = link->p();
        updated = true;
    }
    if(linkInfo->R != link->R()){
        linkInfo->R = link->R();
        updated = true;
    }
    
    std::vector<Vector3> contactPoints;
    const std::vector<CollisionLinkPairPtr>& collisions = currentBodyItem->collisionsOfLink(link->index());
    for(size_t i=0; i < collisions.size(); ++i){
        if(!collisions[i]->isSelfCollision()){
            CollisionLinkPair& collisionPair = *collisions[i];
            for(std::vector<Collision>::iterator iter = collisionPair.collisions.begin(); iter != collisionPair.collisions.end(); ++iter){
                contactPoints.push_back(link->R().transpose()*(iter->point - link->p()));
            }
        }
    }

    if(contactPoints.size() > 0){
        /**
           \todo set a parting direction correctly
           (now it is assumed that the touching only happens for the flat and level floor).
        */
        Vector3 partingDirection(0.0, 0.0, 1.0);
        if(!linkInfo->isTouching() || linkInfo->partingDirection() != partingDirection || linkInfo->contactPoints() != contactPoints){
            linkInfo->setTouching(partingDirection, contactPoints);
            updated = true;
        }
    } else {
        if(linkInfo->isTouching()){
            linkInfo->clearTouching();
            updated = true;
        }
    }

    return updated;
}


void PoseSeqViewBase::onDeleteButtonClicked()
{
    cutSelectedPoses();
}


void PoseSeqViewBase::onPoseInserted(PoseSeq::iterator it, bool isMoving)
{
    if(isSelectedPoseMoving && isMoving){
        selectedPoseIters.insert(it);
        isSelectedPoseMoving = false;
        onSelectedPosesModified();
    }
}

    
void PoseSeqViewBase::onPoseRemoving(PoseSeq::iterator it, bool isMoving)
{
    if(TRACE_FUNCTIONS){
        cout << "PoseSeqViewBase::onPoseRemoving(): isMoving = " << isMoving << endl;
    }

    if(it == currentPoseIter){
        if(currentPoseIter != seq->begin()){
            --currentPoseIter;
        } else if(currentPoseIter != seq->end()){
            ++currentPoseIter;
        }
    }

    PoseIterSet::iterator p = findPoseIterInSelected(it);
    if(p != selectedPoseIters.end()){
        selectedPoseIters.erase(p);
        if(isMoving){
            isSelectedPoseMoving = true;
        } else {
            onSelectedPosesModified();
        }
    }
}


/**
   @todo update currentBodyItem and call notifyUpdate in this function ?
*/
void PoseSeqViewBase::onPoseModified(PoseSeq::iterator it)
{
    if(TRACE_FUNCTIONS){
        cout << "PoseSeqViewBase::onPoseModified()" << endl;
    }

    if(!selectedPoseIters.empty() && it == *selectedPoseIters.begin()){
        updateLinkTreeModel();
        onSelectedPosesModified();
    }
}


void PoseSeqViewBase::updateLinkTreeModel()
{
    if(TRACE_FUNCTIONS){
        cout << "PoseSeqViewBase::updateLinkTreeModel()" << endl;
    }
    
    PosePtr pose;

    for(PoseIterSet::iterator p = selectedPoseIters.begin(); p != selectedPoseIters.end(); ++p){
        pose = (*p)->get<Pose>();
        if(pose){
            break;
        }
    } 
    if(!pose){
        pose = poseForDefaultStateSetting;
    }

    linkTreeAttributeChangeConnections.block(); // Probably this set of block / unblock is not needed

    int n = linkTreeWidget->topLevelItemCount();
    for(int i=0; i < n; ++i){
        LinkTreeItem* item = dynamic_cast<LinkTreeItem*>(linkTreeWidget->topLevelItem(i));
        if(item){
            updateLinkTreeModelSub(item, linkTreeWidget->bodyItem()->body(), *pose);
        }
    }

    linkTreeAttributeChangeConnections.unblock();
}


PoseSeqViewBase::ChildrenState PoseSeqViewBase::updateLinkTreeModelSub
(LinkTreeItem* item, const BodyPtr& body, const Pose& pose)
{
    ChildrenState state;

    int n = item->childCount();
    for(int i=0; i < n; ++i){
        LinkTreeItem* childItem = dynamic_cast<LinkTreeItem*>(item->child(i));
        if(childItem){
            ChildrenState childrenState = updateLinkTreeModelSub(childItem, body, pose);
            state.validChildExists |= childrenState.validChildExists;
            state.allChildrenAreValid &= childrenState.allChildrenAreValid;
            state.childWithStationaryPointExists |= childrenState.childWithStationaryPointExists;
            state.allChildrenAreStationaryPoints &= childrenState.allChildrenAreStationaryPoints;
        }
    }

    if(item == zmpRow){
        if(pose.isZmpValid()){
            setChecked(item, validPartColumn, true);
            setChecked(item, stationaryPointColumn, pose.isZmpStationaryPoint());
        } else {
            setChecked(item, validPartColumn, false);
            setChecked(item, stationaryPointColumn, false);
        }
    } else {
        Link* link = item->link();

        if(link){
            bool isBaseLink = false;
            bool isValidPart = false;
            bool isStationaryPoint = false;
            bool isIkPart = false;
            
            const Pose::LinkInfo* linkInfo = pose.ikLinkInfo(link->index());
            if(linkInfo){
                isValidPart = true;
                if(!possibleIkLinkFlag[link->index()]){
                    /// \todo put warning here or do the following call ?
                    //pose.removeIkLink(link->index);
                } else if(!linkInfo->isSlave()){
                    isIkPart = true;
                    isBaseLink = linkInfo->isBaseLink();
                    if(linkInfo->isStationaryPoint()){
                        isStationaryPoint = true;
                    }
                }
            }
            
            int jointId = link->jointId();
            if(jointId >= 0){
                if(pose.isJointValid(jointId)){
                    isValidPart = true;
                    if(pose.isJointStationaryPoint(jointId)){
                        isStationaryPoint = true;
                    }
                }
            }

            if(isValidPart && !isStationaryPoint){
                state.allChildrenAreStationaryPoints = false;
            }
            if(!isValidPart){
                state.allChildrenAreValid = false;
            }

            setChecked(item, baseLinkColumn, isBaseLink);
            setChecked(item, validPartColumn, isValidPart);
            setChecked(item, stationaryPointColumn, isStationaryPoint);
            setChecked(item, ikPartColumn, isIkPart);
            state.validChildExists = isValidPart;
            state.childWithStationaryPointExists |= isStationaryPoint;

        } else {
            
            if(state.allChildrenAreValid){
                setChecked(item, validPartColumn, true);
            } else if(state.validChildExists){
                setCheckState(item, validPartColumn, Qt::PartiallyChecked);
            } else {
                setChecked(item, validPartColumn, false);
            }

            if(state.allChildrenAreStationaryPoints && state.childWithStationaryPointExists){
                setChecked(item, stationaryPointColumn, true);
            } else if(state.childWithStationaryPointExists){
                setCheckState(item, stationaryPointColumn, Qt::PartiallyChecked);
            } else {
                setChecked(item, stationaryPointColumn, false);
            }
        }
    }

    return state;
}


void PoseSeqViewBase::doAutomaticInterpolationUpdate()
{
    BodyMotionGenerationBar* generationBar = BodyMotionGenerationBar::instance();
    if(generationBar->isAutoInterpolationUpdateMode()){
        currentPoseSeqItem->updateInterpolation(); // not needed ?
        if(generationBar->isAutoGenerationMode()){
            currentPoseSeqItem->updateTrajectory();
        }
    }
}


bool PoseSeqViewBase::storeState(Archive& archive)
{
    archive.writeItemId("currentPoseSeqItem", currentPoseSeqItem);
    archive.write("defaultTransitionTime", transitionTimeSpin.value());
    archive.write("updateAll", updateAllToggle.isChecked());
    archive.write("autoUpdate", autoUpdateModeCheck.isChecked());
    archive.write("timeSync", timeSyncCheck.isChecked());

    linkPositionAdjustmentDialog->storeState(archive);
    
    return linkTreeWidget->storeState(archive);
}


bool PoseSeqViewBase::restoreState(const Archive& archive)
{
    transitionTimeSpin.setValue(archive.get("defaultTransitionTime", transitionTimeSpin.value()));
    updateAllToggle.setChecked(archive.get("updateAll", updateAllToggle.isChecked()));
    autoUpdateModeCheck.setChecked(archive.get("autoUpdate", autoUpdateModeCheck.isChecked()));
    timeSyncCheck.setChecked(archive.get("timeSync", timeSyncCheck.isChecked()));
    
    linkPositionAdjustmentDialog->restoreState(archive);
    
    archive.addPostProcess(
        std::bind(&PoseSeqViewBase::restoreCurrentPoseSeqItem, this, std::ref(archive)));

    linkTreeWidget->restoreState(archive);
        
    return true;
}


void PoseSeqViewBase::restoreCurrentPoseSeqItem(const Archive& archive)
{
    PoseSeqItem* item = archive.findItem<PoseSeqItem>("currentPoseSeqItem");
    if(item){
        setCurrentPoseSeqItem(item);
    }
}
  
