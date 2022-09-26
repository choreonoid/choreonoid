/*! @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_POSE_SEQ_PLUGIN_POSE_SEQ_VIEW_BASE_H
#define CNOID_POSE_SEQ_PLUGIN_POSE_SEQ_VIEW_BASE_H

#include "PoseSeqItem.h"
#include <cnoid/ItemList>
#include <cnoid/View>
#include <cnoid/LinkDeviceTreeWidget>
#include <cnoid/ConnectionSet>
#include <cnoid/TimeBar>
#include <cnoid/Archive>
#include <cnoid/MenuManager>
#include <cnoid/Link>
#include <cnoid/Buttons>
#include <cnoid/CheckBox>
#include <cnoid/ButtonGroup>
#include <cnoid/SpinBox>
#include <cnoid/BodyItem>
#include <QBoxLayout>
#include <QLabel>
#include <ostream>
#include <set>

namespace cnoid {

class PoseSelectionDialog; /// \todo this should be independent ?
class LinkPositionAdjustmentDialog; /// \todo this should be independent ?
class YawOrientationRotationDialog;
        
class PoseSeqViewBase
{
public:
    PoseSeqViewBase(View* view);
    virtual ~PoseSeqViewBase();

    View* view;
    std::ostream& os;

    QString textForEmptyName;

    PoseSeqItemPtr currentPoseSeqItem;
    PoseSeqPtr seq;
    bool isSelectedPoseMoving;
    BodyItemPtr currentBodyItem;
    BodyPtr body;
    double currentTime;
    double timeScale;

    PoseSeq::iterator currentPoseIter;

    struct PoseIterTimeComp {
        bool operator()(const PoseSeq::iterator it1, const PoseSeq::iterator it2) const {
            return it1->time() < it2->time();
        }
    };
    typedef std::multiset<PoseSeq::iterator, PoseIterTimeComp> PoseIterSet;
    PoseIterSet selectedPoseIters;

    PoseSeqPtr copiedPoses;
    
    ConnectionSet staticConnections;
    ConnectionSet poseSeqConnections;
    Connection connectionOfBodyKinematicStateUpdated;

    ConnectionSet linkTreeAttributeChangeConnections;

    TimeBar* timeBar;
    Connection connectionOfTimeChanged;

    LinkDeviceTreeWidget* linkTreeWidget;
    int baseLinkColumn;
    ButtonGroup* baseLinkRadioGroup;
    int validPartColumn;
    int stationaryPointColumn;
    int ikPartColumn;
    std::vector<bool> possibleIkLinkFlag;
    LinkDeviceTreeItem* zmpRow;
        
    PosePtr poseForDefaultStateSetting;

    QLabel currentItemLabel;
    CheckBox timeSyncCheck;
            
    ToolButton insertPoseButton;
    ToolButton updateButton;
    ToggleToolButton updateAllToggle;
    ToolButton deleteButton;
    CheckBox autoUpdateModeCheck;
    DoubleSpinBox transitionTimeSpin;

    struct ChildrenState
    {
        ChildrenState()
            : validChildExists(false), allChildrenAreValid(true),
              childWithStationaryPointExists(false), allChildrenAreStationaryPoints(true) { }
        bool validChildExists;
        bool allChildrenAreValid;
        bool childWithStationaryPointExists;
        bool allChildrenAreStationaryPoints;
    };

    Menu popupMenu;
    MenuManager menuManager;

    PoseSelectionDialog* poseSelectionDialog;
    LinkPositionAdjustmentDialog* linkPositionAdjustmentDialog;
    YawOrientationRotationDialog* yawOrientationRotationDialog;

    PoseSeq::iterator insertPose();
    PoseSeq::iterator insertPronunSymbol();
    PoseSeq::iterator insertPoseUnit(PoseUnit* poseUnit); 
    PoseIterSet::iterator findPoseIterInSelected(PoseSeq::iterator poseIter);
    bool toggleSelection(PoseSeq::iterator poseIter, bool adding, bool changeTime);
    void selectAllPoses();
    void selectAllPosesAfterCurrentPosition();
    void selectAllPosesBeforeCurrentPosition();
    void selectPosesHavingSelectedLinks();
    void selectPosesJustHavingSelectedLinks();
    void removeSelectedPartsFromKeyPoses();
    void doAutomaticInterpolationUpdate();
    void updateLinkTreeModel();
    bool deleteSelectedPoses();
    bool cutSelectedPoses();
    bool copySelectedPoses();
    bool pasteCopiedPoses(double timeToPaste);
    bool moveSelectedPoses(double time0);
    bool modifyTransitionTimeOfSelectedPoses(double ttime);
    void popupContextMenu(QMouseEvent* event);

    void onSelectSpecifiedKeyPosesActivated();
    void onPoseSelectionDialogAccepted();
    void onAdjustStepPositionsActivated();
    void onRotateYawOrientationsActivated();
    void onYawOrientationRotationDialogAccepted();
    void onAdjustWaistPositionActivated();
    void onLinkPositionAdjustmentDialogAccepted();
    void onUpdateKeyposesWithBalancedTrajectoriesActivated();
    void onFlipPosesActivated();
    void countSelectedKeyPoses();
    double quantizedTime(double time);
        
    virtual void onLinkTreeUpdateRequest(bool isInitialCreation);
    virtual void setCurrentPoseSeqItem(PoseSeqItem* poseSeqItem);
    virtual void onTimeScaleChanged();
    virtual void onSelectedPosesModified();
    virtual void onDeleteButtonClicked();
    virtual void onPoseInserted(PoseSeq::iterator it, bool isMoving);
    virtual void onPoseRemoving(PoseSeq::iterator it, bool isMoving);
    virtual void onPoseModified(PoseSeq::iterator it);
    virtual bool onTimeChanged(double time) = 0;
    virtual void onInsertPoseButtonClicked() = 0;
    virtual bool restoreState(const Archive& archive);
    virtual bool storeState(Archive& archive);

    void onViewActivated();
    void onViewDeactivated();
    void onTimeSyncCheckToggled();
    void setupOperationParts();
    void setupLinkTreeWidget();
    bool isChecked(LinkDeviceTreeItem* item, int column);
    void setChecked(LinkDeviceTreeItem* item, int column, bool checked);
    void setCheckState(LinkDeviceTreeItem* item, int column, Qt::CheckState state);
    void initializeLinkTree();
    void initializeLinkTreeIkLinkColumn();
    void initializeLinkTreeTraverse(QTreeWidgetItem* parentItem);

    void togglePoseAttribute(std::function<bool(Pose* pose)> toggleFunction);
    void onBaseLinkRadioClicked();
    bool setBaseLink(Pose* pose, Link* link);
    void onValidPartCheckClicked(LinkDeviceTreeItem* item, Qt::CheckState checkState);
    bool toggleZmp(Pose* pose, bool on);
    bool toggleLink(Pose* pose, LinkDeviceTreeItem* item, Link* link, bool partOn, bool ikOn);
    bool togglePart(Pose* pose, LinkDeviceTreeItem* item, bool on);
    void onStationaryPointCheckClicked(LinkDeviceTreeItem* linkTreeItem, Qt::CheckState checkState);
    bool toggleZmpStationaryPoint(Pose* pose, bool on);
    bool toggleStationaryPoint(Pose* pose, Link* link, bool on);
    bool togglePartStationaryPoints(Pose* pose, LinkDeviceTreeItem* item, bool on);
    void onIkPartCheckClicked(LinkDeviceTreeItem* item, Qt::CheckState checkState);

    void onInterpolationParametersChanged();
    void onSelectedItemsChanged(ItemList<PoseSeqItem> selectedItems);

    void setCurrentItemName(Item* item);
    void onBodyKinematicStateUpdated();
    void onUpdateButtonClicked();
    void setCurrentBodyStateToSelectedPoses(bool onlySelected);
    bool setCurrentBodyStateToPose(Pose* pose, bool onlySelected);
    bool setCurrentLinkStateToIkLink(Link* link, Pose::LinkInfo* linkInfo);
    ChildrenState updateLinkTreeModelSub(LinkDeviceTreeItem* item,  Body* body, Pose* pose);

private:
    void restoreCurrentPoseSeqItem(const Archive& archive);
};

}

#endif
