#include "HumanoidPoseFetchView.h"
#include "PoseSeqItem.h"
#include "BodyKeyPose.h"
#include "PoseRollView.h"
#include <cnoid/ViewManager>
#include <cnoid/RootItem>
#include <cnoid/BodyItem>
#include <cnoid/Body>
#include <cnoid/JointPath>
#include <cnoid/LinkGroup>
#include <cnoid/LeggedBodyHelper>
#include <cnoid/TimeBar>
#include <cnoid/InfoBar>
#include <cnoid/Archive>
#include <cnoid/MessageView>
#include <cnoid/AppUtil>
#include <cnoid/ConnectionSet>
#include <cnoid/EigenUtil>
#include <cnoid/ComboBox>
#include <cnoid/Separator>
#include <cnoid/CheckBox>
#include <cnoid/Buttons>
#include <cnoid/ButtonGroup>
#include <cnoid/DoubleSpinBox>
#include <QBoxLayout>
#include <QGridLayout>
#include <QLabel>
#include <QKeyEvent>
#include <cnoid/stdx/clamp>
#include <fmt/format.h>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

namespace {

enum BodyPartId {
    Head = 1 << 0,
    RightArm = 1 << 1,
    LeftArm = 1 << 2,
    Chest = 1 << 3,
    Waist = 1 << 4,
    RightLeg = 1 << 5,
    LeftLeg = 1 << 6,
    Zmp= 1 << 7,

    Arms = RightArm | LeftArm,
    UpperBody = Head | Arms | Chest,
    Legs = RightLeg | LeftLeg,
    LowerBody = Legs | Waist,
    WholeBody = Waist | UpperBody | Legs
};

enum FetchOption {
    NoFetchOption,
    FootGrounding,
    ZmpAtCenterOfBothFeet,
    ZmpAtRightFoot,
    ZmpAtLeftFoot
};

enum AdjustmentCoordinateId {
    GlobalCoordinate,
    LocalCoordinate
};

class BodyPartCheckBox : public CheckBox
{
public:
    BodyPartCheckBox()
    {
        setTristate(true);
    }
    
protected:
    virtual void nextCheckState() override
    {
        switch(checkState()){
        case Qt::Unchecked:
            break;
        case Qt::PartiallyChecked:
        case Qt::Checked:
        default:
            setCheckState(Qt::Unchecked);
            break;
        }
    }
};

string getPoseId(const PoseSeq::iterator it, const BodyKeyPose* pose)
{
    return format("{0:0X}@{1:.2f}", reinterpret_cast<ulong>(pose) & 0xffffffff, it->time());
}

}

namespace cnoid {

class HumanoidPoseFetchView::Impl
{
public:
    HumanoidPoseFetchView* self;

    ScopedConnectionSet baseConnections;
    ScopedConnection poseRollViewTimeConnection;
    ScopedConnectionSet appConnections;

    BodyItem* srcBodyItem;
    LinkPtr waistLink;
    vector<LinkPtr> headLinks;
    LinkPtr headRpyJoints[3];
    vector<LinkPtr> chestLinks;
    LinkPtr chestRpyJoints[3];
    vector<LinkPtr> armLinkSets[2];
    // For the body object in the PoseSeqItem's PoseSeqInterpolator
    shared_ptr<JointPath> armJointPaths[2];
    vector<LinkPtr> legLinkSets[2];
    shared_ptr<JointPath> legJointPaths[2];
    shared_ptr<JointPath> waistToFootJointPaths[2];
    LeggedBodyHelperPtr leggedBody;
        
    PoseSeqItem* poseSeqItem;
    ScopedConnectionSet poseSeqConnections;
    PoseSeq::iterator currentPoseIter;
    vector<PoseSeq::iterator> targetPoseIters;
    double srcTime;
    double poseSeqTime;

    ComboBox srcBodyItemCombo;
    vector<BodyItem*> srcBodyItemCandidates;
    QLabel srcTimeLabel;
    QLabel targetPoseSeqLabel;
    QLabel poseSeqTimeLabel;
    QLabel targetPoseLabel;
    CheckBox forceNewPoseCheck;
    bool isRotationAdjustmentMode;
    bool isSmallStepAdjustmentMode;
    
    BodyPartCheckBox wholeBodyCheck;
    PushButton wholeBodyButton;
    CheckBox jointRangeLimitCheck;
    BodyPartCheckBox upperBodyCheck;
    PushButton upperBodyButton;
    CheckBox upperBodyWaistCheck;
    BodyPartCheckBox headCheck;
    PushButton headButton;
    PushButton headAxisUpButtons[3];
    PushButton headAxisDownButtons[3];

    struct ArmWidgetSet
    {
        BodyPartCheckBox partCheck;
        PushButton fetchButton;
        PushButton axisUpButtons[3];
        PushButton axisDownButtons[3];
    };
    ArmWidgetSet armWidgetSets[2];
    BodyPartCheckBox armsCheck;
    PushButton armsButton;
        
    BodyPartCheckBox chestCheck;
    PushButton chestButton;
    PushButton chestAxisUpButtons[3];
    PushButton chestAxisDownButtons[3];

    BodyPartCheckBox waistCheck;
    PushButton waistButton;
    PushButton waistAxisUpButtons[3];
    PushButton waistAxisDownButtons[3];
    double adjustedWaistHeight;
    double waistHeightOffset;
    DoubleSpinBox waistHeightSpin;
    CheckBox waistHeightOffsetCheck;

    struct LegWidgetSet
    {
        BodyPartCheckBox partCheck;
        PushButton fetchButton;
        PushButton axisUpButtons[3];
        PushButton axisDownButtons[3];
        CheckBox groundingCheck;
        PushButton groundingButton;
    };
    LegWidgetSet legWidgetSets[2];
    
    CheckBox legWaistCheck;
    BodyPartCheckBox legsCheck;
    PushButton legsButton;
    CheckBox relativePositionCheck;

    BodyPartCheckBox zmpCheck;
    PushButton zmpRightButton;
    PushButton zmpCenterButton;
    PushButton zmpLeftButton;
    PushButton zmpAxisUpButtons[2];
    PushButton zmpAxisDownButtons[2];

    vector<CheckBox*> bodyPartChecks;

    ButtonGroup adjustmentCoordRadioGroup;
    RadioButton globalCoordRadio;
    RadioButton localCoordRadio;

    Impl(HumanoidPoseFetchView* self);
    ~Impl();
    void setTranslationModeCaptions(PushButton upButtons[], PushButton downButtons[], int dimension);
    void setRotationModeCaptions(PushButton upButtons[], PushButton downButtons[]);
    int mergeWaistPartCheck(int bodyPart, CheckBox& check);
    void onActivated();
    void onAppKeyPressed(QKeyEvent* event);
    void onAppKeyReleased(QKeyEvent* event);
    void setSrcTime(double time);
    void setPoseSeqTime(double time);
    void onPoseRollViewTimeBarSyncToggled(PoseRollView* poseRollView, bool on);
    void updateSrcBodyItemCombo();
    bool checkIfBipedHumanoidRobot(Body* body);
    void onSrcBodyItemComboActivated(int comboIndex);
    void setSrcBodyItem(BodyItem* bodyItem, bool doUpdateCombo);
    void setBodyPartLinks(
        Body* body, LinkGroup* group, const string& prefix, vector<LinkPtr>& out_links);
    void detectRpyJoints(
        const vector<LinkPtr>& links, LinkPtr out_rpyJoints[], PushButton upButtons[], PushButton downButtons[]);
    bool checkBodyValidity();
    void onSelectedItemsChanged(ItemList<PoseSeqItem> selectedItems);
    void setPoseSeqItem(PoseSeqItem* item);
    void updateTargetKeyPoses(bool isForceNewPoseEnabled);
    void clearTargetKeyPoses();    
    void setTargetKeyPoses(const std::vector<PoseSeq::iterator>& poses);
    void updateKeyPoseInterface(BodyKeyPose* pose);
    Qt::CheckState getCombinedState(int state1, int state2);
    Qt::CheckState getCombinedState(int state1, int state2, int state3);
    Qt::CheckState getBodyPartState(BodyKeyPose* pose, const vector<LinkPtr>& partLinks, bool doCheckIkLink);
    void fetchOrCreatePoses(int bodyPart, int fetchOption = NoFetchOption);
    bool fetchPose(BodyKeyPose* pose, int bodyPart, int fetchOption);
    bool setLinkElementsToKeyPose(BodyKeyPose* pose, vector<LinkPtr>& links);
    bool fetchLegPose(
        BodyKeyPose* pose, int which,
        bool isWaistPositionIncluded, bool isWaistPositionModified, const Isometry3& T_waist, bool doFootGrounding);
    bool fetchLegJointDisplacements(
        int which, bool isLegPoseModified, const Isometry3& T_waist, const Isometry3& T_foot,
        vector<pair<int, double>>* out_displacements);
    void adjustTranslation(Isometry3& T, int axis, double sign);
    void adjustRotation(Isometry3& T, int axis, double sign);
    void adjustPosition(Isometry3& T, int axis, double sign);
    void adjustIkLinkPosition(
        Link* link, int axis, double sign, bool doClearTouching,
        std::function<bool(BodyKeyPose* pose, const Isometry3& T_link)> fetchJointDisplacements);
    void adjustWaistPosition(int axis, double sign);
    void onWaistHeightOffsetCheckToggled(bool on);
    void onWaistHeightSpinValueChanged(double value);
    void adjustHeadRotation(int axis, double sign);
    void adjustChestRotation(int axis, double sign);
    void adjustJointAngle(Link* joint, double sign);
    void adjustHandPosition(int which, int axis, double sign);
    void adjustFootPosition(int which, int axis, double sign);
    void setFootGrounding(int which, bool on);
    void adjustZmp(int axis, double sign);
    void handleBodyPartCheck(int bodyPart, const string& bodyPartName, CheckBox& checkBox, bool on);
    bool removeBodyPart(BodyKeyPose* pose, int bodyPart);
    bool removeLinkElementsFromKeyPose(BodyKeyPose* pose, vector<LinkPtr>& links);
    bool confirmKeyPoseRemovalByUncheckingBodypart(const std::string& bodyPartName, CheckBox& checkBox);
    bool storeState(Archive& archive);
    bool restoreState(const Archive& archive);
    void restoreItems(const Archive& archive);
};

}


void HumanoidPoseFetchView::initializeClass(ExtensionManager* ext)
{
    ext->viewManager().registerClass<HumanoidPoseFetchView>(N_("HumanoidPoseFetchView"), N_("Humanoid Pose Fetch"));
}


HumanoidPoseFetchView::HumanoidPoseFetchView()
{
    impl = new Impl(this);
}


HumanoidPoseFetchView::Impl::Impl(HumanoidPoseFetchView* self)
    : self(self)
{
    srcTime = 0.0;
    poseSeqTime = 0.0;
    srcBodyItem = nullptr;
    poseSeqItem = nullptr;
    
    self->setDefaultLayoutArea(BottomCenterArea);
    self->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
    self->setFocusPolicy(Qt::WheelFocus);
    auto mainVBox = new QVBoxLayout;

    auto topGrid = new QGridLayout;
    topGrid->setColumnStretch(1, 1);

    topGrid->addWidget(new QLabel(_("Source:")), 0, 0);
    srcBodyItemCombo.sigAboutToShowPopup().connect(
        [this](){ updateSrcBodyItemCombo(); });
    srcBodyItemCombo.sigActivated().connect(
        [this](int index){ onSrcBodyItemComboActivated(index); });
    topGrid->addWidget(&srcBodyItemCombo, 0, 1);

    topGrid->addWidget(new QLabel(_("Time:")), 0, 2);
    topGrid->addWidget(&srcTimeLabel, 0, 3);

    topGrid->addWidget(new QLabel(_("Pose Seq:")), 1, 0);
    targetPoseSeqLabel.setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
    topGrid->addWidget(&targetPoseSeqLabel, 1, 1);

    topGrid->addWidget(new QLabel(_("Time:")), 1, 2);
    topGrid->addWidget(&poseSeqTimeLabel, 1, 3);

    topGrid->addWidget(new QLabel(_("Pose:")), 2, 0);
    targetPoseLabel.setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
    topGrid->addWidget(&targetPoseLabel, 2, 1);
    forceNewPoseCheck.setText(_("Foce new pose"));
    forceNewPoseCheck.sigToggled().connect(
        [this](bool /* on */){ updateTargetKeyPoses(true); });
    topGrid->addWidget(&forceNewPoseCheck, 2, 2, 1, 2);
    
    mainVBox->addLayout(topGrid);
    mainVBox->addWidget(new HSeparator);

    auto grid = new QGridLayout;
    int row = 0;

    bodyPartChecks.reserve(10);

    wholeBodyCheck.sigToggled().connect(
        [this](bool on){
            handleBodyPartCheck(WholeBody, _("whole body"), wholeBodyCheck, on);
        });
    bodyPartChecks.push_back(&wholeBodyCheck);
    grid->addWidget(&wholeBodyCheck, row, 0, Qt::AlignRight);
    
    wholeBodyButton.setText(_("Whole Body"));
    wholeBodyButton.sigClicked().connect(
        [this](){ fetchOrCreatePoses(WholeBody); });
    grid->addWidget(&wholeBodyButton, row, 1, 1, 13);
    ++row;

    jointRangeLimitCheck.setText(_("Keep within upper body joint ranges"));
    jointRangeLimitCheck.setChecked(true);
    grid->addWidget(&jointRangeLimitCheck, row, 0, 1, 14, Qt::AlignCenter);
    ++row;

    upperBodyCheck.sigToggled().connect(
        [this](bool on){
            handleBodyPartCheck(UpperBody, _("upper body"), upperBodyCheck, on);
        });
    bodyPartChecks.push_back(&upperBodyCheck);
    grid->addWidget(&upperBodyCheck, row, 2, Qt::AlignRight);
    
    upperBodyButton.setText(_("Upper Body"));
    upperBodyButton.sigClicked().connect(
        [this](){
            fetchOrCreatePoses(
                mergeWaistPartCheck(UpperBody, upperBodyWaistCheck)); });
    grid->addWidget(&upperBodyButton, row, 3, 1, 9);

    upperBodyWaistCheck.setText(_("Waist"));
    upperBodyWaistCheck.setChecked(true);
    upperBodyWaistCheck.sigToggled().connect(
        [this](bool){ updateTargetKeyPoses(true); });
    grid->addWidget(&upperBodyWaistCheck, row, 12, 1, 3, Qt::AlignLeft | Qt::AlignVCenter);
    
    ++row;

    headCheck.sigToggled().connect(
        [this](bool on){
            handleBodyPartCheck(Head, _("head"), headCheck, on);
        });
    bodyPartChecks.push_back(&headCheck);
    grid->addWidget(&headCheck, row, 6, Qt::AlignHCenter | Qt::AlignBottom);
    
    headButton.setText(_("Head"));
    headButton.sigClicked().connect(
        [this](){ fetchOrCreatePoses(Head); });
    grid->addWidget(&headButton, row + 1, 6, 1, 3);

    setRotationModeCaptions(headAxisUpButtons, headAxisDownButtons);

    for(int i=0; i < 3; ++i){
        headAxisUpButtons[i].sigClicked().connect(
            [this, i](){ adjustHeadRotation(i, +1.0); });
        headAxisDownButtons[i].sigClicked().connect(
            [this, i](){ adjustHeadRotation(i, -1.0); });
    }

    grid->addWidget(&headAxisUpButtons[0], row + 2, 6);
    grid->addWidget(&headAxisDownButtons[0], row, 8);
    grid->addWidget(&headAxisUpButtons[1], row + 1, 9);
    grid->addWidget(&headAxisDownButtons[1], row + 1, 5);
    grid->addWidget(&headAxisUpButtons[2], row, 7, Qt::AlignCenter);
    grid->addWidget(&headAxisDownButtons[2], row + 2, 7, Qt::AlignCenter);

    row += 4;

    const char* armNames[] = { _("R-Arm"), _("L-Arm") };

    for(int i=0; i < 2; ++i){
        int armPart = (i == 0) ? RightArm : LeftArm;
        auto armName = armNames[i];
        auto& aw = armWidgetSets[i];

        auto check = &aw.partCheck;
        check->sigToggled().connect(
            [this, armPart, armName, check](bool on){
                handleBodyPartCheck(armPart, armName, *check, on);
            });
        bodyPartChecks.push_back(check);
        grid->addWidget(check, row - 1, i * 10 + 1, Qt::AlignHCenter | Qt::AlignBottom);

        auto button = &aw.fetchButton;
        button->setText(armName);
        button->sigClicked().connect(
            [this, armPart](){ fetchOrCreatePoses(armPart); });
        grid->addWidget(button, row, i * 10 + 1, 1, 3);

        setTranslationModeCaptions(aw.axisUpButtons, aw.axisDownButtons, 3);

        for(int j=0; j < 3; ++j){
            aw.axisUpButtons[j].sigClicked().connect(
                [this, i, j](){ adjustHandPosition(i, j, +1.0); });
            aw.axisDownButtons[j].sigClicked().connect(
                [this, i, j](){ adjustHandPosition(i, j, -1.0); });
        }

        grid->addWidget(&aw.axisUpButtons[0], row + 1, i * 10 + 1);
        grid->addWidget(&aw.axisDownButtons[0], row - 1, i * 10 + 3);
        grid->addWidget(&aw.axisUpButtons[1], row, i * 10 + 4);
        grid->addWidget(&aw.axisDownButtons[1], row, i * 10);
        grid->addWidget(&aw.axisUpButtons[2], row - 1, i * 10 + 2, Qt::AlignCenter);
        grid->addWidget(&aw.axisDownButtons[2], row + 1, i * 10 + 2, Qt::AlignCenter);
    }

    armsCheck.sigToggled().connect(
        [this](bool on){
            handleBodyPartCheck(Arms, _("arms"), armsCheck, on);
        });
    bodyPartChecks.push_back(&armsCheck);
    grid->addWidget(&armsCheck, row - 1, 7, Qt::AlignHCenter | Qt::AlignBottom);
    
    armsButton.setText(_("Arms"));
    armsButton.sigClicked().connect(
        [this](){ fetchOrCreatePoses(Arms); });
    grid->addWidget(&armsButton, row, 6, 1, 3);

    row += 2;

    chestCheck.sigToggled().connect(
        [this](bool on){
            handleBodyPartCheck(Chest, _("chest"), chestCheck, on);
        });
    bodyPartChecks.push_back(&chestCheck);
    grid->addWidget(&chestCheck, row, 6, Qt::AlignHCenter | Qt::AlignBottom);
    
    chestButton.setText(_("Chest"));
    chestButton.sigClicked().connect(
        [this](){ fetchOrCreatePoses(Chest); });
    grid->addWidget(&chestButton, row + 1, 6, 1, 3);

    setRotationModeCaptions(chestAxisUpButtons, chestAxisDownButtons);

    for(int i=0; i < 3; ++i){
        chestAxisUpButtons[i].sigClicked().connect(
            [this, i](){ adjustChestRotation(i, +1.0); });
        chestAxisDownButtons[i].sigClicked().connect(
            [this, i](){ adjustChestRotation(i, -1.0); });
    }

    grid->addWidget(&chestAxisUpButtons[0], row + 2, 6);
    grid->addWidget(&chestAxisDownButtons[0], row, 8);
    grid->addWidget(&chestAxisUpButtons[1], row + 1, 9);
    grid->addWidget(&chestAxisDownButtons[1], row + 1, 5);
    grid->addWidget(&chestAxisUpButtons[2], row, 7, Qt::AlignCenter);
    grid->addWidget(&chestAxisDownButtons[2], row + 2, 7, Qt::AlignCenter);
    
    row += 1;

    grid->addWidget(new QLabel(_("Adjustment coord. system")), row, 0, 1, 5);
    globalCoordRadio.setText(_("Global"));
    grid->addWidget(&globalCoordRadio, row + 1, 0, 1, 6);
    localCoordRadio.setText(_("Local"));
    grid->addWidget(&localCoordRadio, row + 2, 0, 1, 6);

    adjustmentCoordRadioGroup.addButton(&globalCoordRadio, GlobalCoordinate);
    adjustmentCoordRadioGroup.addButton(&localCoordRadio, LocalCoordinate);
    globalCoordRadio.setChecked(true);

    row += 2;

    waistCheck.sigToggled().connect(
        [this](bool on){
            handleBodyPartCheck(Waist, _("waist"), waistCheck, on);
        });
    bodyPartChecks.push_back(&waistCheck);
    grid->addWidget(&waistCheck, row, 6, Qt::AlignHCenter | Qt::AlignBottom);
    
    waistButton.setText(_("Waist"));
    waistButton.sigClicked().connect(
        [this](){ fetchOrCreatePoses(Waist); });
    grid->addWidget(&waistButton, row + 1, 6, 1, 3);

    setTranslationModeCaptions(waistAxisUpButtons, waistAxisDownButtons, 3);

    for(int i=0; i < 3; ++i){
        waistAxisUpButtons[i].sigClicked().connect(
            [this, i](){ adjustWaistPosition(i, +1.0); });
        waistAxisDownButtons[i].sigClicked().connect(
            [this, i](){ adjustWaistPosition(i, -1.0); });
    }

    grid->addWidget(&waistAxisUpButtons[0], row + 2, 6);
    grid->addWidget(&waistAxisDownButtons[0], row, 8);
    grid->addWidget(&waistAxisUpButtons[1], row + 1, 9);
    grid->addWidget(&waistAxisDownButtons[1], row + 1, 5);
    grid->addWidget(&waistAxisUpButtons[2], row, 7, Qt::AlignCenter);
    grid->addWidget(&waistAxisDownButtons[2], row + 2, 7, Qt::AlignCenter);

    adjustedWaistHeight = 0.8;
    waistHeightOffset = 0.0;
    
    grid->addWidget(new QLabel(_("Height")), row, 10, 1, 4, Qt::AlignCenter | Qt::AlignBottom);

    waistHeightSpin.setAlignment(Qt::AlignCenter);
    waistHeightSpin.setDecimals(3);
    waistHeightSpin.sigValueChanged().connect(
        [this](double value){ onWaistHeightSpinValueChanged(value); });
    grid->addWidget(&waistHeightSpin, row + 1, 10, 1, 4);

    waistHeightOffsetCheck.setText(_("Offset"));
    waistHeightOffsetCheck.setChecked(true);
    waistHeightOffsetCheck.sigToggled().connect(
        [this](bool on){ onWaistHeightOffsetCheckToggled(on); });
    onWaistHeightOffsetCheckToggled(true);
    grid->addWidget(&waistHeightOffsetCheck, row + 2, 10, 1, 4, Qt::AlignHCenter | Qt::AlignTop);

    row += 3;

    const char* legNames[] = { _("R-Leg"), _("L-Leg") };

    for(int i=0; i < 2; ++i){
        int legPart = (i == 0) ? RightLeg : LeftLeg;
        auto legName = legNames[i];
        auto& lw = legWidgetSets[i];

        auto legCheck = &lw.partCheck;
        legCheck->sigToggled().connect(
            [this, legPart, legName, legCheck](bool on){
                handleBodyPartCheck(legPart, legName, *legCheck, on);
            });
        bodyPartChecks.push_back(legCheck);
        grid->addWidget(legCheck, row, i * 10 + 1, Qt::AlignHCenter | Qt::AlignBottom);

        auto fetchButton = &lw.fetchButton;
        fetchButton->setText(legName);
        fetchButton->sigClicked().connect(
            [this, legPart](){
                fetchOrCreatePoses(mergeWaistPartCheck(legPart, legWaistCheck));
            });
        grid->addWidget(fetchButton, row + 1, i * 10 + 1, 1, 3);

        setTranslationModeCaptions(lw.axisUpButtons, lw.axisDownButtons, 3);

        for(int j=0; j < 3; ++j){
            lw.axisUpButtons[j].sigClicked().connect(
                [this, i, j](){ adjustFootPosition(i, j, +1.0); });
            lw.axisDownButtons[j].sigClicked().connect(
                [this, i, j](){ adjustFootPosition(i, j, -1.0); });
        }

        grid->addWidget(&lw.axisUpButtons[0], row + 2, i * 10 + 1);
        grid->addWidget(&lw.axisDownButtons[0], row, i * 10 + 3);
        grid->addWidget(&lw.axisUpButtons[1], row + 1, i * 10 + 4);
        grid->addWidget(&lw.axisDownButtons[1], row + 1, i * 10);
        grid->addWidget(&lw.axisUpButtons[2], row, i * 10 + 2, Qt::AlignCenter);
        grid->addWidget(&lw.axisDownButtons[2], row + 2, i * 10 + 2, Qt::AlignCenter);
        
        auto groundingCheck = &lw.groundingCheck;
        bodyPartChecks.push_back(groundingCheck);
        groundingCheck->sigToggled().connect(
            [this, i](bool on){
                setFootGrounding(i, on);
            });
        grid->addWidget(groundingCheck, row + 3, i * 10, Qt::AlignRight | Qt::AlignVCenter);
        
        auto groundingButton = &lw.groundingButton;
        groundingButton->setText(_("Ground"));
        groundingButton->sigClicked().connect(
            [this, legPart](){
                fetchOrCreatePoses(mergeWaistPartCheck(legPart, legWaistCheck), FootGrounding); });
        grid->addWidget(groundingButton, row + 3, i * 10 + 1, 1, 3);
    }

    legsCheck.sigToggled().connect(
        [this](bool on){
            handleBodyPartCheck(mergeWaistPartCheck(Legs, legWaistCheck), _("legs"), legsCheck, on);
        });
    bodyPartChecks.push_back(&legsCheck);
    grid->addWidget(&legsCheck, row, 7, Qt::AlignHCenter | Qt::AlignBottom);
    
    legsButton.setText(_("Legs"));
    legsButton.sigClicked().connect(
        [this](){ fetchOrCreatePoses(mergeWaistPartCheck(Legs, legWaistCheck)); });
    grid->addWidget(&legsButton, row + 1, 6, 1, 3);

    legWaistCheck.setText(_("Waist"));
    legWaistCheck.setChecked(true);
    legWaistCheck.sigToggled().connect(
        [this](bool){ updateTargetKeyPoses(true); });
    grid->addWidget(&legWaistCheck, row + 2, 6, 1, 3, Qt::AlignCenter);

    row += 4;

    auto hbox = new QHBoxLayout;
    zmpCheck.sigToggled().connect(
        [this](bool on){
            handleBodyPartCheck(Zmp, _("ZMP"), zmpCheck, on);
        });
    bodyPartChecks.push_back(&zmpCheck);
    hbox->addWidget(&zmpCheck);
    hbox->addWidget(new QLabel(_("ZMP")));
    grid->addLayout(hbox, row, 6, 1, 2, Qt::AlignRight | Qt::AlignVCenter);
    
    zmpRightButton.setText(_("R"));
    zmpRightButton.sigClicked().connect(
        [this](){ fetchOrCreatePoses(Zmp, ZmpAtRightFoot); });
    grid->addWidget(&zmpRightButton, row + 1, 2, 1, 3, Qt::AlignRight | Qt::AlignVCenter);
    
    zmpCenterButton.setText(_("Center"));
    zmpCenterButton.sigClicked().connect(
        [this](){ fetchOrCreatePoses(Zmp, ZmpAtCenterOfBothFeet); });
    grid->addWidget(&zmpCenterButton, row + 1, 6, 1, 3);
    
    zmpLeftButton.setText(_("L"));
    zmpLeftButton.sigClicked().connect(
        [this](){ fetchOrCreatePoses(Zmp, ZmpAtLeftFoot); });
    grid->addWidget(&zmpLeftButton, row + 1, 10, 1, 3, Qt::AlignLeft | Qt::AlignVCenter);

    setTranslationModeCaptions(zmpAxisUpButtons, zmpAxisDownButtons, 2);

    for(int i=0; i < 2; ++i){
        zmpAxisUpButtons[i].sigClicked().connect(
            [this, i](){ adjustZmp(i, +1.0); });
        zmpAxisDownButtons[i].sigClicked().connect(
            [this, i](){ adjustZmp(i, -1.0); });
    }

    grid->addWidget(&zmpAxisUpButtons[0], row + 2, 6);
    grid->addWidget(&zmpAxisDownButtons[0], row, 8);
    grid->addWidget(&zmpAxisUpButtons[1], row + 1, 9);
    grid->addWidget(&zmpAxisDownButtons[1], row + 1, 5);

    // Not implemented yet
    //relativePositionCheck.setText(_("Relative positioning for the waist and feet"));
    //relativePositionCheck.setChecked(true);
    //grid->addWidget(&relativePositionCheck, row, 1, 1, 9, Qt::AlignCenter);

    auto mainHBox = new QHBoxLayout;
    //mainHBox->addStretch();
    mainHBox->addLayout(grid);
    //mainHBox->addStretch();
    mainVBox->addLayout(mainHBox);
    
    mainVBox->addStretch();
    
    self->setLayout(mainVBox, 1.0);
    
    setSrcBodyItem(nullptr, false);
}


HumanoidPoseFetchView::~HumanoidPoseFetchView()
{
    delete impl;
}


HumanoidPoseFetchView::Impl::~Impl()
{

}


void HumanoidPoseFetchView::Impl::setTranslationModeCaptions
(PushButton upButtons[], PushButton downButtons[], int dimension)
{
    static QIcon frontArrow(":/PoseSeq/icon/front-arrow.svg");
    static QIcon backArrow(":/PoseSeq/icon/back-arrow.svg");
    static QIcon rightArrow(":/PoseSeq/icon/right-arrow.svg");
    static QIcon leftArrow(":/PoseSeq/icon/left-arrow.svg");
    static QIcon upArrow(":/PoseSeq/icon/up-arrow.svg");
    static QIcon downArrow(":/PoseSeq/icon/down-arrow.svg");

    upButtons[0].setIcon(frontArrow);
    upButtons[0].setToolTip(_("Move toward the X-axis positive direction"));
    downButtons[0].setIcon(backArrow);
    downButtons[0].setToolTip(_("Move toward the X-axis negative direction"));
    upButtons[1].setIcon(rightArrow);
    upButtons[1].setToolTip(_("Move toward the Y-axis positive direction"));
    downButtons[1].setIcon(leftArrow);
    downButtons[1].setToolTip(_("Move toward the Y-axis negative direction"));

    if(dimension == 3){
        upButtons[2].setIcon(upArrow);
        upButtons[2].setToolTip(_("Move toward the Z-axis positive direction"));
        downButtons[2].setIcon(downArrow);
        downButtons[2].setToolTip(_("Move toward the Z-axis negative direction"));
    }
}


void HumanoidPoseFetchView::Impl::setRotationModeCaptions(PushButton upButtons[], PushButton downButtons[])
{
    static QIcon frontArrowRotation(":/PoseSeq/icon/front-arrow-rotation.svg");
    static QIcon backArrowRotation(":/PoseSeq/icon/back-arrow-rotation.svg");
    static QIcon rightArrowRotation(":/PoseSeq/icon/right-arrow-rotation.svg");
    static QIcon leftArrowRotation(":/PoseSeq/icon/left-arrow-rotation.svg");
    static QIcon upArrowRotation(":/PoseSeq/icon/up-arrow-rotation.svg");
    static QIcon downArrowRotation(":/PoseSeq/icon/down-arrow-rotation.svg");
    
    upButtons[0].setIcon(frontArrowRotation);
    upButtons[0].setToolTip(_("Rotate in the positive direction around the X-axis"));
    downButtons[0].setIcon(backArrowRotation);
    downButtons[0].setToolTip(_("Rotate in the negative direction around the X-axis"));
    upButtons[1].setIcon(rightArrowRotation);
    upButtons[1].setToolTip(_("Rotate in the positive direction around the Y-axis"));
    downButtons[1].setIcon(leftArrowRotation);
    downButtons[1].setToolTip(_("Rotate in the negative direction around the Y-axis"));
    upButtons[2].setIcon(upArrowRotation);
    upButtons[2].setToolTip(_("Rotate in the positive direction around the Z-axis"));
    downButtons[2].setIcon(downArrowRotation);
    downButtons[2].setToolTip(_("Rotate in the negative direction around the Z-axis"));
}


int HumanoidPoseFetchView::Impl::mergeWaistPartCheck(int bodyPart, CheckBox& check)
{
    if(check.isChecked()){
        bodyPart |= Waist;
    }
    return bodyPart;
}


void HumanoidPoseFetchView::onActivated()
{
    impl->onActivated();
}


void HumanoidPoseFetchView::Impl::onActivated()
{
    auto timeBar = TimeBar::instance();
    auto poseRollView = ViewManager::getOrCreateView<PoseRollView>();
    
    if(baseConnections.empty()){
        baseConnections.add(
            RootItem::instance()->sigSelectedItemsChanged().connect(
                [this](const ItemList<>& selectedItems){
                    onSelectedItemsChanged(selectedItems);
                }));

        baseConnections.add(
            timeBar->sigTimeChanged().connect(
                [this](double time){
                    setSrcTime(time);
                    if(!poseRollViewTimeConnection.connected()){
                        setPoseSeqTime(time);
                    }
                    return false;
                }));

        baseConnections.add(
            poseRollView->sigTimeBarSyncToggled().connect(
                [this, poseRollView](bool on){
                    onPoseRollViewTimeBarSyncToggled(poseRollView, on);
                }));
    }

    setSrcTime(timeBar->time());
    onPoseRollViewTimeBarSyncToggled(poseRollView, poseRollView->isTimeBarSyncEnabled());

    appConnections.add(
        AppUtil::sigKeyPressed().connect(
            [this](QKeyEvent* event){ onAppKeyPressed(event); }));
    appConnections.add(
        AppUtil::sigKeyReleased().connect(
            [this](QKeyEvent* event){ onAppKeyReleased(event); }));
    
    isRotationAdjustmentMode = false;
    isSmallStepAdjustmentMode = false;
}


void HumanoidPoseFetchView::onDeactivated()
{
    impl->baseConnections.disconnect();
    impl->appConnections.disconnect();
}


void HumanoidPoseFetchView::Impl::onAppKeyPressed(QKeyEvent* event)
{
    if(event->key() == Qt::Key_Control){
        setRotationModeCaptions(waistAxisUpButtons, waistAxisDownButtons);
        for(int i=0; i < 2; ++i){
            auto& aw = armWidgetSets[i];
            setRotationModeCaptions(aw.axisUpButtons, aw.axisDownButtons);
            auto& lw = legWidgetSets[i];
            setRotationModeCaptions(lw.axisUpButtons, lw.axisDownButtons);
        }
        isRotationAdjustmentMode = true;

    } else if(event->key() == Qt::Key_Shift){
        isSmallStepAdjustmentMode = true;

    } else if(event->key() == Qt::Key_Alt){
        if(forceNewPoseCheck.isChecked()){
            updateTargetKeyPoses(false);
        } else {
            clearTargetKeyPoses();
        }
    }
}


void HumanoidPoseFetchView::Impl::onAppKeyReleased(QKeyEvent* event)
{
    if(event->key() == Qt::Key_Control){
        setTranslationModeCaptions(waistAxisUpButtons, waistAxisDownButtons, 3);
        for(int i=0; i < 2; ++i){
            auto& aw = armWidgetSets[i];
            setTranslationModeCaptions(aw.axisUpButtons, aw.axisDownButtons, 3);
            auto& lw = legWidgetSets[i];
            setTranslationModeCaptions(lw.axisUpButtons, lw.axisDownButtons, 3);
        }
        isRotationAdjustmentMode = false;

    } else if(event->key() == Qt::Key_Shift){
        isSmallStepAdjustmentMode = false;

    } else if(event->key() == Qt::Key_Alt){
        if(forceNewPoseCheck.isChecked()){
            clearTargetKeyPoses();
        } else {
            updateTargetKeyPoses(false);
        }
    }
}


void HumanoidPoseFetchView::Impl::setSrcTime(double time)
{
    srcTime = time;
    srcTimeLabel.setText(QString::number(time, 'f', 3));
}


void HumanoidPoseFetchView::Impl::setPoseSeqTime(double time)
{
    poseSeqTime = time;
    poseSeqTimeLabel.setText(QString::number(time, 'f', 3));
}


void HumanoidPoseFetchView::Impl::onPoseRollViewTimeBarSyncToggled(PoseRollView* poseRollView, bool on)
{
    if(on){
        poseRollViewTimeConnection.disconnect();
        setPoseSeqTime(srcTime);
    } else {
        poseRollViewTimeConnection =
            poseRollView->sigCurrentTimeChanged().connect(
                [this](double time){ setPoseSeqTime(time); });
        setPoseSeqTime(poseRollView->currentTime());
    }
}


void HumanoidPoseFetchView::Impl::updateSrcBodyItemCombo()
{
    srcBodyItemCombo.blockSignals(true);
    srcBodyItemCombo.clear();
    srcBodyItemCandidates.clear();
    map<string, int> nameCounterMap;
    int currentIndex = -1;

    for(auto& bodyItem : RootItem::instance()->descendantItems<BodyItem>()){
        bool isHumanoid = false;
        if(bodyItem == srcBodyItem){
            isHumanoid = true;
            currentIndex = srcBodyItemCombo.count();
        } else {
            isHumanoid = checkIfBipedHumanoidRobot(bodyItem->body());
        }
        if(isHumanoid){
            string name = bodyItem->displayName();
            int counter = 1;
            auto inserted = nameCounterMap.insert(make_pair(name, counter));
            if(!inserted.second){
                counter = ++inserted.first->second;
            }
            if(counter == 2){
                int index = srcBodyItemCombo.findText(name.c_str());
                srcBodyItemCombo.setItemText(index, QString("%1 (1)").arg(name.c_str()));
            }
            if(counter >= 2){
                name = format("{0} ({1})", name, counter);
            }
            srcBodyItemCombo.addItem(name.c_str());
            srcBodyItemCandidates.push_back(bodyItem);
        }
    }
    if(currentIndex >= 0){
        srcBodyItemCombo.setCurrentIndex(currentIndex);
    }
    
    srcBodyItemCombo.blockSignals(false);
}


bool HumanoidPoseFetchView::Impl::checkIfBipedHumanoidRobot(Body* body)
{
    if(body->numJoints() >= 20){
        LeggedBodyHelper legged(body);
        if(legged.numFeet() == 2){
            return true;
        }
    }
    return false;
}


void HumanoidPoseFetchView::Impl::onSrcBodyItemComboActivated(int comboIndex)
{
    setSrcBodyItem(srcBodyItemCandidates[comboIndex], false);
}


void HumanoidPoseFetchView::Impl::setSrcBodyItem(BodyItem* bodyItem, bool doUpdateCombo)
{
    if(bodyItem != srcBodyItem){

        srcBodyItem = bodyItem;

        waistLink.reset();
        headLinks.clear();
        chestLinks.clear();

        for(int i=0; i < 3; ++i){
            headRpyJoints[i].reset();
            chestRpyJoints[i].reset();
        }
            
        for(int i=0; i < 2; ++i){
            armLinkSets[i].clear();
            armJointPaths[i].reset();
            legLinkSets[i].clear();
            legJointPaths[i].reset();
            waistToFootJointPaths[i].reset();
        }
        
        if(srcBodyItem){
            auto body = srcBodyItem->body();
            leggedBody = new LeggedBodyHelper(body);
            
            waistLink = body->link("WAIST");
            LinkGroupPtr linkGroup = LinkGroup::create(srcBodyItem->body());

            if(auto upperBodyGroup = linkGroup->findSubGroup("UPPER_BODY")){
                if(auto headGroup = upperBodyGroup->findSubGroup("HEAD")){
                    setBodyPartLinks(body, headGroup, "", headLinks);
                    detectRpyJoints(headLinks, headRpyJoints, headAxisUpButtons, headAxisDownButtons);
                }
                const char* armGroupNames[2] = { "R_ARM", "L_ARM" };
                for(int i=0; i < 2; ++i){
                    if(auto armGroup = upperBodyGroup->findSubGroup(armGroupNames[i])){
                        setBodyPartLinks(body, armGroup, "", armLinkSets[i]);
                    }
                }
                if(auto chestGroup = upperBodyGroup->findSubGroup("CHEST")){
                    setBodyPartLinks(body, chestGroup, "", chestLinks);
                    detectRpyJoints(chestLinks, chestRpyJoints, chestAxisUpButtons, chestAxisDownButtons);
                }
            }
            const char* legGroupNames[2] = { "R_LEG", "L_LEG" };
            for(int i=0; i < 2; ++i){
                if(auto legGroup = linkGroup->findSubGroup(legGroupNames[i])){
                    setBodyPartLinks(body, legGroup, "", legLinkSets[i]);

                    // Find the foot link
                    for(int j=0; j < leggedBody->numFeet(); ++j){
                        for(auto& link : legLinkSets[i]){
                            if(link == leggedBody->footLink(j)){
                                waistToFootJointPaths[i] = JointPath::getCustomPath(waistLink, link);
                            }
                        }
                    }
                }
            }
        }
        
        if(doUpdateCombo){
            updateSrcBodyItemCombo();
        }
    }
}


void HumanoidPoseFetchView::Impl::setBodyPartLinks
(Body* body, LinkGroup* group, const string& prefix, vector<LinkPtr>& out_links)
{
    out_links.clear();
    int n = group->numElements();
    for(int i=0; i < n; ++i){
        if(group->checkIfLink(i)){
            if(auto link = body->link(group->linkIndex(i))){
                if(!prefix.empty()){
                    if(link->name().find_first_of(prefix) != 0){
                        continue;
                    }
                }
                out_links.push_back(link);
            }
        }
    }
}


void HumanoidPoseFetchView::Impl::detectRpyJoints
(const vector<LinkPtr>& links, LinkPtr out_rpyJoints[], PushButton upButtons[], PushButton downButtons[])
{
    for(int i=0; i < 3; ++i){
        out_rpyJoints[i].reset();
    }

    for(auto& link : links){
        if(link->isRevoluteJoint()){
            Vector3 a = link->jointAxis();
            if(a.isApprox(Vector3::UnitX())){
                out_rpyJoints[0] = link;
            } else if(a.isApprox(Vector3::UnitY())){
                out_rpyJoints[1] = link;
            } else if(a.isApprox(Vector3::UnitZ())){
                out_rpyJoints[2] = link;
            }
        }
    }

    for(int i=0; i < 3; ++i){
        bool on = out_rpyJoints[i] != nullptr;
        upButtons[i].setEnabled(on);
        downButtons[i].setEnabled(on);
    }
}


bool HumanoidPoseFetchView::Impl::checkBodyValidity()
{
    if(!srcBodyItem){
        return false;
    }
    if(leggedBody->numFeet() != 2){
        return false;
    }
    if(!waistLink){
        return false;
    }
    for(int i=0; i < 2; ++i){
        if(armLinkSets[i].empty()){
            return false;
        }
        if(legLinkSets[i].empty()){
            return false;
        }
        if(!waistToFootJointPaths[i]){
            return false;
        }
    }
    return true;
}


void HumanoidPoseFetchView::Impl::onSelectedItemsChanged(ItemList<PoseSeqItem> selectedItems)
{
    if(auto item = selectedItems.toSingle()){
        setPoseSeqItem(item);
    }
}


void HumanoidPoseFetchView::Impl::setPoseSeqItem(PoseSeqItem* item)
{
    if(item != poseSeqItem){
        poseSeqItem = item;
        if(!poseSeqItem){
            targetPoseIters.clear();
            targetPoseSeqLabel.setText("---");
            poseSeqConnections.disconnect();
        } else {
            auto poseSeq = poseSeqItem->poseSeq();
            currentPoseIter = poseSeq->end();
            targetPoseSeqLabel.setText(poseSeqItem->displayName().c_str());
            poseSeqConnections.add(
                poseSeqItem->sigPoseSelectionChanged().connect(
                    [this](const std::vector<PoseSeq::iterator>& /* selected */){
                        updateTargetKeyPoses(true);
                    }));
            poseSeqConnections.add(
                poseSeq->sigPoseAboutToBeRemoved().connect(
                    [this, poseSeq](PoseSeq::iterator it, bool isMoving){
                        if(!isMoving){
                            if(it == currentPoseIter){
                                currentPoseIter = poseSeq->end();
                            }
                        }
                    }));
            updateTargetKeyPoses(true);
        }
    }
}


void HumanoidPoseFetchView::Impl::updateTargetKeyPoses(bool isForceNewPoseEnabled)
{
    bool doForceNewPose = isForceNewPoseEnabled && forceNewPoseCheck.isChecked();
    if(poseSeqItem && !doForceNewPose){
        setTargetKeyPoses(poseSeqItem->selectedPoses());
    } else {
        clearTargetKeyPoses();
    }
}


void HumanoidPoseFetchView::Impl::clearTargetKeyPoses()
{
    std::vector<PoseSeq::iterator> emptyPoseIters;
    setTargetKeyPoses(emptyPoseIters);
}


void HumanoidPoseFetchView::Impl::setTargetKeyPoses(const std::vector<PoseSeq::iterator>& poseIters)
{
    if(poseSeqItem){
        for(auto& it : targetPoseIters){
            if(currentPoseIter == it){
                currentPoseIter = poseSeqItem->poseSeq()->end();
                break;
            }
        }
    }
    
    targetPoseIters.clear();

    BodyKeyPose* firstKeyPose = nullptr;
    PoseSeq::iterator firstIter;
    for(auto& it : poseIters){
        if(auto bkPose = it->get<BodyKeyPose>()){
            targetPoseIters.push_back(it);
            if(!firstKeyPose){
                firstKeyPose = bkPose;
                firstIter = it;
            }
            currentPoseIter = it;
        }
    }

    if(targetPoseIters.empty()){
        targetPoseLabel.setText(_("New pose"));
    } else {
        string id = getPoseId(firstIter, firstKeyPose);
        if(targetPoseIters.size() == 1){
            targetPoseLabel.setText(format("{0}", id).c_str());
        } else {
            targetPoseLabel.setText(format(_("{0} ..."), id).c_str());
        }
    }

    updateKeyPoseInterface(firstKeyPose);
}


void HumanoidPoseFetchView::Impl::updateKeyPoseInterface(BodyKeyPose* pose)
{
    bool isExistingPose = (pose != nullptr);

    if(!pose){
        for(auto& check : bodyPartChecks){
            check->blockSignals(true);
            check->setChecked(false);
            check->setEnabled(false);
            check->blockSignals(false);
        }
        return;
    }

    for(auto& check : bodyPartChecks){
        check->blockSignals(true);
    }

    Qt::CheckState waistState = pose->ikLinkInfo(waistLink->index()) ? Qt::Checked : Qt::Unchecked;
    waistCheck.setCheckState(waistState);
    Qt::CheckState headState = getBodyPartState(pose, headLinks, false);
    headCheck.setCheckState(headState);
    Qt::CheckState chestState = getBodyPartState(pose, chestLinks, false);
    chestCheck.setCheckState(chestState);

    Qt::CheckState armStates[2];
    Qt::CheckState legStates[2];
    bool footIk[2];

    for(int i=0; i < 2; ++i){
        armStates[i] = getBodyPartState(pose, armLinkSets[i], false);
        armWidgetSets[i].partCheck.setCheckState(armStates[i]);

        auto legState = getBodyPartState(pose, legLinkSets[i], true);
        if(legState != Qt::Unchecked && legWaistCheck.isChecked()){
            legState = getCombinedState(legState, waistState);
        }
        legWidgetSets[i].partCheck.setCheckState(legState);
        legStates[i] = legState;

        bool isTouching = false;
        footIk[i] = false;
        if(auto footLink = waistToFootJointPaths[i]->endLink()){
            if(auto footInfo = pose->ikLinkInfo(footLink->index())){
                footIk[i] = true;
                isTouching = footInfo->isTouching();
            }
        }
        legWidgetSets[i].groundingCheck.setChecked(isTouching);
    }
    Qt::CheckState armsState = getCombinedState(armStates[0], armStates[1]);
    armsCheck.setCheckState(armsState);

    Qt::CheckState legsState = getCombinedState(legStates[0], legStates[1]);
    legsCheck.setCheckState(legsState);

    Qt::CheckState upperBodyState = getCombinedState(armsState, headState, chestState);
    if(upperBodyState != Qt::Unchecked && upperBodyWaistCheck.isChecked()){
        upperBodyState = getCombinedState(upperBodyState, waistState);
    }
    upperBodyCheck.setCheckState(upperBodyState);

    Qt::CheckState wholeBodyState = getCombinedState(upperBodyState, legsState);
    wholeBodyState = getCombinedState(wholeBodyState, waistState);
    wholeBodyCheck.setCheckState(wholeBodyState);

    zmpCheck.setCheckState(pose->isZmpValid() ? Qt::Checked : Qt::Unchecked);

    for(auto& check : bodyPartChecks){
        check->setEnabled(check->checkState() != Qt::Unchecked);
        check->blockSignals(false);
    }
}


Qt::CheckState HumanoidPoseFetchView::Impl::getCombinedState(int state1, int state2)
{
    if(state1 == Qt::Checked){
        if(state2 == Qt::Checked){
            return Qt::Checked;
        } else {
            return Qt::PartiallyChecked;
        }
    } else if(state1 == Qt::PartiallyChecked){
        return Qt::PartiallyChecked;
    } else {
        if(state2 == Qt::Checked){
            return Qt::PartiallyChecked;
        } else {
            return Qt::Unchecked;
        }
    }
}


Qt::CheckState HumanoidPoseFetchView::Impl::getCombinedState(int state1, int state2, int state3)
{
    return getCombinedState(state1, getCombinedState(state2, state3));
}


Qt::CheckState HumanoidPoseFetchView::Impl::getBodyPartState
(BodyKeyPose* pose, const vector<LinkPtr>& partLinks, bool doCheckIkLink)
{
    bool hasPartLinks = false;
    bool hasMissingLinks = false;

    for(auto& link : partLinks){
        if(doCheckIkLink){
            if(pose->ikLinkInfo(link->index())){
                hasPartLinks = true;
                hasMissingLinks = false;
                break;
            }
        }
        if(pose->isJointValid(link->jointId())){
            hasPartLinks = true;
        } else {
            hasMissingLinks = true;
        }
    }

    if(hasPartLinks){
        if(!hasMissingLinks){
            return Qt::Checked;
        } else {
            return Qt::PartiallyChecked;
        }
    } else {
        return Qt::Unchecked;
    }
}


void HumanoidPoseFetchView::Impl::fetchOrCreatePoses(int bodyPart, int fetchOption)
{
    if(!poseSeqItem || !checkBodyValidity()){
        showErrorDialog(_("Not ready to fetch poses"));
        return;
    }

    bool modified = false;
    auto poseSeq = poseSeqItem->poseSeq();
    auto block = poseSeqConnections.scopedBlock();

    if(targetPoseIters.empty()){
        BodyKeyPosePtr pose = new BodyKeyPose;
        if(!fetchPose(pose, bodyPart, fetchOption)){
            showErrorDialog(
                format(_("The current pose of {0} cannot be fetched as a new key pose."),
                       srcBodyItem->displayName()));
        } else {
            poseSeqItem->beginEditing();
            auto it = poseSeq->insert(currentPoseIter, poseSeqTime, pose, true);
            poseSeqItem->endEditing();
            poseSeqItem->clearPoseSelection(true);
            poseSeqItem->selectPose(it, true);
            currentPoseIter = it;
            InfoBar::instance()->notify(
                format(_("The current pose of {0} has been fetched as a new key pose of {1}."),
                       srcBodyItem->displayName(), poseSeqItem->displayName()));
            modified = true;
        }
    } else {
        bool modified2 = false;
        poseSeqItem->beginEditing();
        for(auto& it : targetPoseIters){
            if(auto pose = it->get<BodyKeyPose>()){
                poseSeq->beginPoseModification(it);
                if(!fetchPose(pose, bodyPart, fetchOption)){
                    showErrorDialog(
                        format(_("The current pose of {0} cannot be fetched."),
                               srcBodyItem->displayName()));
                    break;
                }
                poseSeq->endPoseModification(it);
                currentPoseIter = it;
                modified2 = true;
            }
        }
        poseSeqItem->endEditing(modified2);
        if(modified2){
            modified = true;
        }
        InfoBar::instance()->notify(
            format(_("The current pose of {0} has been fetched in the selected key pose(s) of {1}."),
                   srcBodyItem->displayName(), poseSeqItem->displayName()));
    }

    if(modified){
        updateTargetKeyPoses(true);
    }
}


bool HumanoidPoseFetchView::Impl::fetchPose(BodyKeyPose* pose, int bodyPart, int fetchOption)
{
    bool fetched = false;
    bool failed = false;

    bool isWaistPositionIncluded = false;
    bool isWaistPositionModified = false;
    Isometry3 T_waist = waistLink->position();
    
    if(bodyPart & Waist){
        double z0 = T_waist.translation().z();
        if(waistHeightOffsetCheck.isChecked()){
            T_waist.translation().z() += waistHeightOffset;
        } else {
            T_waist.translation().z() = adjustedWaistHeight;
        }
        if(T_waist.translation().z() != z0){
            isWaistPositionModified = true;
        }
        pose->setBaseLink(waistLink->index(), T_waist);
        isWaistPositionIncluded = true;
        fetched = true;
    }
    if(bodyPart & Head){
        if(setLinkElementsToKeyPose(pose, headLinks)){
            fetched = true;
        }
    }
    if(bodyPart & Chest){
        if(setLinkElementsToKeyPose(pose, chestLinks)){
            fetched = true;
        }
    }

    int armParts[2] = { RightArm, LeftArm };
    int legParts[2] = { RightLeg, LeftLeg };

    for(int i=0; i < 2; ++i){
        if(bodyPart & armParts[i]){
            if(setLinkElementsToKeyPose(pose, armLinkSets[i])){
                fetched = true;
            }
        }
        if(bodyPart & legParts[i]){
            bool doFootGrounding = (fetchOption == FootGrounding);
            if(fetchLegPose(pose, i, isWaistPositionIncluded, isWaistPositionModified, T_waist, doFootGrounding)){
                fetched = true;
            } else {
                failed = true;
            }
        }
    }

    if(bodyPart & Zmp){
        Vector3 zmp;
        Vector3 rpos = leggedBody->centerOfSole(0);
        Vector3 lpos = leggedBody->centerOfSole(1);
        if(fetchOption == ZmpAtRightFoot){
            zmp = rpos;
        } else if(fetchOption == ZmpAtLeftFoot){
            zmp = lpos;
        } else {
            zmp = (rpos + lpos) / 2.0;
        }
        zmp.z() = 0.0;
        pose->setZmp(zmp);
        fetched = true;
    }

    return fetched && !failed;
}


bool HumanoidPoseFetchView::Impl::setLinkElementsToKeyPose(BodyKeyPose* pose, vector<LinkPtr>& links)
{
    bool fetched = false;
    for(auto& link : links){
        if(pose->removeIkLink(link->index())){
            fetched = true;
        }
        if(link->jointId() >= 0){
            double q = link->q();
            if(jointRangeLimitCheck.isChecked()){
                q = stdx::clamp(q, link->q_lower(), link->q_upper());
            }
            pose->setJointDisplacement(link->jointId(), q);
            fetched = true;
        }
    }
    return fetched;
}


bool HumanoidPoseFetchView::Impl::fetchLegPose
(BodyKeyPose* pose, int which,
 bool isWaistPositionIncluded, bool isWaistPositionModified, const Isometry3& T_waist,
 bool doFootGrounding)
{
    Link* footLink = waistToFootJointPaths[which]->endLink();
    constexpr double MarginToHaveContactDepth = 0.001;
    double ankleHeight = -leggedBody->centerOfSoleLocal(which).z() - MarginToHaveContactDepth;

    auto footInfo = pose->getOrCreateIkLink(footLink->index());
    
    Isometry3 T_foot = footLink->position();
    if(doFootGrounding){
        double diff = abs(T_foot.translation().z() - ankleHeight);
        if(diff < 0.1){
            double yaw = rpyFromRot(T_foot.linear()).z();
            T_foot.linear() = Matrix3(AngleAxis(yaw, Vector3::UnitZ()));
            T_foot.translation().z() = ankleHeight;
        }
    }

    bool failed = false;
    if(isWaistPositionIncluded){
        bool isLegPoseModified = isWaistPositionModified || doFootGrounding;
        if(!fetchLegJointDisplacements(which, isLegPoseModified, T_waist, T_foot, nullptr)){
            failed = true;
        }
    }

    if(!failed){
        footInfo->setPosition(T_foot);
        footInfo->setTouching(doFootGrounding);
    }

    return !failed;
}


/**
   \param out_displacements The array of a joint id and displacement value pair.
*/
bool HumanoidPoseFetchView::Impl::fetchLegJointDisplacements
(int which, bool isLegPoseModified, const Isometry3& T_waist, const Isometry3& T_foot,
 vector<pair<int, double>>* out_displacements)
{
    bool failed = false;

    auto& legLinks = legLinkSets[which];
    auto& waistToFoot = waistToFootJointPaths[which];
    Isometry3 T_waist_org;
    vector<double> q0;

    if(isLegPoseModified){
        // Store the original pose
        T_waist_org = waistLink->T();
        int nj = waistToFoot->numJoints();
        q0.resize(nj);
        for(int i=0; i < nj; ++i){
            q0[i] = waistToFoot->joint(i)->q();
        }
        // Calculate the joint angles to fit the sole to the fllor
        waistLink->setPosition(T_waist);
        if(!waistToFoot->calcInverseKinematics(T_foot)){
            failed = true;
        }
    }
    if(out_displacements && !failed){
        for(auto& link : legLinks){
            out_displacements->emplace_back(link->jointId(), link->q());
        }
    }
    if(isLegPoseModified){
        // restore the original pose
        waistLink->setPosition(T_waist_org);
        for(int i=0; i < waistToFoot->numJoints(); ++i){
            waistToFoot->joint(i)->q() = q0[i];
        }
        auto& path = legJointPaths[which];
        if(!path){
            path = JointPath::getCustomPath(waistLink, legLinks.back());
        }
        path->calcForwardKinematics();
    }

    return !failed;
}


void HumanoidPoseFetchView::Impl::adjustTranslation(Isometry3& T, int axis, double sign)
{
    double diff = sign * (isSmallStepAdjustmentMode ? 0.001 : 0.01);
    
    if(adjustmentCoordRadioGroup.checkedId() == GlobalCoordinate){
        T.translation()[axis] += diff;
        
    } else { // Local coordinate system
        T.translation() += diff * T.linear().col(axis);
    }
}


void HumanoidPoseFetchView::Impl::adjustRotation(Isometry3& T, int axis, double sign)
{
    double dTheta = sign * radian(isSmallStepAdjustmentMode ? 1.0 : 5.0);

    if(adjustmentCoordRadioGroup.checkedId() == GlobalCoordinate){
        T.linear() = AngleAxis(dTheta, Vector3::Unit(axis)) * T.linear();

    } else { // Local coordinate system
        T.linear() = AngleAxis(dTheta, T.linear().col(axis)) * T.linear();
    }
}


void HumanoidPoseFetchView::Impl::adjustPosition(Isometry3& T, int axis, double sign)
{
    if(isRotationAdjustmentMode){
        adjustRotation(T, axis, sign);
    } else {
        adjustTranslation(T, axis, sign);
    }
}


void HumanoidPoseFetchView::Impl::adjustIkLinkPosition
(Link* link, int axis, double sign, bool doClearTouching,
 std::function<bool(BodyKeyPose* pose, const Isometry3& T_link)> fetchJointDisplacements)
{
    if(!poseSeqItem || !checkBodyValidity() || targetPoseIters.empty()){
        showErrorDialog(_("Not ready to adjust the link position."));
        return;
    }

    struct LinkAdjustmentBuf {
        PoseSeq::iterator it;
        BodyKeyPose* pose;
        BodyKeyPose::LinkInfo* linkInfo;
        Isometry3 T_link;
    };
    vector<LinkAdjustmentBuf, Eigen::aligned_allocator<LinkAdjustmentBuf>> linkAdjustmentBufs;
    linkAdjustmentBufs.reserve(targetPoseIters.size());

    auto poseSeq = poseSeqItem->poseSeq();
    bool failed = false;
    
    for(auto& it : targetPoseIters){
        if(auto pose = it->get<BodyKeyPose>()){
            if(auto linkInfo = pose->ikLinkInfo(link->index())){
                Isometry3 T_link = linkInfo->T();
                adjustPosition(T_link, axis, sign);
                LinkAdjustmentBuf buf;

                if(!fetchJointDisplacements(pose, T_link)){
                    showErrorDialog(
                        format(_("The {0} position of key pose {1} cannot be adjusted due to the inverse kinematics error."),
                               link->name(), getPoseId(it, pose)));
                    linkAdjustmentBufs.clear();
                    failed = true;
                    break;
                }
                buf.it = it;
                buf.pose = pose;
                buf.linkInfo = linkInfo;
                buf.T_link = T_link;
                linkAdjustmentBufs.push_back(buf);
            }
        }
    }

    if(!linkAdjustmentBufs.empty()){
        auto block = poseSeqConnections.scopedBlock();
        poseSeqItem->beginEditing();
        for(auto& buf : linkAdjustmentBufs){
            poseSeq->beginPoseModification(buf.it);
            buf.linkInfo->setPosition(buf.T_link);
            if(doClearTouching){
                buf.linkInfo->clearTouching();
            }
            poseSeq->endPoseModification(buf.it);
            currentPoseIter = buf.it;
        }
        poseSeqItem->endEditing(true);
        updateTargetKeyPoses(true);

    } else if(!failed){
        showWarningDialog(_("There are no elements in the selected key poses to be adjusted."));
    }
}


void HumanoidPoseFetchView::Impl::adjustWaistPosition(int axis, double sign)
{
    adjustIkLinkPosition(
        waistLink, axis, sign, false,
        [this](BodyKeyPose* pose, const Isometry3& T_link){
            for(int i=0; i < 2; ++i){
                auto footLink = waistToFootJointPaths[i]->endLink();
                if(auto footInfo = pose->ikLinkInfo(footLink->index())){
                    if(!fetchLegJointDisplacements(i, true, T_link, footInfo->T(), nullptr)){
                        return false;
                    }
                }
            }
            return true;
        });
}


void HumanoidPoseFetchView::Impl::onWaistHeightOffsetCheckToggled(bool on)
{
    waistHeightSpin.blockSignals(true);
    
    if(on){ // offset mode
        waistHeightSpin.setRange(-1.0, 1.0);
        waistHeightSpin.setValue(waistHeightOffset);
    } else {
        waistHeightSpin.setRange(0.0, 10.0);
        waistHeightSpin.setValue(adjustedWaistHeight);
    }

    waistHeightSpin.blockSignals(false);
}


void HumanoidPoseFetchView::Impl::onWaistHeightSpinValueChanged(double value)
{
    if(waistHeightOffsetCheck.isChecked()){
        waistHeightOffset = value;
    } else {
        adjustedWaistHeight = value;
    }
}


void HumanoidPoseFetchView::Impl::adjustHeadRotation(int axis, double sign)
{
    if(auto joint = headRpyJoints[axis]){
        adjustJointAngle(joint, sign);
    }
}


void HumanoidPoseFetchView::Impl::adjustChestRotation(int axis, double sign)
{
    if(auto joint = chestRpyJoints[axis]){
        adjustJointAngle(joint, sign);
    }
}


void HumanoidPoseFetchView::Impl::adjustJointAngle(Link* joint, double sign)
{
    if(!poseSeqItem || !checkBodyValidity() || targetPoseIters.empty()){
        showErrorDialog(_("Not ready to adjust the joint angle."));
        return;
    }

    struct JointAdjustmentBuf {
        PoseSeq::iterator it;
        BodyKeyPose* pose;
        double q;
    };
    vector<JointAdjustmentBuf> jointAdjustmentBufs;
    jointAdjustmentBufs.reserve(targetPoseIters.size());

    auto poseSeq = poseSeqItem->poseSeq();
    int jointId = joint->jointId();
    bool failed = false;
    
    for(auto& it : targetPoseIters){
        if(auto pose = it->get<BodyKeyPose>()){
            if(pose->isJointValid(jointId)){
                double dq = sign * radian(isSmallStepAdjustmentMode ? 1.0 : 5.0);
                double q = pose->jointDisplacement(jointId) + dq;
                if(q > joint->q_upper() || q < joint->q_lower()){
                    showErrorDialog(
                        format(_("The joint angle of {0} in key pose {1} cannot be adjusted due to the joint limit over."),
                               joint->jointName(), getPoseId(it, pose)));
                    jointAdjustmentBufs.clear();
                    failed = true;
                    break;
                }
                JointAdjustmentBuf buf;
                buf.it = it;
                buf.pose = pose;
                buf.q = q;
                jointAdjustmentBufs.push_back(buf);
            }
        }
    }

    if(!jointAdjustmentBufs.empty()){
        auto block = poseSeqConnections.scopedBlock();
        poseSeqItem->beginEditing();
        for(auto& buf : jointAdjustmentBufs){
            poseSeq->beginPoseModification(buf.it);
            buf.pose->setJointDisplacement(jointId, buf.q);
            poseSeq->endPoseModification(buf.it);
            currentPoseIter = buf.it;
        }
        poseSeqItem->endEditing(true);
        updateTargetKeyPoses(true);

    } else if(!failed){
        showWarningDialog(_("There are no elements in the selected key poses to be adjusted."));
    }
}
    

void HumanoidPoseFetchView::Impl::adjustHandPosition(int which, int axis, double sign)
{
    if(!poseSeqItem || targetPoseIters.empty()){
        showErrorDialog(_("Not ready to adjust the hand position."));
        return;
    }
    for(int i=0; i < 2; ++i){
        if(armLinkSets[i].empty()){
            showErrorDialog(_("Not ready to adjust the hand position."));
            return;
        }
    }
    
    auto interpolator = poseSeqItem->interpolator();
    auto body = interpolator->body();
    if(!body){
        showErrorDialog(
            format(_("{0} must be associated with a body item to adjust hand positions."),
                   poseSeqItem->displayName()));
        return;
    }

    auto& path = armJointPaths[which];
    if(!path){
        auto& armLinks = armLinkSets[which];
        Link* baseLink = body->link(armLinks[0]->parent()->index());
        Link* endLink = body->link(armLinks.back()->index());
        if(!baseLink || !endLink){
            showErrorDialog(
                format(_("The internal body object of {0} does not conform to the source body."),
                       poseSeqItem->displayName()));
        }
        path = JointPath::getCustomPath(baseLink, endLink);
    }

    struct ArmAdjustmentBuf {
        PoseSeq::iterator it;
        BodyKeyPose* pose;
        vector<double> angles;
    };
    vector<ArmAdjustmentBuf> armAdjustmentBufs;
    armAdjustmentBufs.reserve(targetPoseIters.size());
    
    auto poseSeq = poseSeqItem->poseSeq();
    string message;
    bool failed = false;

    for(auto& it : targetPoseIters){
        if(auto pose = it->get<BodyKeyPose>()){
            bool hasCorrespondingJoints = true;
            for(auto& joint : *path){
                if(!pose->isJointValid(joint->jointId())){
                    hasCorrespondingJoints = false;
                    break;
                }
            }
            if(hasCorrespondingJoints){
                if(!interpolator->interpolate(it->time())){
                    message = format(
                        _("A hand position of key pose {0} cannot be adjusted due to the interpolation error."),
                        getPoseId(it, pose));
                    failed = true;
                } else {
                    Isometry3 T;
                    interpolator->getBaseLinkPosition(T);
                    body->rootLink()->setPosition(T);
                    std::vector<stdx::optional<double>> qs;
                    interpolator->getJointDisplacements(qs);
                    for(size_t i=0; i < qs.size(); ++i){
                        if(auto q = qs[i]){
                            body->joint(i)->q() = *q;
                        }
                    }
                    body->calcForwardKinematics();
                    
                    T = path->endLink()->T();
                    adjustPosition(T, axis, sign);
                    
                    if(!path->calcInverseKinematics(T)){
                        message = format(
                            _("A hand position of key pose {0} cannot be adjusted due to the inverse kinematics error."),
                            getPoseId(it, pose));
                        failed = true;
                    } else {
                        ArmAdjustmentBuf buf;
                        buf.it = it;
                        buf.pose = pose;
                        int n = path->numJoints();
                        buf.angles.resize(n);
                        for(int i=0; i < n; ++i){
                            buf.angles[i] = path->joint(i)->q();
                        }
                        armAdjustmentBufs.push_back(buf);
                    }
                    currentPoseIter = it;
                }
            }
            if(failed){
                break;
            }
        }
    }

    if(failed){
        showErrorDialog(message);

    } else if(!armAdjustmentBufs.empty()){
        auto block = poseSeqConnections.scopedBlock();
        poseSeqItem->beginEditing();
        for(auto& buf : armAdjustmentBufs){
            poseSeq->beginPoseModification(buf.it);
            for(int i=0; i < path->numJoints(); ++i){
                auto joint = path->joint(i);
                buf.pose->setJointDisplacement(joint->jointId(), buf.angles[i]);
            }
            poseSeq->endPoseModification(buf.it);
            currentPoseIter = buf.it;
        }
        poseSeqItem->endEditing(true);
        updateTargetKeyPoses(true);

    } else {
        showWarningDialog(_("There are no elements in the selected key poses to be adjusted."));
    }
}


void HumanoidPoseFetchView::Impl::adjustFootPosition(int which, int axis, double sign)
{
    bool doClearTouching = false;
    if(isRotationAdjustmentMode){
        if(axis == 0 || axis == 1){
            doClearTouching = true;
        }
    } else {
        if(axis == 2){
            doClearTouching = true;
        }
    }
    
    auto footLink = waistToFootJointPaths[which]->endLink();
    adjustIkLinkPosition(
        footLink, axis, sign, doClearTouching,
        [this, which](BodyKeyPose* pose, const Isometry3& T_link){
            if(auto waistInfo = pose->ikLinkInfo(waistLink->index())){
                return fetchLegJointDisplacements(which, true, waistInfo->T(), T_link, nullptr);
            }
            return true;
        });
}


void HumanoidPoseFetchView::Impl::setFootGrounding(int which, bool on)
{
    if(!targetPoseIters.empty()){
        bool modified = false;
        poseSeqItem->beginEditing();
        auto poseSeq = poseSeqItem->poseSeq();
        for(auto it : targetPoseIters){
            if(auto pose = it->get<BodyKeyPose>()){
                if(auto footLink = waistToFootJointPaths[which]->endLink()){
                    if(auto footInfo = pose->ikLinkInfo(footLink->index())){
                        if(on != footInfo->isTouching()){
                            poseSeq->beginPoseModification(it);
                            footInfo->setTouching(on);
                            poseSeq->endPoseModification(it);
                            modified = true;
                        }
                    }
                }
            }
        }
        poseSeqItem->endEditing(modified);
        updateTargetKeyPoses(true);
    }
}


void HumanoidPoseFetchView::Impl::adjustZmp(int axis, double sign)
{
    if(!poseSeqItem || !checkBodyValidity() || targetPoseIters.empty()){
        showErrorDialog(_("Not ready to adjust ZMP."));
        return;
    }

    bool modified = false;
    auto poseSeq = poseSeqItem->poseSeq();
    auto block = poseSeqConnections.scopedBlock();

    poseSeqItem->beginEditing();

    for(auto& it : targetPoseIters){
        if(auto pose = it->get<BodyKeyPose>()){
            if(pose->isZmpValid()){
                poseSeq->beginPoseModification(it);
                Vector3 zmp = pose->zmp();
                zmp[axis] += sign * (isSmallStepAdjustmentMode ? 0.001 : 0.01);
                pose->setZmp(zmp);
                modified = true;
                poseSeq->endPoseModification(it);
                currentPoseIter = it;
            }
        }
    }

    poseSeqItem->endEditing(modified);
}


void HumanoidPoseFetchView::Impl::handleBodyPartCheck
(int bodyPart, const string& bodyPartName, CheckBox& checkBox, bool on)
{
    if(on){
        // Ignore checking
        checkBox.blockSignals(true);
        checkBox.setChecked(false);
        checkBox.blockSignals(false);
        
    } else {
        if(!targetPoseIters.empty()){

            bool modified = false;
            
            // Do test
            bool needKeyPoseRemoval = false;
            for(auto& it : targetPoseIters){
                if(auto pose = it->get<BodyKeyPose>()){
                    BodyKeyPosePtr clone = pose->clone();
                    removeBodyPart(clone, bodyPart);
                    if(clone->empty()){
                        needKeyPoseRemoval = true;
                        break;
                    }
                }
            }
            bool doExecution = true;
            if(needKeyPoseRemoval){
                if(!confirmKeyPoseRemovalByUncheckingBodypart(bodyPartName, checkBox)){
                    doExecution = false;
                }
            }

            if(doExecution){
                auto poseSeq = poseSeqItem->poseSeq();
                auto block = poseSeqConnections.scopedBlock();
                poseSeqItem->beginEditing();

                auto it = targetPoseIters.begin();
                while(it != targetPoseIters.end()){
                    auto poseIter = (*it);
                    if(auto pose = poseIter->get<BodyKeyPose>()){
                        poseSeq->beginPoseModification(poseIter);
                        if(removeBodyPart(pose, bodyPart)){
                            modified = true;
                        }
                        poseSeq->endPoseModification(poseIter);
                        if(pose->empty()){
                            if(poseIter == currentPoseIter){
                                currentPoseIter = poseSeq->end();
                            }
                            poseSeq->erase(poseIter);
                            it = targetPoseIters.erase(it);
                            modified = true;
                        } else {
                            ++it;
                        }
                    }
                }
                
                poseSeqItem->endEditing(modified);
            }
            
            if(modified || needKeyPoseRemoval){
                updateTargetKeyPoses(true);
            }
        }
    }
}


bool HumanoidPoseFetchView::Impl::removeBodyPart(BodyKeyPose* pose, int bodyPart)
{
    bool modified = false;
    
    if(bodyPart & Waist){
        if(pose->removeIkLink(waistLink->index())){
            modified = true;
        }
    }
    if(bodyPart & Head){
        if(removeLinkElementsFromKeyPose(pose, headLinks)){
            modified = true;
        }
    }
    if(bodyPart & Chest){
        if(removeLinkElementsFromKeyPose(pose, chestLinks)){
            modified = true;
        }
    }

    int armParts[2] = { RightArm, LeftArm };
    int legParts[2] = { RightLeg, LeftLeg };

    for(int i=0; i < 2; ++i){
        if(bodyPart & armParts[i]){
            if(removeLinkElementsFromKeyPose(pose, armLinkSets[i])){
                modified = true;
            }
        }
        if(bodyPart & legParts[i]){
            if(removeLinkElementsFromKeyPose(pose, legLinkSets[i])){
                modified = true;
            }
        }
    }

    if(bodyPart & Zmp){
        if(pose->invalidateZmp()){
            modified = true;
        }
    }

    return modified;
}


bool HumanoidPoseFetchView::Impl::removeLinkElementsFromKeyPose(BodyKeyPose* pose, vector<LinkPtr>& links)
{
    bool modified = false;
    
    for(auto& link : links){
        if(pose->removeIkLink(link->index())){
            modified = true;
        }
        if(link->jointId() >= 0){
            if(pose->invalidateJoint(link->jointId())){
                modified = true;
            }
        }
    }
    
    return modified;
}


bool HumanoidPoseFetchView::Impl::confirmKeyPoseRemovalByUncheckingBodypart
(const std::string& bodyPartName, CheckBox& checkBox)
{
    bool confirmed =
        showConfirmDialog(
            _("Warning"),
            format(
                _("Key poses will be removed by unchecking the {0} check. "
                  "Do you really want to remove the key poses?"),
                bodyPartName));

    if(!confirmed){
        checkBox.blockSignals(true);
        checkBox.setChecked(true);
        checkBox.blockSignals(false);
    }

    return confirmed;
}


bool HumanoidPoseFetchView::storeState(Archive& archive)
{
    return impl->storeState(archive);
}


bool HumanoidPoseFetchView::Impl::storeState(Archive& archive)
{
    archive.writeItemId("src_body_item", srcBodyItem);
    archive.writeItemId("pose_seq_item", poseSeqItem);
    archive.write("force_new_pose", forceNewPoseCheck.isChecked());
    archive.write("include_waist_in_upper_body", upperBodyWaistCheck.isChecked());
    archive.write("keep_within_upper_body_joint_ranges", jointRangeLimitCheck.isChecked());
    archive.write("adjusted_waist_height", adjustedWaistHeight);
    archive.write("waist_height_offset", waistHeightOffset);
    archive.write("is_waist_height_offset_mode", waistHeightOffsetCheck.isChecked());
    archive.write("include_waist_in_leg", legWaistCheck.isChecked());

    if(adjustmentCoordRadioGroup.checkedId() == GlobalCoordinate){
        archive.write("adjustment_coordinate_system", "global");
    } else {
        archive.write("adjustment_coordinate_system", "local");
    }
    
    return true;
}


bool HumanoidPoseFetchView::restoreState(const Archive& archive)
{
    return impl->restoreState(archive);
}


bool HumanoidPoseFetchView::Impl::restoreState(const Archive& archive)
{
    forceNewPoseCheck.setChecked(
        archive.get("force_new_pose", forceNewPoseCheck.isChecked()));
    upperBodyWaistCheck.setChecked(
        archive.get("include_waist_in_upper_body", upperBodyWaistCheck.isChecked()));
    jointRangeLimitCheck.setChecked(
        archive.get("keep_within_upper_body_joint_ranges", jointRangeLimitCheck.isChecked()));
    archive.read("adjusted_waist_height", adjustedWaistHeight);
    archive.read("waist_height_offset", waistHeightOffset);
    legWaistCheck.setChecked(
        archive.get("include_waist_in_leg", legWaistCheck.isChecked()));
    
    waistHeightOffsetCheck.blockSignals(true);
    onWaistHeightOffsetCheckToggled(waistHeightOffsetCheck.isChecked());
    waistHeightOffsetCheck.blockSignals(false);

    string cs;
    if(archive.read("adjustment_coordinate_system", cs)){
        if(cs == "global"){
            globalCoordRadio.setChecked(true);
        } else if(cs == "local"){
            localCoordRadio.setChecked(true);
        }
    }

    archive.addPostProcess([this, &archive](){ restoreItems(archive); });
    
    return true;
}


void HumanoidPoseFetchView::Impl::restoreItems(const Archive& archive)
{
    if(auto item = archive.findItem<BodyItem>("src_body_item")){
        setSrcBodyItem(item, true);
    }
    if(auto item = archive.findItem<PoseSeqItem>("pose_seq_item")){
        setPoseSeqItem(item);
    }
}
