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
#include <cnoid/ConnectionSet>
#include <cnoid/EigenUtil>
#include <cnoid/ComboBox>
#include <cnoid/Separator>
#include <cnoid/CheckBox>
#include <cnoid/Buttons>
#include <cnoid/DoubleSpinBox>
#include <QBoxLayout>
#include <QGridLayout>
#include <QLabel>
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

    Arms = RightArm | LeftArm,
    UpperBody = Head | Arms | Chest,
    Legs = RightLeg | LeftLeg,
    LowerBody = Legs | Waist,
    WholeBody = Waist | UpperBody | Legs
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

}

namespace cnoid {

class HumanoidPoseFetchView::Impl
{
public:
    HumanoidPoseFetchView* self;

    ScopedConnectionSet baseConnections;
    ScopedConnection poseRollViewTimeConnection;
    BodyItem* srcBodyItem;
    LinkPtr waistLink;
    vector<LinkPtr> headLinks;
    vector<LinkPtr> chestLinks;
    vector<LinkPtr> armLinkSets[2];
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
    BodyPartCheckBox wholeBodyCheck;
    PushButton wholeBodyButton;
    CheckBox jointRangeLimitCheck;
    BodyPartCheckBox upperBodyCheck;
    PushButton upperBodyButton;
    CheckBox upperBodyWaistCheck;
    BodyPartCheckBox headCheck;
    PushButton headButton;
    BodyPartCheckBox armChecks[2];
    PushButton armButtons[2];
    BodyPartCheckBox armsCheck;
    PushButton armsButton;
    BodyPartCheckBox chestCheck;
    PushButton chestButton;
    PushButton waistUpButton;
    BodyPartCheckBox waistCheck;
    PushButton waistButton;
    double adjustedWaistHeight;
    double waistHeightOffset;
    DoubleSpinBox waistHeightSpin;
    CheckBox waistHeightOffsetCheck;
    PushButton waistDownButton;
    BodyPartCheckBox legChecks[2];
    PushButton legButtons[2];
    CheckBox legWaistCheck;
    PushButton footUpButtons[2];
    PushButton footDownButtons[2];
    PushButton footGroundingButtons[2];
    CheckBox footGroundingChecks[2];
    BodyPartCheckBox legsCheck;
    PushButton legsButton;
    CheckBox relativePositionCheck;

    vector<CheckBox*> bodyPartChecks;

    Impl(HumanoidPoseFetchView* self);
    ~Impl();
    int mergeWaistPartCheck(int bodyPart, CheckBox& check);
    void onActivated();
    void setSrcTime(double time);
    void setPoseSeqTime(double time);
    void onPoseRollViewTimeBarSyncToggled(PoseRollView* poseRollView, bool on);
    void updateSrcBodyItemCombo();
    bool checkIfBipedHumanoidRobot(Body* body);
    void onSrcBodyItemComboActivated(int comboIndex);
    void setSrcBodyItem(BodyItem* bodyItem, bool doUpdateCombo);
    void setBodyPartLinks(
        Body* body, LinkGroup* group, const string& prefix, vector<LinkPtr>& out_links);
    bool checkBodyValidity();
    void onSelectedItemsChanged(ItemList<PoseSeqItem> selectedItems);
    void setPoseSeqItem(PoseSeqItem* item);
    void updateTargetKeyPoses();
    void clearTargetKeyPoses();    
    void setTargetKeyPoses(const std::vector<PoseSeq::iterator>& poses);
    void updateKeyPoseInterface(BodyKeyPose* pose);
    Qt::CheckState getCombinedState(int state1, int state2);
    Qt::CheckState getCombinedState(int state1, int state2, int state3);
    Qt::CheckState getBodyPartState(BodyKeyPose* pose, const vector<LinkPtr>& partLinks, bool doCheckIkLink);
    void fetchOrCreatePoses(int bodyPart, bool doFootGrounding);
    bool fetchPose(BodyKeyPose* pose, int bodyPart, bool doFootGrounding);
    bool setLinkElementsToKeyPose(BodyKeyPose* pose, vector<LinkPtr>& links);
    bool fetchLegPose(
        BodyKeyPose* pose, int which,
        bool isWaistPositionIncluded, bool isWaistPositionModified, const Isometry3& T_waist, bool doFootGrounding);
    bool fetchLegJointDisplacements(
        BodyKeyPose* pose, int which, bool isLegPoseModified, const Isometry3& T_waist, const Isometry3& T_foot,
        bool doUpdatee);
    void adjustWaistHeight(double diff);
    void onWaistHeightOffsetCheckToggled(bool on);
    void onWaistHeightSpinValueChanged(double value);
    void adjustFootHeight(int which, double diff);
    void setFootGrounding(int which, bool on);
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
        [this](bool /* on */){ updateTargetKeyPoses(); });
    topGrid->addWidget(&forceNewPoseCheck, 2, 2, 1, 2);
    
    mainVBox->addLayout(topGrid);
    mainVBox->addWidget(new HSeparator);

    auto grid = new QGridLayout;
    grid->setColumnStretch(4, 1);
    grid->setColumnStretch(5, 1);
    grid->setColumnStretch(6, 1);

    int row = 0;

    bodyPartChecks.reserve(10);

    wholeBodyCheck.sigToggled().connect(
        [this](bool on){
            handleBodyPartCheck(WholeBody, _("whole body"), wholeBodyCheck, on);
        });
    bodyPartChecks.push_back(&wholeBodyCheck);
    grid->addWidget(&wholeBodyCheck, row, 0);
    
    wholeBodyButton.setText(_("Whole Body"));
    wholeBodyButton.sigClicked().connect(
        [this](){ fetchOrCreatePoses(WholeBody, false); });
    grid->addWidget(&wholeBodyButton, row, 1, 1, 9);

    ++row;

    jointRangeLimitCheck.setText(_("Keep within upper body joint ranges"));
    jointRangeLimitCheck.setChecked(true);
    grid->addWidget(&jointRangeLimitCheck, row, 1, 1, 9, Qt::AlignCenter);
    ++row;

    upperBodyCheck.sigToggled().connect(
        [this](bool on){
            handleBodyPartCheck(UpperBody, _("upper body"), upperBodyCheck, on);
        });
    bodyPartChecks.push_back(&upperBodyCheck);
    grid->addWidget(&upperBodyCheck, row, 1, Qt::AlignRight);
    
    upperBodyButton.setText(_("Upper Body"));
    upperBodyButton.sigClicked().connect(
        [this](){
            fetchOrCreatePoses(
                mergeWaistPartCheck(UpperBody, upperBodyWaistCheck), false); });
    grid->addWidget(&upperBodyButton, row, 2, 1, 7);

    upperBodyWaistCheck.setText(_("Waist"));
    upperBodyWaistCheck.setChecked(true);
    upperBodyWaistCheck.sigToggled().connect(
        [this](bool){ updateTargetKeyPoses(); });
    grid->addWidget(&upperBodyWaistCheck, row, 9);
    
    ++row;

    headCheck.sigToggled().connect(
        [this](bool on){
            handleBodyPartCheck(Head, _("head"), headCheck, on);
        });
    bodyPartChecks.push_back(&headCheck);
    grid->addWidget(&headCheck, row, 4, Qt::AlignRight);
    
    headButton.setText(_("Head"));
    headButton.sigClicked().connect(
        [this](){ fetchOrCreatePoses(Head, false); });
    grid->addWidget(&headButton, row, 5);

    ++row;

    const char* armNames[] = { _("R-Arm"), _("L-Arm") };

    for(int i=0; i < 2; ++i){
        int armPart = (i == 0) ? RightArm : LeftArm;
        auto armName = armNames[i];
        auto check = &armChecks[i];

        check->sigToggled().connect(
            [this, armPart, armName, check](bool on){
                handleBodyPartCheck(armPart, armName, *check, on);
            });
        bodyPartChecks.push_back(check);
        grid->addWidget(check, row, i * 7, Qt::AlignRight | Qt::AlignVCenter);

        auto button = &armButtons[i];
        button->setText(armNames[i]);
        button->sigClicked().connect(
            [this, armPart](){ fetchOrCreatePoses(armPart, false); });
        grid->addWidget(button, row, i * 7 + 1, 1, 2);
    }

    armsCheck.sigToggled().connect(
        [this](bool on){
            handleBodyPartCheck(Arms, _("arms"), armsCheck, on);
        });
    bodyPartChecks.push_back(&armsCheck);
    grid->addWidget(&armsCheck, row, 3);
    
    armsButton.setText(_("Arms"));
    armsButton.sigClicked().connect(
        [this](){ fetchOrCreatePoses(Arms, false); });
    grid->addWidget(&armsButton, row, 4, 1, 3);

    ++row;

    chestCheck.sigToggled().connect(
        [this](bool on){
            handleBodyPartCheck(Chest, _("chest"), chestCheck, on);
        });
    bodyPartChecks.push_back(&chestCheck);
    grid->addWidget(&chestCheck, row, 4, Qt::AlignRight);
    
    chestButton.setText(_("Chest"));
    chestButton.sigClicked().connect(
        [this](){ fetchOrCreatePoses(Chest, false); });
    grid->addWidget(&chestButton, row, 5);
    
    ++row;

    adjustedWaistHeight = 0.8;
    waistHeightOffset = 0.0;

    auto hbox = new QHBoxLayout;
    hbox->addStretch();
    waistUpButton.setText(_("^"));
    waistUpButton.sigClicked().connect(
        [this](){ adjustWaistHeight(0.01); });
    hbox->addWidget(&waistUpButton);
    hbox->addStretch();
    grid->addLayout(hbox, row, 5);

    grid->addWidget(new QLabel(_("Height")), row, 7, 1, 4, Qt::AlignCenter | Qt::AlignBottom);
    ++row;

    waistCheck.sigToggled().connect(
        [this](bool on){
            handleBodyPartCheck(Waist, _("waist"), waistCheck, on);
        });
    bodyPartChecks.push_back(&waistCheck);
    grid->addWidget(&waistCheck, row, 3);
    
    waistButton.setText(_("Waist"));
    waistButton.sigClicked().connect(
        [this](){ fetchOrCreatePoses(Waist, false); });
    grid->addWidget(&waistButton, row, 4, 1, 3);

    waistHeightSpin.setAlignment(Qt::AlignCenter);
    waistHeightSpin.setDecimals(3);
    waistHeightSpin.sigValueChanged().connect(
        [this](double value){ onWaistHeightSpinValueChanged(value); });
    grid->addWidget(&waistHeightSpin, row, 7, 1, 4);
    ++row;

    hbox = new QHBoxLayout;
    hbox->addStretch();
    waistDownButton.setText(_("v"));
    waistDownButton.sigClicked().connect(
        [this](){ adjustWaistHeight(-0.01); });
    hbox->addWidget(&waistDownButton);
    hbox->addStretch();
    grid->addLayout(hbox, row, 5);

    waistHeightOffsetCheck.setText(_("Offset"));
    waistHeightOffsetCheck.setChecked(true);
    waistHeightOffsetCheck.sigToggled().connect(
        [this](bool on){ onWaistHeightOffsetCheckToggled(on); });
    onWaistHeightOffsetCheckToggled(true);
    grid->addWidget(&waistHeightOffsetCheck, row, 7, 1, 4, Qt::AlignHCenter | Qt::AlignTop);

    ++row;

    const char* legNames[] = { _("R-Leg"), _("L-Leg") };

    for(int i=0; i < 2; ++i){
        int legPart = (i == 0) ? RightLeg : LeftLeg;
        auto legName = legNames[i];
        auto legCheck = &legChecks[i];

        legCheck->sigToggled().connect(
            [this, legPart, legName, legCheck](bool on){
                handleBodyPartCheck(legPart, legName, *legCheck, on);
            });
        bodyPartChecks.push_back(legCheck);
        grid->addWidget(legCheck, row + 1, i * 7, Qt::AlignRight | Qt::AlignVCenter);

        auto legButton = &legButtons[i];
        legButton->setText(legName);
        legButton->sigClicked().connect(
            [this, legPart](){
                fetchOrCreatePoses(mergeWaistPartCheck(legPart, legWaistCheck), false);
            });
        grid->addWidget(legButton, row + 1, i * 7 + 1, 1, 2);

        auto upButton = &footUpButtons[i];
        upButton->setText(_("^"));
        upButton->sigClicked().connect(
            [this, i](){ adjustFootHeight(i, 0.01); });
        grid->addWidget(upButton, row, i * 7 + 1, 1 , 2);

        auto downButton = &footDownButtons[i];
        downButton->setText(_("v"));
        downButton->sigClicked().connect(
            [this, i](){ adjustFootHeight(i, -0.01); });
        grid->addWidget(downButton, row + 2, i * 7 + 1, 1 , 2);

        auto groundCheck = &footGroundingChecks[i];
        bodyPartChecks.push_back(groundCheck);
        groundCheck->sigToggled().connect(
            [this, i](bool on){
                setFootGrounding(i, on);
            });
        grid->addWidget(groundCheck, row + 3, i * 7, Qt::AlignRight | Qt::AlignVCenter);
        
        auto groundButton = &footGroundingButtons[i];
        groundButton->setText(_("Ground"));
        groundButton->sigClicked().connect(
            [this, legPart](){
                fetchOrCreatePoses(mergeWaistPartCheck(legPart, legWaistCheck), true); });
        grid->addWidget(groundButton, row + 3, i * 7 + 1, 1, 2, Qt::AlignCenter);
    }

    legsCheck.sigToggled().connect(
        [this](bool on){
            handleBodyPartCheck(mergeWaistPartCheck(Legs, legWaistCheck), _("legs"), legsCheck, on);
        });
    bodyPartChecks.push_back(&legsCheck);
    grid->addWidget(&legsCheck, row + 1, 3);
    
    legsButton.setText(_("Legs"));
    legsButton.sigClicked().connect(
        [this](){ fetchOrCreatePoses(mergeWaistPartCheck(Legs, legWaistCheck), false); });
    grid->addWidget(&legsButton, row + 1, 4, 1, 3);

    legWaistCheck.setText(_("Waist"));
    legWaistCheck.setChecked(true);
    legWaistCheck.sigToggled().connect(
        [this](bool){ updateTargetKeyPoses(); });
    grid->addWidget(&legWaistCheck, row + 2, 4, 1, 3, Qt::AlignCenter);

    row += 4;

    relativePositionCheck.setText(_("Relative positioning for the waist and feet"));
    relativePositionCheck.setChecked(true);
    // Not implemented yet
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
                    return true;
                }));

        baseConnections.add(
            poseRollView->sigTimeBarSyncToggled().connect(
                [this, poseRollView](bool on){
                    onPoseRollViewTimeBarSyncToggled(poseRollView, on);
                }));
    }

    setSrcTime(timeBar->time());
    onPoseRollViewTimeBarSyncToggled(poseRollView, poseRollView->isTimeBarSyncEnabled());
}


void HumanoidPoseFetchView::onDeactivated()
{
    impl->baseConnections.disconnect();
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
        for(int i=0; i < 2; ++i){
            armLinkSets[i].clear();
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
                }
                const char* armGroupNames[2] = { "R_ARM", "L_ARM" };
                for(int i=0; i < 2; ++i){
                    if(auto armGroup = upperBodyGroup->findSubGroup(armGroupNames[i])){
                        setBodyPartLinks(body, armGroup, "", armLinkSets[i]);
                    }
                }
                if(auto chestGroup = upperBodyGroup->findSubGroup("CHEST")){
                    setBodyPartLinks(body, chestGroup, "", chestLinks);
                }
            }
            const char* legGroupNames[2] = { "R_LEG", "L_LEG" };
            for(int i=0; i < 2; ++i){
                if(auto legGroup = linkGroup->findSubGroup(legGroupNames[i])){
                    setBodyPartLinks(body, legGroup, "", legLinkSets[i]);
                    legJointPaths[i] = JointPath::getCustomPath(body, waistLink, legLinkSets[i].back());
                    
                    // Find the foot link
                    for(int j=0; j < leggedBody->numFeet(); ++j){
                        for(auto& link : legLinkSets[i]){
                            if(link == leggedBody->footLink(j)){
                                waistToFootJointPaths[i] = JointPath::getCustomPath(body, waistLink, link);
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
        if(!legJointPaths[i] || !waistToFootJointPaths[i]){
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
                        updateTargetKeyPoses();
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
            updateTargetKeyPoses();
        }
    }
}


void HumanoidPoseFetchView::Impl::updateTargetKeyPoses()
{
    if(poseSeqItem && !forceNewPoseCheck.isChecked()){
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
    for(auto& it : poseIters){
        if(auto bkPose = it->get<BodyKeyPose>()){
            targetPoseIters.push_back(it);
            if(!firstKeyPose){
                firstKeyPose = bkPose;
            }
            currentPoseIter = it;
        }
    }

    if(targetPoseIters.empty()){
        targetPoseLabel.setText(_("New pose"));
    } else {
        ulong id = reinterpret_cast<ulong>(firstKeyPose) & 0xffffffff;
        if(targetPoseIters.size() == 1){
            targetPoseLabel.setText(format("{0:0X}", id).c_str());
        } else {
            targetPoseLabel.setText(format(_("{0:0X} ..."), id).c_str());
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
        armChecks[i].setCheckState(armStates[i]);

        auto legState = getBodyPartState(pose, legLinkSets[i], true);
        if(legState != Qt::Unchecked && legWaistCheck.isChecked()){
            legState = getCombinedState(legState, waistState);
        }
        legChecks[i].setCheckState(legState);
        legStates[i] = legState;

        bool isTouching = false;
        footIk[i] = false;
        if(auto footLink = waistToFootJointPaths[i]->endLink()){
            if(auto footInfo = pose->ikLinkInfo(footLink->index())){
                footIk[i] = true;
                isTouching = footInfo->isTouching();
            }
        }
        footGroundingChecks[i].setChecked(isTouching);
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


void HumanoidPoseFetchView::Impl::fetchOrCreatePoses(int bodyPart, bool doFootGrounding)
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
        if(!fetchPose(pose, bodyPart, doFootGrounding)){
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
                if(!fetchPose(pose, bodyPart, doFootGrounding)){
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
        updateTargetKeyPoses();
    }
}


bool HumanoidPoseFetchView::Impl::fetchPose(BodyKeyPose* pose, int bodyPart, bool doFootGrounding)
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
            if(fetchLegPose(pose, i, isWaistPositionIncluded, isWaistPositionModified, T_waist, doFootGrounding)){
                fetched = true;
            } else {
                failed = true;
            }
        }
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
    constexpr double MarginToHaveContactDepth = 0.002;
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
        if(!fetchLegJointDisplacements(pose, which, isLegPoseModified, T_waist, T_foot, false)){
            failed = true;
        }
    }

    if(!failed){
        footInfo->setPosition(T_foot);
        footInfo->setTouching(doFootGrounding);
    }

    return !failed;
}


bool HumanoidPoseFetchView::Impl::fetchLegJointDisplacements
(BodyKeyPose* pose, int which, bool isLegPoseModified, const Isometry3& T_waist, const Isometry3& T_foot,
 bool doUpdate)
{
    bool failed = false;

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
    if(doUpdate && !failed){
        for(auto& link : legLinkSets[which]){
            pose->setJointDisplacement(link->jointId(), link->q());
        }
    }
    if(isLegPoseModified){
        // restore the original pose
        waistLink->setPosition(T_waist_org);
        for(int i=0; i < waistToFoot->numJoints(); ++i){
            waistToFoot->joint(i)->q() = q0[i];
        }
        legJointPaths[which]->calcForwardKinematics();
    }

    return !failed;
}


void HumanoidPoseFetchView::Impl::adjustWaistHeight(double diff)
{
    if(!poseSeqItem || !checkBodyValidity() || targetPoseIters.empty()){
        showErrorDialog(_("Not ready to adjust the waist height."));
        return;
    }

    bool modified = false;
    auto poseSeq = poseSeqItem->poseSeq();
    auto block = poseSeqConnections.scopedBlock();

    poseSeqItem->beginEditing();
    
    for(auto& it : targetPoseIters){
        if(auto pose = it->get<BodyKeyPose>()){
            if(auto waistInfo = pose->ikLinkInfo(waistLink->index())){
                poseSeq->beginPoseModification(it);
                Isometry3 T_waist = waistInfo->T();
                T_waist.translation().z() += diff;
                bool failed = false;
                for(int i=0; i < 2; ++i){
                    auto footLink = waistToFootJointPaths[i]->endLink();
                    if(auto footInfo = pose->ikLinkInfo(footLink->index())){
                        if(!fetchLegJointDisplacements(pose, i, true, T_waist, footInfo->T(), false)){
                            failed = true;
                            break;
                        }
                    }
                }
                if(!failed){
                    waistInfo->setPosition(T_waist);
                    modified = true;
                }
                poseSeq->endPoseModification(it);
                currentPoseIter = it;
            }
        }
    }
    poseSeqItem->endEditing(modified);

    if(!targetPoseIters.empty() && !modified){
        if(targetPoseIters.size() == 1){
            showWarningDialog(
                _("The waist height of the selected key pose cannot be modified due to the inverse kinematics error."));
        } else {
            showWarningDialog(
                _("The waist height of the selected key poses cannot be modified due to the inverse kinematics error."));
        }
    }
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


void HumanoidPoseFetchView::Impl::adjustFootHeight(int which, double diff)
{
    if(!poseSeqItem || !checkBodyValidity() || targetPoseIters.empty()){
        showErrorDialog(_("Not ready to adjust the foot height."));
        return;
    }

    bool modified = false;
    auto footLink = waistToFootJointPaths[which]->endLink();
    auto poseSeq = poseSeqItem->poseSeq();
    auto block = poseSeqConnections.scopedBlock();

    poseSeqItem->beginEditing();
    
    for(auto& it : targetPoseIters){
        if(auto pose = it->get<BodyKeyPose>()){
            if(auto footInfo = pose->ikLinkInfo(footLink->index())){
                poseSeq->beginPoseModification(it);
                Isometry3 T_foot = footInfo->T();
                T_foot.translation().z() += diff;
                bool failed = false;
                if(auto waistInfo = pose->ikLinkInfo(waistLink->index())){
                    if(!fetchLegJointDisplacements(pose, which, true, waistInfo->T(), T_foot, false)){
                        failed = true;
                    }
                }
                if(!failed){
                    footInfo->setPosition(T_foot);
                    footInfo->clearTouching();
                    modified = true;
                }
                poseSeq->endPoseModification(it);
                currentPoseIter = it;
            }
        }
    }
    poseSeqItem->endEditing(modified);

    if(modified){
        updateTargetKeyPoses();
    }

    if(!targetPoseIters.empty() && !modified){
        if(targetPoseIters.size() == 1){
            showWarningDialog(
                _("The foot height of the selected key pose cannot be modified due to the inverse kinematics error."));
        } else {
            showWarningDialog(
                _("The foot height of the selected key poses cannot be modified due to the inverse kinematics error."));
        }
    }

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
        updateTargetKeyPoses();
    }
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
                updateTargetKeyPoses();
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
