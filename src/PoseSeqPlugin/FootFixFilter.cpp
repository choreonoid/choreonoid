#include "FootFixFilter.h"
#include <cnoid/RootItem>
#include <cnoid/ItemList>
#include <cnoid/Archive>
#include <cnoid/ExtensionManager>
#include <cnoid/MainMenu>
#include <cnoid/MessageView>
#include <cnoid/BodyItem>
#include <cnoid/BodyMotionItem>
#include <cnoid/LeggedBodyHelper>
#include <cnoid/BodyState>
#include <cnoid/ZMPSeq>
#include <cnoid/BodyKeyPose>
#include <cnoid/EigenUtil>
#include <cnoid/Dialog>
#include <cnoid/Button>
#include <cnoid/SpinBox>
#include <cnoid/FloatingNumberBox>
#include <cnoid/PoseSeqInterpolator>
#include <cnoid/stdx/optional>
#include <Eigen/StdVector>
#include <QLabel>
#include <QDialogButtonBox>
#include <QBoxLayout>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

namespace {

typedef std::vector<Isometry3, Eigen::aligned_allocator<Isometry3>> Isometry3Array;

class FootFixFilterDialog : public Dialog
{
public:
    ostream& os;
    DoubleSpinBox waistHeightDiffSpin;
    CheckBox transitionCheck;
    DoubleSpinBox pausingTimeSpin;
    DoubleSpinBox transitionTimeSpin;
    
    FootFixFilterDialog();
    bool store(Archive& archive);
    void restore(const Archive& archive);
    virtual void onAccepted() override;
    bool fixFootPositions(
        Body* body, const BodyMotion& orgMotion, double waistHeightDiff, double pausingTime, double transitionTime,
        BodyMotion& out_motion);
};

}


void cnoid::initializeFootFixFilter(ExtensionManager* ext)
{
    static FootFixFilterDialog* dialog = nullptr;

    if(!dialog){
        dialog = ext->manage(new FootFixFilterDialog);

        MainMenu::instance()->add_Filters_Item(
            _("Fix foot positions in body motions"), [](){ dialog->show(); });

        ext->setProjectArchiver(
            "FootFixFilterDialog",
            [](Archive& archive){ return dialog->store(archive); },
            [](const Archive& archive){ return dialog->restore(archive);});
    }
}

            
FootFixFilterDialog::FootFixFilterDialog()
    : os(MessageView::instance()->cout())
{
    setWindowTitle(_("Fix foot positions in body motions"));

    QVBoxLayout* vbox = new QVBoxLayout;
    setLayout(vbox);

    QHBoxLayout* hbox = new QHBoxLayout;
    hbox->addWidget(new QLabel(_("Waist height diff")));
    waistHeightDiffSpin.setDecimals(3);
    waistHeightDiffSpin.setRange(-99.0, 99.0);
    waistHeightDiffSpin.setValue(0.0);
    hbox->addWidget(&waistHeightDiffSpin);
    hbox->addStretch();
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout;
    transitionCheck.setText(_("Transition from / to the standard pose"));
    transitionCheck.setChecked(true);
    hbox->addWidget(&transitionCheck);
    hbox->addStretch();
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout;
    hbox->addSpacing(20);
    hbox->addWidget(new QLabel(_("Pausing time")));
    pausingTimeSpin.setValue(1.0);
    hbox->addWidget(&pausingTimeSpin);
    hbox->addWidget(new QLabel(_("Transition time")));
    transitionTimeSpin.setValue(1.0);
    hbox->addWidget(&transitionTimeSpin);
    hbox->addStretch();
    vbox->addLayout(hbox);
    
    PushButton* applyButton = new PushButton(_("&Apply"));
    applyButton->setDefault(true);
    QDialogButtonBox* buttonBox = new QDialogButtonBox(this);
    buttonBox->addButton(applyButton, QDialogButtonBox::AcceptRole);
    applyButton->sigClicked().connect([this](){ accept(); });
    
    vbox->addWidget(buttonBox);
}


bool FootFixFilterDialog::store(Archive& archive)
{
    archive.write("waist_height_offset", waistHeightDiffSpin.value());
    return true;
}


void FootFixFilterDialog::restore(const Archive& archive)
{
    waistHeightDiffSpin.setValue(archive.get("waist_height_offset", waistHeightDiffSpin.value()));
}


void FootFixFilterDialog::onAccepted()
{
    auto motionItems = RootItem::instance()->selectedItems<BodyMotionItem>();
    for(int i=0; i < motionItems.size(); ++i){
        BodyMotionItem* orgMotionItem = motionItems[i];
        BodyItem* bodyItem = orgMotionItem->findOwnerItem<BodyItem>();
        if(bodyItem){
            os << format(_("Apply the foot fix filter to {0} of {1}"),
                         orgMotionItem->name(), bodyItem->name()) << endl;
            
            BodyMotionItemPtr filterdMotionItem = new BodyMotionItem;
            BodyPtr body = bodyItem->body()->clone();

            bool result = fixFootPositions(
                body, *orgMotionItem->motion(),
                waistHeightDiffSpin.value(), pausingTimeSpin.value(), transitionTimeSpin.value(),
                *filterdMotionItem->motion());

            if(result){
                filterdMotionItem->setName(orgMotionItem->name() + "-foot-fixed");
                bodyItem->addChildItem(filterdMotionItem);
            } else {
                os << "failed." << endl;
            }
        }
    }
}


static void setStandardPose(Body* body)
{
    const Listing& pose = *body->info()->findListing({ "standard_pose", "standardPose" });
    if(pose.isValid()){
        int jointIndex = 0;
        const int n = std::min(pose.size(), body->numJoints());
        while(jointIndex < n){
            body->joint(jointIndex)->q() = radian(pose[jointIndex].toDouble());
            ++jointIndex;
        }
        while(jointIndex < body->numJoints()){
            body->joint(jointIndex++)->q() = 0.0;
        }
        body->calcForwardKinematics();
    }
}


static void getUpperPartTraverseIter(LinkTraverse& traverse, Link* link, double z)
{
    traverse.append(link);
    for(Link* child = link->child(); child; child = child->sibling()){
        if(child->translation().z() > z){
            getUpperPartTraverseIter(traverse, child, z);
        }
    }
}
        

static LinkTraverse getUpperPartTraverse(Link* link)
{
    LinkTraverse traverse;
    getUpperPartTraverseIter(traverse, link, link->translation().z());
    return traverse;
}


static void setTransition
(PoseSeqInterpolator& interpolator, Body* body, const BodyState& state1, const BodyState& state2, double time)
{
    PoseSeqPtr pseq = new PoseSeq;
    PoseSeq::iterator iter = pseq->begin();
    
    Link* root = body->rootLink();
    
    BodyKeyPosePtr pose1 = new BodyKeyPose;
    *body << state1;
    pose1->setBaseLink(root->index(), root->T());
    LinkTraverse upperPart = getUpperPartTraverse(root);
    for(int i=1; i < upperPart.size(); ++i){
        Link* link = upperPart[i];
        pose1->setJointDisplacement(link->jointId(), link->q());
    }
    iter = pseq->insert(iter, 0.0, pose1);
    
    BodyKeyPosePtr pose2 = new BodyKeyPose;
    *body << state2;
    pose2->setBaseLink(root->index(), root->T());
    for(int i=0; i < body->numJoints(); ++i){
        pose2->setJointDisplacement(i, body->joint(i)->q());
    }
    pseq->insert(iter, time, pose2);
    
    interpolator.setBody(body);
    interpolator.setPoseSeq(pseq);
    interpolator.update();
}


static void getInterpolatedPose(PoseSeqInterpolator& interpolator, double time, Body* body)
{
    interpolator.interpolate(time);
    interpolator.getBaseLinkPosition(body->rootLink()->T());

    for(int i=0; i < body->numJoints(); ++i){
        stdx::optional<double> q = interpolator.jointPosition(i);
        if(q){
            body->joint(i)->q() = *q;
        }
    }
}    
    

bool FootFixFilterDialog::fixFootPositions
(Body* body, const BodyMotion& orgMotion, double waistHeightDiff, double pausingTime, double transitionTime,
 BodyMotion& out_motion)
{
    const int numOrgFrames = orgMotion.numFrames();
    if(numOrgFrames == 0){
        return false;
    }
    
    LeggedBodyHelperPtr legged = getLeggedBodyHelper(body);
    if(!legged->isValid()){
        return false;
    } 

    Link* waistLink = body->rootLink();
    auto ik = legged->getFootBasedIK(waistLink);
    if(!ik){
        return false;
    }

    *body << orgMotion.frame(0);
    body->calcForwardKinematics();
    const int numFeet = legged->numFeet();
    Isometry3Array footPositions(numFeet);
    vector<Link*> extraFootJoints;
    for(int i=0; i < numFeet; ++i){
        Link* foot = legged->footLink(i);
        Vector3 rpy = rpyFromRot(foot->rotation());
        rpy[0] = 0.0;
        rpy[1] = 0.0;
        Isometry3& P = footPositions[i];
        P.linear() = rotFromRpy(rpy);
        double ankleHeight = -legged->centerOfSoleLocal(i).z();
        foot->translation().z() = ankleHeight;
        P.translation() = foot->translation();

	LinkTraverse extras(foot);
        for(int j=1; j < extras.size(); ++j){
            extraFootJoints.push_back(extras[j]);
        }
    }
    const Vector3 zmp = legged->centerOfSoles();

    const int numPausingFrames = pausingTime * orgMotion.frameRate();
    const int numTransitionFrames = transitionTime * orgMotion.frameRate();

    *body << orgMotion.frame(0);
    BodyState orgInitialState(*body);
    BodyState initialState;

    int phase2Begin = numPausingFrames;
    
    PoseSeqInterpolator interpolator1;
    int phase3Begin;
    if(transitionTime == 0.0){
        initialState = orgInitialState;
        phase3Begin = phase2Begin;
    } else {
        waistLink->setRotation(Matrix3::Identity());
        setStandardPose(body);
        initialState << *body;
        setTransition(interpolator1, body, initialState, orgInitialState, transitionTime);
        phase3Begin = phase2Begin + numTransitionFrames;
    }

    int phase4Begin = phase3Begin + numPausingFrames;
    int phase5Begin = phase4Begin + numOrgFrames;

    *body << orgMotion.frame(numOrgFrames - 1);
    BodyState orgFinalState(*body);

    int phase6Begin = phase5Begin + numPausingFrames;

    BodyState finalState;
    PoseSeqInterpolator interpolator2;
    int phase7Begin;
    if(transitionTime == 0.0){
        finalState = orgFinalState;
        phase7Begin = phase6Begin;
    } else {
        waistLink->setRotation(Matrix3::Identity());
        setStandardPose(body);
        finalState << *body;
        setTransition(interpolator2, body, orgFinalState, finalState, transitionTime);
        phase7Begin = phase6Begin + numTransitionFrames;
    }

    int phase7End = phase7Begin + numPausingFrames;

    const int numFrames = phase7End;
    out_motion.setFrameRate(orgMotion.frameRate());
    out_motion.setDimension(numFrames, orgMotion.numJoints(), orgMotion.numLinks());
    
    ZMPSeq& zmpSeq = *getOrCreateZMPSeq(out_motion);

    double dt = 1.0 / orgMotion.frameRate();

    for(int i=0; i < numFrames; ++i){

        if(i < phase2Begin){
            *body << initialState;
        } else if(i < phase3Begin){
            getInterpolatedPose(interpolator1, (i - phase2Begin) * dt, body);
        } else if(i < phase4Begin){
            *body << orgInitialState;
        } else if(i < phase5Begin){
            *body << orgMotion.frame(i - phase4Begin);
        } else if(i < phase6Begin){
            *body << orgFinalState;
        } else if(i < phase7Begin){
            getInterpolatedPose(interpolator2, (i - phase6Begin) * dt, body);
        } else {
            *body << finalState;
        }

        for(size_t j=0; j < extraFootJoints.size(); ++j){
            extraFootJoints[j]->q() = 0.0;
        }
        body->calcForwardKinematics();
        
        for(int j=0; j < numFeet; ++j){
            legged->footLink(j)->setPosition(footPositions[j]);
        }
        Vector3 p = waistLink->translation();
        p.z() += waistHeightDiff;
        ik->calcInverseKinematics(p, waistLink->rotation());

        out_motion.frame(i) << *body;

        zmpSeq[i] = zmp;
    }

    return true;
}
