/**
   @author Shin'ichiro NAKAOKA
*/

#include "KinematicFaultChecker.h"
#include "BodyItem.h"
#include "BodyMotionItem.h"
#include "WorldItem.h"
#include "LinkSelectionView.h"
#include <cnoid/BodyState>
#include <cnoid/Archive>
#include <cnoid/MainWindow>
#include <cnoid/MenuManager>
#include <cnoid/TimeBar>
#include <cnoid/ItemTreeView>
#include <cnoid/MessageView>
#include <cnoid/SpinBox>
#include <cnoid/Buttons>
#include <cnoid/CheckBox>
#include <cnoid/Dialog>
#include <cnoid/Separator>
#include <cnoid/EigenUtil>
#include <cnoid/BodyCollisionDetector>
#include <cnoid/AISTCollisionDetector>
#include <cnoid/IdPair>
#include <QDialogButtonBox>
#include <QBoxLayout>
#include <QFrame>
#include <QLabel>
#include <fmt/format.h>
#include <map>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

namespace {

bool USE_DUPLICATED_BODY = false;

KinematicFaultChecker* checkerInstance = 0;

#if defined(_MSC_VER) && _MSC_VER < 1800
inline long lround(double x) {
    return static_cast<long>((x > 0.0) ? floor(x + 0.5) : ceil(x -0.5));
}
#endif
}

namespace cnoid {

class KinematicFaultCheckerImpl : public Dialog
{
public:
    MessageView& mes;
    std::ostream& os;
        
    CheckBox positionCheck;
    DoubleSpinBox angleMarginSpin;
    DoubleSpinBox translationMarginSpin;
    CheckBox velocityCheck;

    QButtonGroup radioGroup;
    RadioButton allJointsRadio;
    RadioButton selectedJointsRadio;
    RadioButton nonSelectedJointsRadio;
        
    DoubleSpinBox velocityLimitRatioSpin;
    CheckBox collisionCheck;

    CheckBox onlyTimeBarRangeCheck;

    int numFaults;
    vector<int> lastPosFaultFrames;
    vector<int> lastVelFaultFrames;
    typedef IdPair<CollisionDetector::GeometryHandle> GeometryPair;
    typedef std::map<GeometryPair, int> LastCollisionFrameMap;
    LastCollisionFrameMap lastCollisionFrames;

    double frameRate;
    double angleMargin;
    double translationMargin;
    double velocityLimitRatio;

    KinematicFaultCheckerImpl();
    bool store(Archive& archive);
    void restore(const Archive& archive);
    void apply();
    int checkFaults(
        BodyItem* bodyItem, BodyMotionItem* motionItem, std::ostream& os,
        bool checkPosition, bool checkVelocity, bool checkCollision,
        vector<bool> linkSelection, double beginningTime, double endingTime);
    void putJointPositionFault(int frame, Link* joint, std::ostream& os);
    void putJointVelocityFault(int frame, Link* joint, std::ostream& os);
    void putSelfCollision(Body* body, int frame, const CollisionPair& collisionPair, std::ostream& os);
};
}


void KinematicFaultChecker::initialize(ExtensionManager* ext)
{
    if(!checkerInstance){
        checkerInstance = ext->manage(new KinematicFaultChecker());

        MenuManager& mm = ext->menuManager();
        mm.setPath("/Tools");
        mm.addItem(_("Kinematic Fault Checker"))
            ->sigTriggered().connect([](){ checkerInstance->impl->show(); });
        
        ext->setProjectArchiver(
            "KinematicFaultChecker",
            [](Archive& archive){ return checkerInstance->impl->store(archive); },
            [](const Archive& archive) { return checkerInstance->impl->restore(archive); });
    }
}


KinematicFaultChecker* KinematicFaultChecker::instance()
{
    return checkerInstance;
}


KinematicFaultChecker::KinematicFaultChecker()
{
    impl = new KinematicFaultCheckerImpl();
}


KinematicFaultChecker::~KinematicFaultChecker()
{
    delete impl;
}


KinematicFaultCheckerImpl::KinematicFaultCheckerImpl()
    : mes(*MessageView::mainInstance()),
      os(mes.cout())
{
    setWindowTitle(_("Kinematic Fault Checker"));
    
    QVBoxLayout* vbox = new QVBoxLayout();
    setLayout(vbox);
    
    QHBoxLayout* hbox = new QHBoxLayout();
    
    positionCheck.setText(_("Joint position check"));
    positionCheck.setChecked(true);
    hbox->addWidget(&positionCheck);
    hbox->addSpacing(10);

    hbox->addWidget(new QLabel(_("Angle margin")));
    angleMarginSpin.setDecimals(2);
    angleMarginSpin.setRange(-99.99, 99.99);
    angleMarginSpin.setSingleStep(0.01);
    hbox->addWidget(&angleMarginSpin);
    hbox->addWidget(new QLabel("[deg]"));
    hbox->addSpacing(10);

    hbox->addWidget(new QLabel(_("Translation margin")));
    translationMarginSpin.setDecimals(4);
    translationMarginSpin.setRange(-9.9999, 9.9999);
    translationMarginSpin.setSingleStep(0.0001);
    hbox->addWidget(&translationMarginSpin);
    hbox->addWidget(new QLabel("[m]"));

    hbox->addStretch();
    vbox->addLayout(hbox);
    hbox = new QHBoxLayout();
    
    velocityCheck.setText(_("Joint velocity check"));
    velocityCheck.setChecked(true);
    hbox->addWidget(&velocityCheck);
    hbox->addSpacing(10);

    hbox->addWidget(new QLabel(_("Limit ratio")));
    velocityLimitRatioSpin.setDecimals(0);
    velocityLimitRatioSpin.setRange(1.0, 100.0);
    velocityLimitRatioSpin.setValue(100.0);
    hbox->addWidget(&velocityLimitRatioSpin);
    hbox->addWidget(new QLabel("%"));

    hbox->addStretch();
    vbox->addLayout(hbox);
    hbox = new QHBoxLayout();

    radioGroup.addButton(&allJointsRadio);
    radioGroup.addButton(&selectedJointsRadio);
    radioGroup.addButton(&nonSelectedJointsRadio);
    
    allJointsRadio.setText(_("All joints"));
    allJointsRadio.setChecked(true);
    hbox->addWidget(&allJointsRadio);

    selectedJointsRadio.setText(_("Selected joints"));
    hbox->addWidget(&selectedJointsRadio);

    nonSelectedJointsRadio.setText(_("Non-selected joints"));
    hbox->addWidget(&nonSelectedJointsRadio);

    hbox->addStretch();
    vbox->addLayout(hbox);
    vbox->addWidget(new HSeparator);
    hbox = new QHBoxLayout();
    
    collisionCheck.setText(_("Self-collision check"));
    collisionCheck.setChecked(true);
    hbox->addWidget(&collisionCheck);

    hbox->addStretch();
    vbox->addLayout(hbox);

    vbox->addWidget(new HSeparator);
    
    hbox = new QHBoxLayout();
    onlyTimeBarRangeCheck.setText(_("Time bar's range only"));
    onlyTimeBarRangeCheck.setChecked(false);
    hbox->addWidget(&onlyTimeBarRangeCheck);
    hbox->addStretch();
    vbox->addLayout(hbox);

    vbox->addWidget(new HSeparator);

    PushButton* applyButton = new PushButton(_("&Apply"));
    applyButton->setDefault(true);
    QDialogButtonBox* buttonBox = new QDialogButtonBox(this);
    buttonBox->addButton(applyButton, QDialogButtonBox::AcceptRole);
    applyButton->sigClicked().connect([&](){ apply(); });
    
    vbox->addWidget(buttonBox);
}


bool KinematicFaultCheckerImpl::store(Archive& archive)
{
    archive.write("checkJointPositions", positionCheck.isChecked());
    archive.write("angleMargin", angleMarginSpin.value());
    archive.write("translationMargin", translationMarginSpin.value());
    archive.write("checkJointVelocities", velocityCheck.isChecked());
    archive.write("velocityLimitRatio", velocityLimitRatioSpin.value());
    archive.write("targetJoints",
                  (allJointsRadio.isChecked() ? "all" :
                   (selectedJointsRadio.isChecked() ? "selected" : "non-selected")));
    archive.write("checkSelfCollisions", collisionCheck.isChecked());
    archive.write("onlyTimeBarRange", onlyTimeBarRangeCheck.isChecked());
    return true;
}


void KinematicFaultCheckerImpl::restore(const Archive& archive)
{
    positionCheck.setChecked(archive.get("checkJointPositions", positionCheck.isChecked()));
    angleMarginSpin.setValue(archive.get("angleMargin", angleMarginSpin.value()));
    translationMarginSpin.setValue(archive.get("translationMargin", translationMarginSpin.value()));
    velocityCheck.setChecked(archive.get("checkJointVelocities", velocityCheck.isChecked()));
    velocityLimitRatioSpin.setValue(archive.get("velocityLimitRatio", velocityLimitRatioSpin.value()));
    string target;
    if(archive.read("targetJoints", target)){
        if(target == "all"){
            allJointsRadio.setChecked(true);
        } else if(target == "selected"){
            selectedJointsRadio.setChecked(true);
        } else if(target == "non-selected"){
            nonSelectedJointsRadio.setChecked(true);
        }
    }
    collisionCheck.setChecked(archive.get("checkSelfCollisions", collisionCheck.isChecked()));
    onlyTimeBarRangeCheck.setChecked(archive.get("onlyTimeBarRange", onlyTimeBarRangeCheck.isChecked()));
}


void KinematicFaultCheckerImpl::apply()
{
    ItemList<BodyMotionItem> items = ItemTreeView::mainInstance()->selectedItems<BodyMotionItem>();
    if(items.empty()){
        mes.notify(_("No BodyMotionItems are selected."));
    } else {
        for(size_t i=0; i < items.size(); ++i){
            BodyMotionItem* motionItem = items.get(i);
            BodyItem* bodyItem = motionItem->findOwnerItem<BodyItem>();
            if(!bodyItem){
                mes.notify(format(_("{} is not owned by any BodyItem. Check skiped."), motionItem->name()));
            } else {
                mes.putln();
                mes.notify(format(_("Applying the Kinematic Fault Checker to {} ..."),
                                  motionItem->headItem()->name()));
                
                vector<bool> linkSelection;
                if(selectedJointsRadio.isChecked()){
                    linkSelection = LinkSelectionView::mainInstance()->linkSelection(bodyItem);
                } else if(nonSelectedJointsRadio.isChecked()){
                    linkSelection = LinkSelectionView::mainInstance()->linkSelection(bodyItem);
                    linkSelection.flip();
                } else {
                    linkSelection.resize(bodyItem->body()->numLinks(), true);
                }
                
                double beginningTime = 0.0;
                double endingTime = motionItem->motion()->getTimeLength();
                std::numeric_limits<double>::max();
                if(onlyTimeBarRangeCheck.isChecked()){
                    TimeBar* timeBar = TimeBar::instance();
                    beginningTime = timeBar->minTime();
                    endingTime = timeBar->maxTime();
                }
                
                int n = checkFaults(bodyItem, motionItem, mes.cout(),
                                    positionCheck.isChecked(),
                                    velocityCheck.isChecked(),
                                    collisionCheck.isChecked(),
                                    linkSelection,
                                    beginningTime, endingTime);
                
                if(n > 0){
                    if(n == 1){
                        mes.notify(_("A fault has been detected."));
                    } else {
                        mes.notify(format(_("{} faults have been detected."), n));
                    }
                } else {
                    mes.notify(_("No faults have been detected."));
                }
            }
        }
    }
}


/**
   @return Number of detected faults
*/
int KinematicFaultChecker::checkFaults
(BodyItem* bodyItem, BodyMotionItem* motionItem, std::ostream& os, double beginningTime, double endingTime)
{
    vector<bool> linkSelection(bodyItem->body()->numLinks(), true);
    return impl->checkFaults(
        bodyItem, motionItem, os, true, true, true, linkSelection, beginningTime, endingTime);
}


int KinematicFaultCheckerImpl::checkFaults
(BodyItem* bodyItem, BodyMotionItem* motionItem, std::ostream& os,
 bool checkPosition, bool checkVelocity, bool checkCollision, vector<bool> linkSelection,
 double beginningTime, double endingTime)
{
    numFaults = 0;

    auto body = bodyItem->body();
    auto motion = motionItem->motion();
    auto qseq = motion->jointPosSeq();;
    auto pseq = motion->linkPosSeq();
    
    if((!checkPosition && !checkVelocity && !checkCollision) || body->isStaticModel() || !qseq->getNumFrames()){
        return numFaults;
    }

    BodyState orgKinematicState;
    
    if(USE_DUPLICATED_BODY){
        body = body->clone();
    } else {
        bodyItem->storeKinematicState(orgKinematicState);
    }

    BodyCollisionDetector bodyCollisionDetector;
    WorldItem* worldItem = bodyItem->findOwnerItem<WorldItem>();
    if(worldItem){
        bodyCollisionDetector.setCollisionDetector(worldItem->collisionDetector()->clone());
    } else {
        bodyCollisionDetector.setCollisionDetector(new AISTCollisionDetector);
    }

    bodyCollisionDetector.addBody(body, true);
    bodyCollisionDetector.makeReady();

    const int numJoints = std::min(body->numJoints(), qseq->numParts());
    const int numLinks = std::min(body->numLinks(), pseq->numParts());

    frameRate = motion->frameRate();
    double stepRatio2 = 2.0 / frameRate;
    angleMargin = radian(angleMarginSpin.value());
    translationMargin = translationMarginSpin.value();
    velocityLimitRatio = velocityLimitRatioSpin.value() / 100.0;

    int beginningFrame = std::max(0, (int)(beginningTime * frameRate));
    int endingFrame = std::min((motion->numFrames() - 1), (int)lround(endingTime * frameRate));

    lastPosFaultFrames.clear();
    lastPosFaultFrames.resize(numJoints, std::numeric_limits<int>::min());
    lastVelFaultFrames.clear();
    lastVelFaultFrames.resize(numJoints, std::numeric_limits<int>::min());
    lastCollisionFrames.clear();

    if(checkCollision){
        Link* root = body->rootLink();
        root->p().setZero();
        root->R().setIdentity();
    }
        
    for(int frame = beginningFrame; frame <= endingFrame; ++frame){

        int prevFrame = (frame == beginningFrame) ? beginningFrame : frame - 1;
        int nextFrame = (frame == endingFrame) ? endingFrame : frame + 1;

        for(int i=0; i < numJoints; ++i){
            Link* joint = body->joint(i);
            double q = qseq->at(frame, i);
            joint->q() = q;
            if(joint->index() >= 0 && linkSelection[joint->index()]){
                if(checkPosition){
                    bool fault = false;
                    if(joint->isRotationalJoint()){
                        fault = (q > (joint->q_upper() - angleMargin) || q < (joint->q_lower() + angleMargin));
                    } else if(joint->isSlideJoint()){
                        fault = (q > (joint->q_upper() - translationMargin) || q < (joint->q_lower() + translationMargin));
                    }
                    if(fault){
                        putJointPositionFault(frame, joint, os);
                    }
                }
                if(checkVelocity){
                    double dq = (qseq->at(nextFrame, i) - qseq->at(prevFrame, i)) / stepRatio2;
                    joint->dq() = dq;
                    if(dq > (joint->dq_upper() * velocityLimitRatio) || dq < (joint->dq_lower() * velocityLimitRatio)){
                        putJointVelocityFault(frame, joint, os);
                    }
                }
            }
        }

        if(checkCollision){

            Link* link = body->link(0);
            if(!pseq->empty())
            {
                const SE3& p = pseq->at(frame, 0);
                link->p() = p.translation();
                link->R() = p.rotation().toRotationMatrix();
            }
            else
            {
                link->p() = Vector3d(0., 0., 0.);
                link->R() = Matrix3d::Identity();
            }

            body->calcForwardKinematics();

            for(int i=1; i < numLinks; ++i){
                link = body->link(i);
                if(!pseq->empty())
                {
                    const SE3& p = pseq->at(frame, i);
                    link->p() = p.translation();
                    link->R() = p.rotation().toRotationMatrix();
                }
            }

            bodyCollisionDetector.updatePositions();

            bodyCollisionDetector.detectCollisions(
                [&](const CollisionPair& collisionPair){
                    putSelfCollision(body, frame, collisionPair, os);
                });
        }
    }

    if(!USE_DUPLICATED_BODY){
        bodyItem->restoreKinematicState(orgKinematicState);
    }

    return numFaults;
}


void KinematicFaultCheckerImpl::putJointPositionFault(int frame, Link* joint, std::ostream& os)
{
    if(frame > lastPosFaultFrames[joint->jointId()] + 1){
        double q, l, u, m;
        if(joint->isRotationalJoint()){
            q = degree(joint->q());
            l = degree(joint->q_lower());
            u = degree(joint->q_upper());
            m = degree(angleMargin);
        } else {
            q = joint->q();
            l = joint->q_lower();
            u = joint->q_upper();
            m = translationMargin;
        }

        if(m != 0.0){
            os << format(_("{0:7.3f} [s]: Position limit over of {1} ({2} is beyond the range ({3} , {4}) with margin {5}.)"),
                         (frame / frameRate), joint->name(), q, l, u, m) << endl;
        } else {
            os << format(_("{0:7.3f} [s]: Position limit over of {1} ({2} is beyond the range ({3} , {4}).)"),
                         (frame / frameRate), joint->name(), q, l, u) << endl;
        }

        numFaults++;
    }
    lastPosFaultFrames[joint->jointId()] = frame;
}


void KinematicFaultCheckerImpl::putJointVelocityFault(int frame, Link* joint, std::ostream& os)
{
    if(frame > lastVelFaultFrames[joint->jointId()] + 1){
        double dq, l, u;
        if(joint->isRotationalJoint()){
            dq = degree(joint->dq());
            l = degree(joint->dq_lower());
            u = degree(joint->dq_upper());
        } else {
            dq = joint->dq();
            l = joint->dq_lower();
            u = joint->dq_upper();
        }

        double r = (dq < 0.0) ? (dq / l) : (dq / u);
        r *= 100.0;

        os << format(_("{0:7.3f} [s]: Velocity limit over of {1} ({2} is {3:.0f}% of the range ({4} , {5}).)"),
                     (frame / frameRate), joint->name(), dq, r, l, u) << endl;
        
        numFaults++;
    }
    lastVelFaultFrames[joint->jointId()] = frame;
}


void KinematicFaultCheckerImpl::putSelfCollision(Body* body, int frame, const CollisionPair& collisionPair, std::ostream& os)
{
    bool putMessage = false;
    GeometryPair gPair(collisionPair.geometries());
    auto p = lastCollisionFrames.find(gPair);
    if(p == lastCollisionFrames.end()){
        putMessage = true;
        lastCollisionFrames[gPair] = frame;
    } else {
        if(frame > p->second + 1){
            putMessage = true;
        }
        p->second = frame;
    }

    if(putMessage){
        Link* link0 = static_cast<Link*>(collisionPair.object(0));
        Link* link1 = static_cast<Link*>(collisionPair.object(1));
        os << format(_("{0:7.3f} [s]: Collision between {1} and {2}"),
                     (frame / frameRate), link0->name(), link1->name()) << endl;
        numFaults++;
    }
}
