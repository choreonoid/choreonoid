/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include "BodyMotionControllerItem.h"
#include "BodyMotionItem.h"
#include <cnoid/ItemManager>
#include <cnoid/ItemList>
#include <cnoid/ItemTreeView>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace cnoid {

class BodyMotionControllerItemImpl
{
public:
    BodyMotionControllerItem* self;
    BodyMotionItemPtr motionItem;
    MultiValueSeqPtr qseqRef;
    BodyPtr body;
    int currentFrame;
    int lastFrame;
    int numJoints;

    BodyMotionControllerItemImpl(BodyMotionControllerItem* self);
    bool initialize(ControllerItemIO* io);
    void output();
};

}


void BodyMotionControllerItem::initializeClass(ExtensionManager* ext)
{
    ext->itemManager().registerClass<BodyMotionControllerItem>(N_("BodyMotionControllerItem"));
    ext->itemManager().addCreationPanel<BodyMotionControllerItem>();
}


BodyMotionControllerItem::BodyMotionControllerItem()
{
    impl = new BodyMotionControllerItemImpl(this);
}


BodyMotionControllerItem::BodyMotionControllerItem(const BodyMotionControllerItem& org)
    : ControllerItem(org)
{
    impl = new BodyMotionControllerItemImpl(this);
}


BodyMotionControllerItemImpl::BodyMotionControllerItemImpl(BodyMotionControllerItem* self)
    : self(self)
{

}


BodyMotionControllerItem::~BodyMotionControllerItem()
{
    delete impl;
}


bool BodyMotionControllerItem::initialize(ControllerItemIO* io)
{
    return impl->initialize(io);
}


bool BodyMotionControllerItemImpl::initialize(ControllerItemIO* io)
{
    ItemList<BodyMotionItem> motionItems;
    if(!motionItems.extractChildItems(self)){
        self->putMessage(
            str(boost::format(_("Any body motion item for %1% is not found."))
                % self->name()));
        return false;
    }
    motionItem = motionItems.front();
    // find the first checked item
    ItemTreeView* itv = ItemTreeView::instance();
    for(int i=0; i < motionItems.size(); ++i){
        if(itv->isItemChecked(motionItems[i])){
            motionItem = motionItems[i];
            break;
        }
    }

    qseqRef = motionItem->jointPosSeq();

    body = io->body();
    currentFrame = 0;
    lastFrame = std::max(0, qseqRef->numFrames() - 1);
    numJoints = std::min(body->numJoints(), qseqRef->numParts());
    if(qseqRef->numFrames() == 0){
        self->putMessage(_("Reference motion is empty()."));
        return false;
    }
    if(fabs(qseqRef->frameRate() - (1.0 / io->worldTimeStep())) > 1.0e-6){
        self->putMessage(_("The frame rate of the reference motion is different from the world frame rate."));
        return false;
    }

    MultiSE3SeqPtr lseq = motionItem->linkPosSeq();
    if(lseq->numParts() > 0 && lseq->numFrames() > 0){
        SE3& p = lseq->at(0, 0);
        Link* rootLink = body->rootLink();
        rootLink->p() = p.translation();
        rootLink->R() = p.rotation().toRotationMatrix();
    }
    self->output();
    body->calcForwardKinematics();
    io->fixInitialBodyState();
    
    return true;
}


bool BodyMotionControllerItem::start(ControllerItemIO* io)
{
    control();
    return true;
}
    
    
double BodyMotionControllerItem::timeStep() const
{
    return impl->qseqRef->timeStep();
}
        

void BodyMotionControllerItem::input()
{

}


bool BodyMotionControllerItem::control()
{
    if(++impl->currentFrame > impl->lastFrame){
        impl->currentFrame = impl->lastFrame;
        return false;
    }
    return true;
}
        

void BodyMotionControllerItemImpl::output()
{
    int prevFrame = std::max(currentFrame - 1, 0);
    int nextFrame = std::min(currentFrame + 1, lastFrame);
            
    MultiValueSeq::Frame q0 = qseqRef->frame(prevFrame);
    MultiValueSeq::Frame q1 = qseqRef->frame(currentFrame);
    MultiValueSeq::Frame q2 = qseqRef->frame(nextFrame);
    
    double dt = qseqRef->timeStep();
    double dt2 = dt * dt;
    
    for(int i=0; i < numJoints; ++i){
        Link* joint = body->joint(i);
        joint->q() = q1[i];
        joint->dq() = (q2[i] - q1[i]) / dt;
        joint->ddq() = (q2[i] - 2.0 * q1[i] + q0[i]) / dt2;
    }
}


void BodyMotionControllerItem::output()
{
    impl->output();
}


void BodyMotionControllerItem::stop()
{
    impl->qseqRef.reset();
    impl->motionItem = 0;
    impl->body = 0;
}


void BodyMotionControllerItem::onDisconnectedFromRoot()
{
    stop();
}
    

Item* BodyMotionControllerItem::doDuplicate() const
{
    return new BodyMotionControllerItem(*this);
}


void BodyMotionControllerItem::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("Control mode"), string("High-gain"));
}


bool BodyMotionControllerItem::store(Archive& archive)
{
    return true;
}
    

bool BodyMotionControllerItem::restore(const Archive& archive)
{
    return true;
}
