/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include "BodyMotionControllerItem.h"
#include "BodyMotionItem.h"
#include <cnoid/ItemManager>
#include <cnoid/ItemList>
#include <cnoid/ItemTreeView>
#include <cnoid/ControllerIO>
#include <cnoid/MessageView>
#include <fmt/format.h>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

namespace cnoid {

class BodyMotionControllerItemImpl
{
public:
    BodyMotionControllerItem* self;
    BodyMotionItemPtr motionItem;
    shared_ptr<MultiValueSeq> qseqRef;
    BodyPtr body;
    int currentFrame;
    int numJoints;

    BodyMotionControllerItemImpl(BodyMotionControllerItem* self);
    bool initialize(ControllerIO* io);
};

}


void BodyMotionControllerItem::initializeClass(ExtensionManager* ext)
{
    ItemManager& itemManager = ext->itemManager();
    itemManager.registerClass<BodyMotionControllerItem>(N_("BodyMotionControllerItem"));
    itemManager.addCreationPanel<BodyMotionControllerItem>();
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


bool BodyMotionControllerItem::initialize(ControllerIO* io)
{
    return impl->initialize(io);
}


bool BodyMotionControllerItemImpl::initialize(ControllerIO* io)
{
    auto mv = MessageView::instance();
    
    ItemList<BodyMotionItem> motionItems;
    if(!motionItems.extractChildItems(self)){
        mv->putln(
            format(_("Any body motion item for {} is not found."), self->name()),
            MessageView::ERROR);
        return false;
    }
    motionItem = motionItems.front();
    // find the first checked item
    ItemTreeView* itv = ItemTreeView::instance();
    for(size_t i=0; i < motionItems.size(); ++i){
        if(itv->isItemChecked(motionItems[i])){
            motionItem = motionItems[i];
            break;
        }
    }

    qseqRef = motionItem->motion()->jointPosSeq();

    if(qseqRef->numFrames() == 0){
        mv->putln(
            format(_("{0} for {1} is empty."), motionItem->name(), self->name()),
            MessageView::ERROR);
        return false;
    }
    if(fabs(qseqRef->frameRate() - (1.0 / io->timeStep())) > 1.0e-6){
        mv->putln(
            format(_("The frame rate of {} is different from the world frame rate."), motionItem->name()),
            MessageView::ERROR);
        return false;
    }

    body = io->body();

    auto lseq = motionItem->motion()->linkPosSeq();
    if(lseq->numParts() > 0 && lseq->numFrames() > 0){
        SE3& p = lseq->at(0, 0);
        Link* rootLink = body->rootLink();
        rootLink->p() = p.translation();
        rootLink->R() = p.rotation().toRotationMatrix();
    }

    numJoints = std::min(body->numJoints(), qseqRef->numParts());

    MultiValueSeq::Frame q = qseqRef->frame(0);
    for(int i=0; i < numJoints; ++i){
        auto joint = body->joint(i);
        joint->setActuationMode(Link::JOINT_DISPLACEMENT);
        joint->q() = joint->q_target() = q[i];
    }
    body->calcForwardKinematics();

    currentFrame = 0;

    return true;
}


bool BodyMotionControllerItem::start()
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
    return (impl->currentFrame < impl->qseqRef->numFrames());
}
        

void BodyMotionControllerItem::output()
{
    if(impl->currentFrame < impl->qseqRef->numFrames()){
        MultiValueSeq::Frame q = impl->qseqRef->frame(impl->currentFrame);
        for(int i=0; i < impl->numJoints; ++i){
            impl->body->joint(i)->q_target() = q[i];
        }
        impl->currentFrame++;
    }
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
