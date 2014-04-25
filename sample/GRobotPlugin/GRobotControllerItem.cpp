/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "GRobotControllerItem.h"
#include "GRobotBar.h"
#include "GRobotController.h"
#include <cnoid/ItemTreeView>
#include <cnoid/MessageView>
#include <cnoid/TimeBar>
#include <cnoid/Archive>
#include <cnoid/BodyItem>
#include <cnoid/BodyMotionItem>
#include <boost/bind.hpp>
#include "gettext.h"

using namespace std;
using namespace boost;
using namespace cnoid;


GRobotControllerItem::GRobotControllerItem()
    : mv(MessageView::mainInstance())

{
    controller = new GRobotController();
}


GRobotControllerItem::GRobotControllerItem(const GRobotControllerItem& org)
    : Item(org),
      mv(MessageView::mainInstance())
{
    controller = new GRobotController(*org.controller);
}


GRobotControllerItem::~GRobotControllerItem()
{
    delete controller;
}


ItemPtr GRobotControllerItem::doDuplicate() const
{
    return new GRobotControllerItem(*this);
}


void GRobotControllerItem::onConnectedToRoot()
{
    checkToggledConnection =
        ItemTreeView::mainInstance()->sigCheckToggled(this).connect(
            bind(&GRobotControllerItem::onCheckToggled, this, _1));
}


void GRobotControllerItem::onDisconnectedFromRoot()
{
    connections.disconnect();
    checkToggledConnection.disconnect();
    setSyncMode(false);
}


void GRobotControllerItem::onCheckToggled(bool on)
{
    if(connections.empty() && on){
        GRobotBar* bar = GRobotBar::instance();

        connections.add(
            bar->sigServoSwitchRequest().connect(
                bind(&GRobotController::switchServos, controller, _1)));

        connections.add(
            bar->sigPoseSendRequest().connect(
                bind(&GRobotControllerItem::requestToSendPose, this, 1.0)));
        
        connections.add(
            bar->sigSyncModeToggled().connect(
                bind(&GRobotControllerItem::setSyncMode, this, _1)));

        if(bar->isSyncMode()){
            setSyncMode(true);
        }
    } else {
        connections.disconnect();
        setSyncMode(false);
    }
}


void GRobotControllerItem::onPositionChanged()
{
    bodyItem = findOwnerItem<BodyItem>();
}


void GRobotControllerItem::setSyncMode(bool on)
{
    kinematicStateChangeConnection.disconnect();
    playbackInitilizeConnection.disconnect();

    if(on && bodyItem){

        kinematicStateChangeConnection =
            bodyItem->sigKinematicStateChanged().connect(
                bind(&GRobotControllerItem::requestToSendPose, this, 0.1));
        requestToSendPose(1.0);

        playbackInitilizeConnection =
            TimeBar::instance()->sigPlaybackInitialized().connect(
                bind(&GRobotControllerItem::onPlaybackInitialized, this, _1));
    }
}


void GRobotControllerItem::requestToSendPose(double transitionTime)
{
    if(bodyItem){
        const BodyPtr& body = bodyItem->body();
        for(int i=0; i < body->numJoints(); ++i){
            controller->setJointAngle(i, body->joint(i)->q());
        }
        controller->requestToSendPose(transitionTime);
    }
}


bool GRobotControllerItem::onPlaybackInitialized(double time)
{
    if(controller->isPlayingMotion()){
        controller->stopMotion();
    }

    playbackConnections.disconnect();

    if(bodyItem){
        BodyMotionItemPtr motionItem =
            ItemTreeView::mainInstance()->selectedSubItem<BodyMotionItem>(bodyItem);
        if(motionItem){
            MultiValueSeqPtr qseq = motionItem->jointPosSeq();
            if(qseq->numFrames() > 0 && qseq->numParts() == controller->numJoints()){
                if(controller->setMotion(&qseq->at(0, 0), qseq->numFrames(), qseq->getTimeStep())){
                    TimeBar* timeBar = TimeBar::instance();
                    playbackConnections.add(
                        timeBar->sigPlaybackStarted().connect(
                            bind(&GRobotControllerItem::onPlaybackStarted, this, _1)));
                    playbackConnections.add(
                        timeBar->sigPlaybackStopped().connect(
                            bind(&GRobotControllerItem::onPlaybackStopped, this)));
                }
            }
        }
    }
    return true;
}


void GRobotControllerItem::onPlaybackStarted(double time)
{
    controller->startMotion(time);
}


void GRobotControllerItem::onPlaybackStopped()
{
    controller->stopMotion();
    playbackConnections.disconnect();
}
                

void GRobotControllerItem::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("Port"), controller->portDevice(),
                bind(&GRobotControllerItem::onPortPropertyChanged, this, _1));
}


bool GRobotControllerItem::onPortPropertyChanged(const std::string& port)
{
    controller->setPortDevice(port);
    return true;
}


bool GRobotControllerItem::store(Archive& archive)
{
    archive.write("port", controller->portDevice());
    return true;
}


bool GRobotControllerItem::restore(const Archive& archive)
{
    controller->setPortDevice(archive.get("port", controller->portDevice()));
    return true;
}
