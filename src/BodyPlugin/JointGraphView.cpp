#include "JointGraphView.h"
#include <cnoid/ViewManager>
#include <QBoxLayout>
#include "gettext.h"

using namespace std;
using namespace cnoid;


void JointGraphView::initializeClass(ExtensionManager* ext)
{
    ext->viewManager().registerClass<JointGraphView>(
        N_("JointGraphView"), N_("Joint Trajectories"));
}


JointGraphView::JointGraphView()
{
    auto vbox = new QVBoxLayout;
    vbox->addWidget(&graph);
    setLayout(vbox);
}


void JointGraphView::addTrajectory(ItemInfo& info, Link* link, std::shared_ptr<BodyPositionSeq> seq)
{
    int id = link->jointId();
    if(id < 0 || id >= seq->numJointDisplacementsHint()){
        return;
    }
    
    shared_ptr<GraphDataHandler> handler = make_shared<GraphDataHandler>();
    handler->setLabel(link->name());
    handler->setValueLimits(link->q_lower(), link->q_upper());
    handler->setVelocityLimits(link->dq_lower(), link->dq_upper());
    handler->setFrameProperties(seq->numFrames(), seq->frameRate());
    handler->setDataRequestCallback(
        [this, &info, link](int frame, int size, double* out_values){
            onDataRequest(info, link->jointId(), frame, size, out_values); });
    handler->setDataModifiedCallback(
        [this, &info, link](int frame, int size, double* values){
            onDataModified(info, link->jointId(), frame, size, values); });
    graph.addDataHandler(handler);
    info.handlers.push_back(handler);
}


void JointGraphView::onDataRequest(ItemInfo& info, int jointId, int frameIndex0, int size, double* out_values)
{
    for(int i=0; i < size; ++i){
        int frameIndex = i + frameIndex0;
        auto& frame = info.seq->frame(frameIndex);
        if(jointId < frame.numJointDisplacements()){
            out_values[i] = frame.jointDisplacement(jointId);
        }
    }
}


void JointGraphView::onDataModified(ItemInfo& info, int jointId, int frameIndex0, int size, double* values)
{
    for(int i=0; i < size; ++i){
        int frameIndex = i + frameIndex0;
        auto& frame = info.seq->frame(frameIndex);
        frame.allocate(frame.numLinkPositions(), std::max(jointId + 1, frame.numJointDisplacements()));
        frame.jointDisplacement(jointId) = values[i];
    }
    
    info.connections.block();
    info.item->notifyUpdate();
    info.connections.unblock();
}
