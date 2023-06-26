#include "BodyPositionSeq.h"
#include "Body.h"

using namespace std;
using namespace cnoid;


BodyPositionSeqFrame::BodyPositionSeqFrame()
{

}


BodyPositionSeqFrame::BodyPositionSeqFrame(const BodyPositionSeqFrame& org)
    : data(org.data)
{
    pdata = data.data();
}


BodyPositionSeqFrame::BodyPositionSeqFrame(BodyPositionSeqFrame&& org)
    : data(std::move(org.data))
{
    pdata = data.data();
}


BodyPositionSeq::BodyPositionSeq(int numFrames)
    : Seq<BodyPositionSeqFrame>("BodyPositionSeq", numFrames)
{
    numLinkPositionsHint_ = 0;
    numJointDisplacementsHint_ = 0;
}


BodyPositionSeq::BodyPositionSeq(const BodyPositionSeq& org)
    : Seq<BodyPositionSeqFrame>(org)
{
    numLinkPositionsHint_ = org.numLinkPositionsHint_;
    numJointDisplacementsHint_ = org.numJointDisplacementsHint_;
}


std::shared_ptr<AbstractSeq> BodyPositionSeq::cloneSeq() const
{
    return make_shared<BodyPositionSeq>(*this);
}


static void setBodyPositionToBodyPositionSeqFrameBlock(const Body& body, BodyPositionSeqFrameBlock block)
{
    int numLinks = std::min(body.numLinks(), block.numLinkPositions());
    for(int i=0; i < numLinks; ++i){
        block.linkPosition(i).set(body.link(i)->position());
    }
    int numJoints = std::min(body.numJoints(), block.numJointDisplacements());
    auto displacements = block.jointDisplacements();
    for(int i=0; i < numJoints; ++i){
        displacements[i] = body.joint(i)->q();
    }
}


static void updateBodyPositionWithSeqFrameBlockToBody(Body& body, const BodyPositionSeqFrameBlock block)
{
    int numLinks = std::min(body.numLinks(), block.numLinkPositions());
    for(int i=0; i < numLinks; ++i){
        auto linkPosition = block.linkPosition(i);
        auto link = body.link(i);
        link->setTranslation(linkPosition.translation());
        link->setRotation(linkPosition.rotation());
    }
    int numJoints = std::min(body.numJoints(), block.numJointDisplacements());
    auto displacements = block.jointDisplacements();
    for(int i=0; i < numJoints; ++i){
        body.joint(i)->q() = displacements[i];
    }
}


/**
   \note The buffer of the frame must be allocated and the allocated number of link positions and joint displacements
   are copied regardless of the actual number of links and joints in the body
*/
BodyPositionSeqFrame& cnoid::operator<<(BodyPositionSeqFrame& frame, const Body& body)
{
    setBodyPositionToBodyPositionSeqFrameBlock(body, frame);
    return frame;
}


BodyPositionSeqFrame& cnoid::operator>>(BodyPositionSeqFrame& frame, Body& body)
{
    updateBodyPositionWithSeqFrameBlockToBody(body, frame);
    return frame;
}


const BodyPositionSeqFrame& cnoid::operator>>(const BodyPositionSeqFrame& frame, Body& body)
{
    updateBodyPositionWithSeqFrameBlockToBody(body, frame);
    return frame;
}


Body& cnoid::operator<<(Body& body, const BodyPositionSeqFrame& frame)
{
    updateBodyPositionWithSeqFrameBlockToBody(body, frame);
    return body;
}


/**
   \note The buffer of the frame must be allocated and the allocated number of link positions and joint displacements
   are copied regardless of the actual number of links and joints in the body
*/
Body& cnoid::operator>>(Body& body, BodyPositionSeqFrame& frame)
{
    setBodyPositionToBodyPositionSeqFrameBlock(body, frame);
    return body;
}


/**
   \note The buffer of the frame must be allocated and the allocated number of link positions and joint displacements
   are copied regardless of the actual number of links and joints in the body
*/
const Body& cnoid::operator>>(const Body& body, BodyPositionSeqFrame& frame)
{
    setBodyPositionToBodyPositionSeqFrameBlock(body, frame);
    return body;
}
