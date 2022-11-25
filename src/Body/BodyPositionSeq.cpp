#include "BodyPositionSeq.h"

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

}


BodyPositionSeq::BodyPositionSeq(const BodyPositionSeq& org)
    : Seq<BodyPositionSeqFrame>(org)
{

}
