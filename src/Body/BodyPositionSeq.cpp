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


BodyPositionSeqFrame& BodyPositionSeqFrame::operator=(const BodyPositionSeqFrame& rhs)
{
    data = rhs.data;
    pdata = data.data();
    return *this;
}


BodyPositionSeqFrame& BodyPositionSeqFrame::operator=(BodyPositionSeqFrame&& rhs)
{
    data = std::move(rhs.data);
    pdata = data.data();
    return *this;
}


BodyPositionSeq::BodyPositionSeq()
    : Seq<BodyPositionSeqFrame>("BodyPositionSeq")
{

}


BodyPositionSeq::BodyPositionSeq(const BodyPositionSeq& org)
    : Seq<BodyPositionSeqFrame>(org)
{

}
