/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "ReferencedObjectSeq.h"

using namespace cnoid;


ReferencedObjectSeq::ReferencedObjectSeq(int nFrames)
    : BaseSeqType("ReferencedObjectSeq", nFrames)
{

}


ReferencedObjectSeq::ReferencedObjectSeq(const ReferencedObjectSeq& org)
    : BaseSeqType(org)
{

}


std::shared_ptr<AbstractSeq> ReferencedObjectSeq::cloneSeq() const
{
    return std::make_shared<ReferencedObjectSeq>(*this);
}


ReferencedObjectSeq::~ReferencedObjectSeq()
{

}
