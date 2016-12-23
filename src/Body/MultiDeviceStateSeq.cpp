/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "MultiDeviceStateSeq.h"
#include "BodyMotion.h"

using namespace std;
using namespace cnoid;

namespace {
static const string mdskey("Devices");
}


MultiDeviceStateSeq::MultiDeviceStateSeq()
    : BaseSeqType("MultiDeviceStateSeq")
{
    setSeqContentName(mdskey);
}


MultiDeviceStateSeq::MultiDeviceStateSeq(int numFrames, int numDevices)
    : BaseSeqType("MultiDeviceStateSeq", numFrames, numDevices)
{
    setSeqContentName(mdskey);
}


/**
   \todo implement deep copy
*/
MultiDeviceStateSeq::MultiDeviceStateSeq(const MultiDeviceStateSeq& org)
    : BaseSeqType(org)
{
    
}


/**
   \todo implement deep copy
*/
MultiDeviceStateSeq& MultiDeviceStateSeq::operator=(const MultiDeviceStateSeq& rhs)
{
    if(this != &rhs){
        BaseSeqType::operator=(rhs);
    }
    return *this;
}


/**
   \todo implement deep copy
*/
AbstractSeqPtr MultiDeviceStateSeq::cloneSeq() const
{
    return std::make_shared<MultiDeviceStateSeq>(*this);
}


MultiDeviceStateSeq::~MultiDeviceStateSeq()
{

}


const std::string& MultiDeviceStateSeq::key()
{
    return mdskey;
}


MultiDeviceStateSeqPtr cnoid::getMultiDeviceStateSeq(const BodyMotion& motion)
{
    return motion.extraSeq<MultiDeviceStateSeq>(mdskey);
}


MultiDeviceStateSeqPtr cnoid::getOrCreateMultiDeviceStateSeq(BodyMotion& motion)
{
    return motion.getOrCreateExtraSeq<MultiDeviceStateSeq>(mdskey);
}


void cnoid::clearMultiDeviceStateSeq(BodyMotion& motion)
{
    motion.clearExtraSeq(mdskey);
}
