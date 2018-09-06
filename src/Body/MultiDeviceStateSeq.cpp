/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "MultiDeviceStateSeq.h"
#include "BodyMotion.h"
#include <cnoid/YAMLWriter>

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


bool MultiDeviceStateSeq::doWriteSeq(YAMLWriter& writer, std::function<void()> additionalPartCallback)
{
    double version = writer.info("formatVersion", 0.0);
    if(version >= 1.0 && version < 2.0){
        return false; // not supported for the format verions 1
    }

    return AbstractSeq::doWriteSeq(
        writer,
        [&](){
            if(additionalPartCallback) additionalPartCallback();

            const int n = numFrames();
            const int m = numParts();
            if(n * m > 0){
                writer.putKey("devices");
                writer.startListing();
                for(int i=0; i < m; ++i){
                    writeDeviceStateSeq(writer, i);
                }
                writer.endListing();
            }
        });
}


void MultiDeviceStateSeq::writeDeviceStateSeq(YAMLWriter& writer, int deviceIndex)
{
    auto seq = part(deviceIndex);
    DeviceState* state = seq[0];
    const int size = state->stateSize();
    vector<double> buf(size);
    writer.startMapping();
    writer.putKeyValue("type", state->typeName());
    state = nullptr;
    
    writer.putKey("frames");
    writer.startListing();

    double dt = timeStep();
    const int numFrames = seq.size();
    for(int i=0; i < numFrames; ++i){
        if(seq[i] != state){
            state = seq[i];
            state->writeState(&buf.front());
            writer.startFlowStyleListing();
            writer.putScalar(dt * i);
            for(int j=0; j < size; ++j){
                writer.putScalar(buf[j]);
            }
            writer.endListing();
        }
    }

    writer.endListing();

    writer.endMapping();
}
