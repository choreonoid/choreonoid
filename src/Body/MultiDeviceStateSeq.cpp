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
static const string contentName("MultiDeviceStateSeq");
}


MultiDeviceStateSeq::MultiDeviceStateSeq()
    : BaseSeqType("MultiDeviceStateSeq")
{
    setSeqContentName(contentName);
}


MultiDeviceStateSeq::MultiDeviceStateSeq(int numFrames, int numDevices)
    : BaseSeqType("MultiDeviceStateSeq", numFrames, numDevices)
{
    setSeqContentName(contentName);
}


/**
   \todo implement deep copy
*/
MultiDeviceStateSeq::MultiDeviceStateSeq(const MultiDeviceStateSeq& org)
    : BaseSeqType(org),
      deviceNames(org.deviceNames)
{
    
}


/**
   \todo implement deep copy
*/
MultiDeviceStateSeq& MultiDeviceStateSeq::operator=(const MultiDeviceStateSeq& rhs)
{
    if(this != &rhs){
        BaseSeqType::operator=(rhs);
        deviceNames = rhs.deviceNames;
    }
    return *this;
}


/**
   \todo implement deep copy
*/
std::shared_ptr<AbstractSeq> MultiDeviceStateSeq::cloneSeq() const
{
    return std::make_shared<MultiDeviceStateSeq>(*this);
}


MultiDeviceStateSeq::~MultiDeviceStateSeq()
{

}


const std::string& MultiDeviceStateSeq::key()
{
    return contentName;
}


std::shared_ptr<MultiDeviceStateSeq> cnoid::getMultiDeviceStateSeq(const BodyMotion& motion)
{
    return motion.extraSeq<MultiDeviceStateSeq>(contentName);
}


std::shared_ptr<MultiDeviceStateSeq> cnoid::getOrCreateMultiDeviceStateSeq(BodyMotion& motion)
{
    return motion.getOrCreateExtraSeq<MultiDeviceStateSeq>(contentName);
}


void cnoid::clearMultiDeviceStateSeq(BodyMotion& motion)
{
    motion.clearExtraSeq(contentName);
}


void MultiDeviceStateSeq::initialize(const DeviceList<>& devices)
{
    const int n = devices.size();
    setNumParts(n);
    deviceNames.resize(n);
    for(int i=0; i < n; ++i){
        deviceNames[i] = devices[i]->name();
    }
}


int MultiDeviceStateSeq::partIndex(const std::string& partLabel) const
{
    for(size_t i=0; i < deviceNames.size(); ++i){
        if(deviceNames[i] == partLabel){
            return i;
        }
    }
    return -1;
}


const std::string& MultiDeviceStateSeq::partLabel(int partIndex) const
{
    if(partIndex < deviceNames.size()){
        return deviceNames[partIndex];
    }
    return AbstractMultiSeq::partLabel(partIndex);
}


bool MultiDeviceStateSeq::doWriteSeq(YAMLWriter& writer, std::function<void()> additionalPartCallback)
{
    double version = writer.info("formatVersion", 3.0);
    if(version >= 1.0 && version < 2.0){
        return false; // not supported for the format verions 1
    }

    return AbstractMultiSeq::doWriteSeq(
        writer,
        [&](){
            if(additionalPartCallback) additionalPartCallback();

            const int n = numFrames();
            const int m = numParts();
            if(n * m > 0){
                writer.putKey("components");
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
    const int stateSize = state->stateSize();
    vector<double> buf(stateSize);
    writer.startMapping();
    writer.putKeyValue("type", "DeviceStateSeq");
    writer.putKeyValue("content", string(state->typeName()) + "StateSeq");
    writer.putKey("name");
    writer.putDoubleQuotedString(partLabel(deviceIndex));

    state = nullptr;
    vector<int> validFrameIndices;
    validFrameIndices.reserve(seq.size());
    for(int i=0; i < seq.size(); ++i){
        if(seq[i] != state){
            validFrameIndices.push_back(i);
            state = seq[i];
        }
    }
    const int numFrames = validFrameIndices.size();
    writer.putKeyValue("numFrames", numFrames);
    const bool hasFrameTime = validFrameIndices.size() < seq.size();
    writer.putKeyValue("hasFrameTime", hasFrameTime);

    writer.putKey("frames");
    writer.startListing();

    const double dt = timeStep();
    for(int i=0; i < numFrames; ++i){
        int index = validFrameIndices[i];
        seq[index]->writeState(&buf.front());
        writer.startFlowStyleListing();
        if(hasFrameTime){
            writer.putScalar(dt * index);
        }
        for(int j=0; j < stateSize; ++j){
            writer.putScalar(buf[j]);
        }
        writer.endListing();
    }
        
    writer.endListing();

    writer.endMapping();
}
