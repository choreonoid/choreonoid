/**
   @file
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_MULTI_DEVICE_STATE_SEQ_H_INCLUDED
#define CNOID_BODY_MULTI_DEVICE_STATE_SEQ_H_INCLUDED

#include "Device.h"
#include <cnoid/MultiSeq>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT MultiDeviceStateSeq : public MultiSeq<DeviceStatePtr>
{
    typedef MultiSeq<DeviceStatePtr> BaseSeqType;
            
public:
    typedef std::shared_ptr<MultiDeviceStateSeq> Ptr;

    static const std::string& key();

    MultiDeviceStateSeq();
    MultiDeviceStateSeq(int numFrames, int numDevices = 1);
    MultiDeviceStateSeq(const MultiDeviceStateSeq& org);
    virtual ~MultiDeviceStateSeq();

    using BaseSeqType::operator=;
    MultiDeviceStateSeq& operator=(const MultiDeviceStateSeq& rhs);
    virtual AbstractSeqPtr cloneSeq() const;        
};

typedef MultiDeviceStateSeq::Ptr MultiDeviceStateSeqPtr;

class BodyMotion;

CNOID_EXPORT MultiDeviceStateSeqPtr getMultiDeviceStateSeq(const BodyMotion& motion);
CNOID_EXPORT MultiDeviceStateSeqPtr getOrCreateMultiDeviceStateSeq(BodyMotion& motion);
CNOID_EXPORT void clearMultiDeviceStateSeq(BodyMotion& motion);
}

#endif
