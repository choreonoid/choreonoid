/**
   @file
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_MULTI_DEVICE_STATE_SEQ_H
#define CNOID_BODY_MULTI_DEVICE_STATE_SEQ_H

#include "Device.h"
#include <cnoid/DeviceList>
#include <cnoid/MultiSeq>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT MultiDeviceStateSeq : public MultiSeq<DeviceStatePtr>
{
    typedef MultiSeq<DeviceStatePtr> BaseSeqType;
            
public:
    static const std::string& key();

    MultiDeviceStateSeq();
    MultiDeviceStateSeq(int numFrames, int numDevices = 1);
    MultiDeviceStateSeq(const MultiDeviceStateSeq& org);
    virtual ~MultiDeviceStateSeq();

    using BaseSeqType::operator=;
    MultiDeviceStateSeq& operator=(const MultiDeviceStateSeq& rhs);
    virtual std::shared_ptr<AbstractSeq> cloneSeq() const;

    void initialize(const DeviceList<>& devices);

    virtual int partIndex(const std::string& partLabel) const override;
    virtual const std::string& partLabel(int partIndex) const override;

protected:
    virtual bool doWriteSeq(YAMLWriter& writer, std::function<void()> additionalPartCallback) override;

private:
    void writeDeviceStateSeq(YAMLWriter& writer, int deviceIndex);

    std::vector<std::string> deviceNames;
};

class BodyMotion;

CNOID_EXPORT std::shared_ptr<MultiDeviceStateSeq> getMultiDeviceStateSeq(const BodyMotion& motion);
CNOID_EXPORT std::shared_ptr<MultiDeviceStateSeq> getOrCreateMultiDeviceStateSeq(BodyMotion& motion);
CNOID_EXPORT void clearMultiDeviceStateSeq(BodyMotion& motion);
}

#endif
