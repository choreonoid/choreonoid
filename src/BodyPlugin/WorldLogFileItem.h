/**
   @file
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_PLUGIN_WORLD_LOG_FILE_ITEM_H
#define CNOID_BODY_PLUGIN_WORLD_LOG_FILE_ITEM_H

#include <cnoid/Item>
#include "exportdecl.h"

namespace cnoid {

class SE3;
class DeviceState;

class CNOID_EXPORT WorldLogFileItem : public Item
{
public:
    static void initializeClass(ExtensionManager* ext);

    WorldLogFileItem();
    WorldLogFileItem(const WorldLogFileItem& org);
    ~WorldLogFileItem();

    bool setLogFile(const std::string& filename);
    const std::string& logFile() const;

    void setTimeStampSuffixEnabled(bool on);
    bool isTimeStampSuffixEnabled() const;

    void setRecordingFrameRate(double rate);
    double recordingFrameRate() const;

    void clearOutput();
    void beginHeaderOutput();
    int outputBodyHeader(const std::string& name);
    void endHeaderOutput();
    void beginFrameOutput(double time);
    void beginBodyStateOutput();
    void outputLinkPositions(SE3* positions, int size);
    void outputJointPositions(double* values, int size);
    void beginDeviceStateOutput();
    void outputDeviceState(DeviceState* state);
    void endDeviceStateOutput();
    void endBodyStateOutput();
    void endFrameOutput();

    int numBodies() const;
    const std::string& bodyName(int bodyIndex) const;

    bool recallStateAtTime(double time);
    void invalidateLastStateConsistency();

    virtual void notifyUpdate() override;

protected:
    virtual Item* doDuplicate() const override;
    virtual void onTreePathChanged() override;
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;
    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;

private:
    class Impl;
    Impl* impl;
};

typedef ref_ptr<WorldLogFileItem> WorldLogFileItemPtr;

}

#endif
