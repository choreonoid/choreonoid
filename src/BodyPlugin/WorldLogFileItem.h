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
class WorldLogFileItemImpl;


class CNOID_EXPORT WorldLogFileItem : public Item
{
public:
    static void initializeClass(ExtensionManager* ext);

    WorldLogFileItem();
    WorldLogFileItem(const WorldLogFileItem& org);
    ~WorldLogFileItem();

    bool setLogFileName(const std::string& filename);
    const std::string& logFileName() const;

    double recordingFrameRate() const;

    void clearOutput();
    void beginHeaderOutput();
    int outputBodyHeader(const std::string& name);
    void endHeaderOutput();
    void beginFrameOutput(double time);
    void beginBodyStatusOutput();
    void outputLinkPositions(SE3* positions, int size);
    void outputJointPositions(double* values, int size);
    void beginDeviceStatusOutput();
    void outputDeviceStatus(DeviceState* status);
    void endDeviceStatusOutput();
    void endBodyStatusOutput();
    void endFrameOutput();

    int numBodies() const;
    const std::string& bodyName(int bodyIndex) const;

    bool recallStatusAtTime(double time);
    void invalidateLastStatusConsistency();

    virtual void notifyUpdate();

protected:
    virtual ItemPtr doDuplicate() const;
    virtual void onPositionChanged();
    virtual void doPutProperties(PutPropertyFunction& putProperty);
    virtual bool store(Archive& archive);
    virtual bool restore(const Archive& archive);

private:
    WorldLogFileItemImpl* impl;
};

typedef ref_ptr<WorldLogFileItem> WorldLogFileItemPtr;

}

#endif
