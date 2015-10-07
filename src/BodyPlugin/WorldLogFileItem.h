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
class Body;
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

    void clear();
    void addBodyToRecord(Body* body);

    int numBodies() const;
    const std::string& modelName(int bodyIndex) const;
    const std::string& bodyName(int bodyIndex) const;

    void beginFrameOutput(double time);
    void beginBodyStatusOutput();
    void outputLinkPositions(SE3* positions, int size);
    void outputJointValues(double* values, int size);
    void endFrameOutput();

    bool readFrame(double time);

    virtual void notifyUpdate();

protected:
    virtual ItemPtr doDuplicate() const;
    virtual void doPutProperties(PutPropertyFunction& putProperty);
    virtual bool store(Archive& archive);
    virtual bool restore(const Archive& archive);

private:
    WorldLogFileItemImpl* impl;
};

typedef ref_ptr<WorldLogFileItem> WorldLogFileItemPtr;

}

#endif
