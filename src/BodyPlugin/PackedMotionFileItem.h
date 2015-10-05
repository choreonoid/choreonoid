/**
   @file
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_JVRC_PLUGIN_PACKED_MOTION_FILE_ITEM_H
#define CNOID_JVRC_PLUGIN_PACKED_MOTION_FILE_ITEM_H

#include <cnoid/Item>
#include "exportdecl.h"

namespace cnoid {

class PackedMotionFileItemImpl;

class CNOID_EXPORT PackedMotionFileItem : public AbstractMultiSeqItem
{
public:
    static void initializeClass(ExtensionManager* ext);

    PackedMotionFileItem();
    PackedMotionFileItem(const PackedMotionFileItem& org);
    ~PackedMotionFileItem();

    void setMotionFileName(const std::string& filename);

    int numTargetBodyItems() const;
    BodyItem* targetBodyItem(int index);

    char* seek(double time);

    

    virtual void notifyUpdate();

protected:
    virtual ItemPtr doDuplicate() const;
    virtual bool store(Archive& archive);
    virtual bool restore(const Archive& archive);

private:
    PackedMotionFileItemImpl* impl;
};

typedef ref_ptr<PackedMotionFileItem> PackedMotionFileItemPtr;

}

#endif
