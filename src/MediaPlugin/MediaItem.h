/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_MEDIAPLUGIN_MEDIA_ITEM_H
#define CNOID_MEDIAPLUGIN_MEDIA_ITEM_H

#include <cnoid/Item>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT MediaItem : public Item
{
public:
    static void initialize(ExtensionManager* ext);
            
    MediaItem();
    MediaItem(const MediaItem& org);
            
    bool setMediaURI(const std::string& uri);
    const std::string& mediaURI() const { return mediaURI_; }
            
    bool setMediaFilePath(const std::string& path);
    const std::string& mediaFilePath() const { return mediaFilePath_; }
            
    void setOffsetTime(double offset);
    double offsetTime() const { return offsetTime_; }
            
    const std::string& lastErrorMessage() const { return lastErrorMessage_; }
            
protected:
    virtual ~MediaItem();
            
    virtual Item* doDuplicate() const;
    virtual void doPutProperties(PutPropertyFunction& putProperty);
    virtual bool store(Archive& archive);
    virtual bool restore(const Archive& archive);
            
private:
    std::string mediaURI_;
    std::string mediaFilePath_;
    double offsetTime_;
    std::string lastErrorMessage_;
};
        
typedef ref_ptr<MediaItem> MediaItemPtr;
}

#endif
