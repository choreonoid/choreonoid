/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_MEDIAPLUGIN_MEDIA_ITEM_H_INCLUDED
#define CNOID_MEDIAPLUGIN_MEDIA_ITEM_H_INCLUDED

#include <cnoid/Item>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT MediaItem : public Item
{
public:
    static void initialize(ExtensionManager* ext);
            
    MediaItem();
    MediaItem(const MediaItem& org);
            
    bool setUri(const std::string& uri);
    const std::string& uri() const { return uri_; }
            
    bool setFilepath(const std::string& filepath);
    const std::string& filepath() const { return filepath_; }
            
    void setOffsetTime(double offset);
    double offsetTime() const { return offsetTime_; }
            
    const std::string& lastErrorMessage() const { return lastErrorMessage_; }
            
protected:
    virtual ~MediaItem();
            
    virtual ItemPtr doDuplicate() const;
    virtual void doPutProperties(PutPropertyFunction& putProperty);
    virtual bool store(Archive& archive);
    virtual bool restore(const Archive& archive);
            
private:
    std::string uri_;
    std::string filepath_;
    double offsetTime_;
    std::string lastErrorMessage_;
};
        
typedef ref_ptr<MediaItem> MediaItemPtr;
}

#endif
