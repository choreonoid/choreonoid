/**
   @file
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_MEDIA_PLUGIN_AUDIO_ITEM_H
#define CNOID_MEDIA_PLUGIN_AUDIO_ITEM_H

#include <cnoid/Item>
#include <memory>
#include <vector>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT AudioItem : public Item
{
public:
    static void initialize(ExtensionManager* ext);
        
    AudioItem();
    AudioItem(const AudioItem& org);

    int numChannels() {
        return numChannels_;
    }

    int numFrames() {
        return samplingData_->size() / numChannels_;
    }
            
    double timeLength() {
        return numFrames() / samplingRate_;
    }

    double samplingRate() {
        return samplingRate_;
    }

    void setOffsetTime(double offset);

    double offsetTime() {
        return offsetTime_;
    }

    int offsetFrame() {
        return offsetTime_ * samplingRate_;
    }

    const std::vector<float>& samplingData() {
        return *samplingData_;
    }

protected:
    ~AudioItem();

    virtual Item* doDuplicate() const;
    virtual void doPutProperties(PutPropertyFunction& putProperty);
    virtual bool store(Archive& archive);
    virtual bool restore(const Archive& archive);

private:
    std::shared_ptr< std::vector<float> > samplingData_;
    double offsetTime_;
    int numChannels_;
    double samplingRate_;
    std::string title;
    std::string copyright;
    std::string artists;
    std::string comment;
    std::string date;

    void clear();
    bool loadAudioFile(const std::string& filename, std::ostream& os, Item* parentItem);
};
    
typedef ref_ptr<AudioItem> AudioItemPtr;

};

#endif
