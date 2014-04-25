/**
   @file
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_MEDIAPLUGIN_AUDIO_ITEM_H_INCLUDED
#define CNOID_MEDIAPLUGIN_AUDIO_ITEM_H_INCLUDED

#include <cnoid/Item>
#include "exportdecl.h"

namespace cnoid {

class ExtensionManager;
    
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

    virtual ItemPtr doDuplicate() const;
    virtual void doPutProperties(PutPropertyFunction& putProperty);
    virtual bool store(Archive& archive);
    virtual bool restore(const Archive& archive);

private:

    boost::shared_ptr< std::vector<float> > samplingData_;
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
