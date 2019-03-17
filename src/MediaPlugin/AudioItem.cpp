/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "AudioItem.h"
#include <cnoid/ItemManager>
#include <cnoid/Archive>
#include <fmt/format.h>

#ifdef CNOID_MEDIA_PLUGIN_USE_LIBSNDFILE
#include <sndfile.h>
#endif

#include "gettext.h"

using namespace std;
using namespace std::placeholders;
using namespace cnoid;
using fmt::format;

namespace {
std::shared_ptr< std::vector<float> > emptySamplingData;
}


void AudioItem::initialize(ExtensionManager* ext)
{
    static bool initialized = false;
    if(!initialized){
        emptySamplingData = std::make_shared< std::vector<float> >();
        ext->itemManager().registerClass<AudioItem>(N_("AudioItem"));

#ifdef CNOID_MEDIA_PLUGIN_USE_LIBSNDFILE
        ext->itemManager().addLoader<AudioItem>(_("Audio File"), "AUDIO-GENERIC", "wav;ogg",
                                                std::bind(&AudioItem::loadAudioFile, _1, _2, _3, _4));
#endif
        initialized = true;
    }
}


AudioItem::AudioItem()
    : samplingData_(emptySamplingData)
{
    numChannels_ = 1;
    samplingRate_ = 44100.0;
    offsetTime_ = 0.0;
}


AudioItem::AudioItem(const AudioItem& org)
    : Item(org),
      samplingData_(org.samplingData_),
      offsetTime_(org.offsetTime_),
      numChannels_(org.numChannels_),
      samplingRate_(org.samplingRate_),
      title(org.title),
      copyright(org.copyright),
      artists(org.artists),
      comment(org.comment),
      date(org.date)
{
    
}


Item* AudioItem::doDuplicate() const
{
    return new AudioItem(*this);
}


AudioItem::~AudioItem()
{

}


void AudioItem::clear()
{
    numChannels_ = 1;
    samplingData_ = emptySamplingData;
    title.clear();
    copyright.clear();
    artists.clear();
    comment.clear();
    date.clear();

    clearFileInformation();
}


void AudioItem::setOffsetTime(double offset)
{
    offsetTime_ = offset;
    notifyUpdate();
}


#ifdef CNOID_MEDIA_PLUGIN_USE_LIBSNDFILE

namespace {
void setTextInfo(SNDFILE* sndfile, int type, std::string& out_text)
{
    const char* text = sf_get_string(sndfile, type);
    if(text){
        out_text = text;
    } else {
        out_text.clear();
    }
}
}


bool AudioItem::loadAudioFile(const std::string& filename, std::ostream& os, Item* parentItem)
{
    clear();
    
    bool result = false;
    
    SF_INFO sfinfo;
    std::memset(&sfinfo, 0, sizeof(sfinfo));

    SNDFILE* sndfile = sf_open(filename.c_str(), SFM_READ, &sfinfo);

    if(!sndfile){
        os << sf_strerror(sndfile);

    } else if(sfinfo.channels < 1 || sfinfo.channels > 2){
        os << format("channels = {:d}", sfinfo.channels);

    } else {
        if(false){
            os << format(" format mask = {0:x}, sub mask = {1:x}, endian = {2:x}\n",
                         (sfinfo.format & SF_FORMAT_TYPEMASK),
                         (sfinfo.format & SF_FORMAT_SUBMASK),
                         (sfinfo.format & SF_FORMAT_ENDMASK));
        }

        numChannels_ = sfinfo.channels;
        samplingRate_ = sfinfo.samplerate;

        samplingData_ = std::make_shared< std::vector<float> >(sfinfo.frames * sfinfo.channels);
        sf_count_t framesRead = sf_readf_float(sndfile, &(*samplingData_)[0], sfinfo.frames);

        if(framesRead < sfinfo.frames){
            samplingData_->resize(framesRead * sfinfo.channels);
        }

        setTextInfo(sndfile, SF_STR_TITLE, title);
        setTextInfo(sndfile, SF_STR_COPYRIGHT, copyright);
        setTextInfo(sndfile, SF_STR_ARTIST, artists);
        setTextInfo(sndfile, SF_STR_COMMENT, comment);
        setTextInfo(sndfile, SF_STR_DATE, date);

        result = true;
    }

    if(sndfile){
        sf_close(sndfile);
    }

    return result;
}

#else

bool AudioItem::loadAudioFile(const std::string& filename, std::ostream& os, Item* parentItem)
{
    os << "Loading an AudioItem is not supported by this platform.";
    return false;
}

#endif


void AudioItem::doPutProperties(PutPropertyFunction& putProperty)
{
    if(!samplingData_->empty()){
        putProperty("title", title);
        putProperty("length", timeLength());
        putProperty("offset", offsetTime(), std::bind(&AudioItem::setOffsetTime, this, _1), true);
        putProperty("channels", numChannels());
        putProperty("sampling rate", samplingRate());
        if(!copyright.empty()) putProperty("copyright", copyright);
        if(!artists.empty()) putProperty("artists", artists);
        if(!comment.empty()) putProperty("comment", comment);
        if(!date.empty()) putProperty("date", date);
    }
}


bool AudioItem::store(Archive& archive)
{
    if(!filePath().empty()){
        archive.writeRelocatablePath("file", filePath());
        if(!fileFormat().empty()){
            archive.write("format", fileFormat());
        }
    }
    archive.write("offsetTime", offsetTime_);
    return true;
}


bool AudioItem::restore(const Archive& archive)
{
    bool restored = false;

    string filepath, format;
    
    if(!archive.readRelocatablePath("file", filepath)){
        /* for backward compatibility */
        archive.readRelocatablePath("audioFile", filepath);
    }
    if(!filepath.empty()){
        archive.read("format", format);
        restored = load(filepath, format);
    }

    offsetTime_ = archive.get("offsetTime", 0.0);
    
    return restored;
}
