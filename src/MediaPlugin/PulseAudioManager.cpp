/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "PulseAudioManager.h"
#include "AudioItem.h"
#include "MediaUtil.h"
#include <cnoid/MenuManager>
#include <cnoid/ItemTreeView>
#include <cnoid/TimeBar>
#include <cnoid/LazyCaller>
#include <cnoid/Archive>
#include <pulse/pulseaudio.h>
#include <fmt/format.h>
#include <map>
#include <cmath>
//#include <iostream> // for debug
#include "gettext.h"

using namespace std;
using namespace std::placeholders;
using namespace cnoid;
using fmt::format;

namespace {

const bool TRACE_FUNCTIONS = true;

PulseAudioManager* pulseAudioManager = 0;

class Source
{
public:
    PulseAudioManagerImpl* manager;
    AudioItemPtr audioItem;
    pa_stream* stream;
    int currentFrame;
    bool isConnected;
    bool isActive_;
    bool doAdjustTime;
    bool initialWritingDone;
    bool hasAllFramesWritten;
    double volumeRatio;
    double timeToFinish;
    pa_sample_spec sampleSpec;
    vector<float> silenceBuf;
    pa_operation* operation;
    LazyCaller stopLater;
        
    Source(PulseAudioManagerImpl* manager, AudioItemPtr audioItem);
    ~Source();
    bool waitForOperation();
    bool initialize();
    bool connectStream();
    bool disconnectStream();
    void finalize();

    bool isActive() const {
        return isActive_;
    }
        
    void seek(double time);
    void initializePlayback(double time);
    void startPlayback();
    void write(size_t nbytes, bool isDoingInitialization);
    void onAllFramesWritten(pa_usec_t latency);
    void onBufferOverflow();
    void onBufferUnderflow();
    void adjustTime(const char* reason);
    void stop();
};

typedef std::shared_ptr<Source> SourcePtr;
}


namespace cnoid {

/**
   \todo To support timeout, mainloop should be manually handled instead of using threaded-mainloop
   because thread-mainloop does not provide any API which accepts a tiemout parameter.
*/
class PulseAudioManagerImpl
{
public:
    std::ostream& os;
    TimeBar* timeBar;
        
    /**
       In some environments, playback is somtimes delayed if the playback
       is done while keeping the stream connection after the previous playback.
       To avoid this, the connection is established and closed every time
       a playback starts and stops by default.
       Sine keeping the connection seems more natural from the viewpoint of API design,
       the code for doing that way has not been removed and can be enabled.
    */
    Action* connectionKeepCheck;
        
    Action* fullSyncPlaybackMenuItem;
    double maxTimeOfActiveSources;
    pa_threaded_mainloop* mainloop;
    pa_context* context;
    typedef map<AudioItemPtr, SourcePtr> SourceMap;
    SourceMap activeSources;
    Connection sigTimeChangedConnection;
        
    PulseAudioManagerImpl(ExtensionManager* ext);
    ~PulseAudioManagerImpl();
    void finalize();
    void onItemCheckToggled(Item* item, bool isChecked);
    void onFullSyncPlaybackToggled();
    bool onPlaybackInitialized(double time);
    void onPlaybackStarted(double time);
    void onPlaybackStopped(double time);
    bool onTimeChanged(double time);
    bool store(Archive& archive);
    void restore(const Archive& archive);
    bool playAudioFile(const std::string& filename, double volumeRatio);
    void onAudioFilePlaybackStopped(AudioItemPtr audioItem);
};


bool playAudioFile(const std::string& filename, double volumeRatio)
{    
    return PulseAudioManager::instance()->playAudioFile(filename, volumeRatio);
}

}


namespace {

void pa_context_state_callback(pa_context* context, void* userdata)
{
    PulseAudioManagerImpl* manager = (PulseAudioManagerImpl*)userdata;
    pa_threaded_mainloop_signal(manager->mainloop, 0);
}
}


void PulseAudioManager::initialize(ExtensionManager* ext)
{
    if(!pulseAudioManager){
        pulseAudioManager = ext->manage(new PulseAudioManager(ext));
    }
}


PulseAudioManager* PulseAudioManager::instance()
{
    assert(pulseAudioManager);
    return pulseAudioManager;
}


PulseAudioManager::PulseAudioManager(ExtensionManager* ext)
{
    impl = new PulseAudioManagerImpl(ext);
}


PulseAudioManagerImpl::PulseAudioManagerImpl(ExtensionManager* ext)
    : os(mvout())
{
    context = 0;
    
    mainloop = pa_threaded_mainloop_new();
    if(!mainloop){
        os << _("PulseAudio's main loop cannot be created.") << endl;
        return;
    }
    pa_threaded_mainloop_start(mainloop);

    pa_threaded_mainloop_lock(mainloop);

    context = pa_context_new(pa_threaded_mainloop_get_api(mainloop), "Choreonoid");
    if(!context){
        os << _("PulseAudio's context cannot be created.") << endl;
        finalize();
        return;
    }
    
    pa_context_set_state_callback(context, pa_context_state_callback, (void*)this);
    pa_context_connect(context, NULL, PA_CONTEXT_NOFLAGS, NULL);

    while(true){
        pa_context_state_t state = pa_context_get_state(context);
        if(state == PA_CONTEXT_READY){
            break;
        }
        if(state == PA_CONTEXT_FAILED || state == PA_CONTEXT_TERMINATED){
            os << _("PulseAudio's context cannot be connected to the server.") << endl;
            finalize();
            break;
        }
        pa_threaded_mainloop_wait(mainloop);
    }
    pa_threaded_mainloop_unlock(mainloop);

    if(!context){
        return;
    }

    maxTimeOfActiveSources = -std::numeric_limits<double>::max();

    MenuManager& mm = ext->menuManager();
    mm.setPath("/Options").setPath(N_("PulseAudio"));

    connectionKeepCheck = mm.addCheckItem(_("Keep Stream Connections"));

    fullSyncPlaybackMenuItem = mm.addCheckItem(_("Fully-Synchronized Audio Playback"));
    fullSyncPlaybackMenuItem->sigToggled().connect(
        std::bind(&PulseAudioManagerImpl::onFullSyncPlaybackToggled, this));

    ext->setProjectArchiver(
        "PulseAudioManager",
        std::bind(&PulseAudioManagerImpl::store, this, _1),
        std::bind(&PulseAudioManagerImpl::restore, this, _1));
    
    ItemTreeView::mainInstance()->sigCheckToggled().connect(
        std::bind(&PulseAudioManagerImpl::onItemCheckToggled, this, _1, _2));

    timeBar = TimeBar::instance();

    timeBar->sigPlaybackInitialized().connect(
        std::bind(&PulseAudioManagerImpl::onPlaybackInitialized, this, _1));

    timeBar->sigPlaybackStarted().connect(
        std::bind(&PulseAudioManagerImpl::onPlaybackStarted, this, _1));

    timeBar->sigPlaybackStopped().connect(
        std::bind(&PulseAudioManagerImpl::onPlaybackStopped, this, _1));

    // In some environments, the initial stream connection after starting up an
    // operating system produces an undesired playback timing offset.
    // To avoid that, an empty connection is done.
    AudioItemPtr audioItem = new AudioItem;
    audioItem->setName("initial connection");
    Source source(this, audioItem);
    if(source.initialize()){
        source.initializePlayback(0.0);
    }
}


PulseAudioManager::~PulseAudioManager()
{
    delete impl;
}


PulseAudioManagerImpl::~PulseAudioManagerImpl()
{
    activeSources.clear();
    finalize();
}


void PulseAudioManagerImpl::finalize()
{
    if(context){
        pa_threaded_mainloop_lock(mainloop);
        pa_context_disconnect(context);
        pa_context_unref(context);
        pa_threaded_mainloop_unlock(mainloop);
        context = 0;
    }
    if(mainloop){
        pa_threaded_mainloop_stop(mainloop);
        pa_threaded_mainloop_free(mainloop);
        mainloop = 0;
    }
}


/**
   This function must be thread safe
*/
bool PulseAudioManager::playAudioFile(const std::string& filename, double volumeRatio)
{
    return impl->playAudioFile(filename, volumeRatio);
}


bool PulseAudioManagerImpl::playAudioFile(const std::string& filename, double volumeRatio)
{
    AudioItemPtr audioItem = new AudioItem();
    if(audioItem->load(filename)){
        SourcePtr source = std::make_shared<Source>(this, audioItem);
        source->volumeRatio = volumeRatio;
        if(source->initialize()){
            if(timeBar->isDoingPlayback()){
                timeBar->stopPlayback();
            }
            activeSources[audioItem] = source;

            timeBar->sigPlaybackStopped().connect(
                std::bind(&PulseAudioManagerImpl::onAudioFilePlaybackStopped, this, audioItem));

            timeBar->setTime(0.0);
            timeBar->startPlayback();
            return true;
        }
    }
    return false;
}


void PulseAudioManagerImpl::onAudioFilePlaybackStopped(AudioItemPtr audioItem)
{
    activeSources.erase(audioItem);
}


void PulseAudioManagerImpl::onItemCheckToggled(Item* item, bool isChecked)
{
    if(AudioItem* audioItem = dynamic_cast<AudioItem*>(item)){
        if(isChecked){
            SourcePtr source = std::make_shared<Source>(this, audioItem);
            if(source->initialize()){
                activeSources[audioItem] = source;
                if(sigTimeChangedConnection.connected()){
                    source->initializePlayback(timeBar->realPlaybackTime());
                    source->startPlayback();
                }
            } else {
                os << audioItem->name() << " cannot be initialized." << endl;
            }
        } else {
            activeSources.erase(audioItem);
        }
    }
}


void PulseAudioManagerImpl::onFullSyncPlaybackToggled()
{

}


bool PulseAudioManagerImpl::onPlaybackInitialized(double time)
{
    if(!sigTimeChangedConnection.connected()){
        for(SourceMap::iterator p = activeSources.begin(); p != activeSources.end(); ++p){
            SourcePtr& source = p->second;
            source->initializePlayback(time);
        }
        sigTimeChangedConnection = timeBar->sigTimeChanged().connect(
            std::bind(&PulseAudioManagerImpl::onTimeChanged, this, _1));
    }
    return true;
}


void PulseAudioManagerImpl::onPlaybackStarted(double time)
{
    for(SourceMap::iterator p = activeSources.begin(); p != activeSources.end(); ++p){
        SourcePtr& source = p->second;
        source->startPlayback();
    }
}


void PulseAudioManagerImpl::onPlaybackStopped(double time)
{
    for(SourceMap::iterator p = activeSources.begin(); p != activeSources.end(); ++p){
        SourcePtr& source = p->second;
        source->stop();
    }
    sigTimeChangedConnection.disconnect();
}


bool PulseAudioManagerImpl::onTimeChanged(double time)
{
    bool isActive = false;

    for(SourceMap::iterator p = activeSources.begin(); p != activeSources.end(); ++p){
        SourcePtr& source = p->second;
        if(time >= source->timeToFinish){
            source->stop();
        } else if(source->isActive()){
            isActive |= true;
        }
    }
    return isActive;
}


bool PulseAudioManagerImpl::store(Archive& archive)
{
    archive.write("keepStreamConnection", connectionKeepCheck->isChecked());
    return true;
}


void PulseAudioManagerImpl::restore(const Archive& archive)
{
    connectionKeepCheck->setChecked(archive.get("keepStreamConnection", connectionKeepCheck->isChecked()));
}


Source::Source(PulseAudioManagerImpl* manager, AudioItemPtr audioItem)
    : manager(manager),
      audioItem(audioItem),
      stopLater(std::bind(&Source::stop, this))
{
    stream = 0;
    currentFrame = 0;
    isConnected = false;
    isActive_ = false;
    operation = 0;
    doAdjustTime = false;
    initialWritingDone = false;
    hasAllFramesWritten = false;
    volumeRatio = -1.0; // using the default volume
}


Source::~Source()
{
    finalize();
}


namespace {

void pa_stream_state_callback(pa_stream* stream, void* userdata)
{
    Source* source = (Source*)userdata;
    pa_threaded_mainloop_signal(source->manager->mainloop, 0);
}

void pa_stream_write_callback(pa_stream* stream,  size_t nbytes, void* userdata)
{
    Source* source = (Source*)userdata;
    source->write(nbytes, false);
}

void pa_stream_success_callback(pa_stream* stream, int success, void* userdata)
{
    Source* source = (Source*)userdata;
    pa_threaded_mainloop_signal(source->manager->mainloop, 0);
}

void pa_stream_overflow_notify_callback(pa_stream* stream, void* userdata)
{
    Source* source = (Source*)userdata;
    callLater(std::bind(&Source::onBufferOverflow, source));
}

void pa_stream_underflow_notify_callback(pa_stream* stream, void* userdata)
{
    Source* source = (Source*)userdata;
    if(!source->hasAllFramesWritten){
        callLater(std::bind(&Source::onBufferUnderflow, source));
    }
}
}


bool Source::waitForOperation()
{
    bool waited = false;
    
    if(operation){
        while(pa_operation_get_state(operation) == PA_OPERATION_RUNNING){
            waited = true;
            pa_threaded_mainloop_wait(manager->mainloop);
        }
        pa_operation_unref(operation);
        operation = 0;
    }
    return waited;
}


bool Source::initialize()
{
    pa_threaded_mainloop_lock(manager->mainloop);

    sampleSpec.format = PA_SAMPLE_FLOAT32NE;
    sampleSpec.rate = audioItem->samplingRate();
    sampleSpec.channels = audioItem->numChannels();

    bool initialized;
    
    if(manager->connectionKeepCheck->isChecked()){
        initialized = connectStream();
    } else {
        initialized = true;
    }

    pa_threaded_mainloop_unlock(manager->mainloop);
    
    if(!initialized){
        finalize();
    }

    return initialized;
}


bool Source::connectStream()
{
    if(!stream){
        stream = pa_stream_new(manager->context, audioItem->name().c_str(), &sampleSpec, NULL);
        if(!stream){
            manager->os << _("PulseAudio's stream cannot be created.") << endl;
            return false;
        }
        pa_stream_set_state_callback(stream, pa_stream_state_callback, (void*)this);
        isConnected = false;
    }
    if(isConnected){
        return true;
    }

    pa_buffer_attr* pattr = 0;
    //pa_stream_flags_t flags = PA_STREAM_START_CORKED;
    pa_stream_flags_t flags = (pa_stream_flags)(PA_STREAM_START_CORKED | PA_STREAM_AUTO_TIMING_UPDATE | PA_STREAM_INTERPOLATE_TIMING);
    

    /*
      pa_buffer_attr attr;
      attr.maxlength = -1;
      attr.tlength = -1;
      attr.prebuf = -1;
      attr.minreq = -1;
      attr.fragsize = -1;
      flags = (pa_stream_flags_t)(
      flags | PA_STREAM_ADJUST_LATENCY | PA_STREAM_INTERPOLATE_TIMING | PA_STREAM_AUTO_TIMING_UPDATE);
      pattr = &attr;
    */
    
    int result = pa_stream_connect_playback(stream, NULL, pattr, flags, NULL, NULL);
    if(result < 0){
        manager->os << "PulseAudio stream cannot be connected: "
                    << pa_strerror(result) << endl;
    } else {

        // wait for ready
        pa_stream_state_t state;
        while(true){
            state = pa_stream_get_state(stream);
            if(state == PA_STREAM_READY || state == PA_STREAM_FAILED || state == PA_STREAM_TERMINATED){
                break;
            }
            pa_threaded_mainloop_wait(manager->mainloop);
        }

        if(state != PA_STREAM_READY){
            manager->os << "PulseAudio stream cannot be ready." << endl;
            result = -1;
        } else {

            if(volumeRatio >= 0.0 && volumeRatio <= 1.0){
                const int sinkIndex = pa_stream_get_index(stream);
                //const int sinkIndex = pa_stream_get_device_index(stream);
                pa_volume_t v = pa_sw_volume_from_linear(volumeRatio);
                pa_cvolume volume;
                pa_cvolume_set(&volume, sampleSpec.channels, v);
                pa_context_set_sink_input_volume(manager->context, sinkIndex, &volume, NULL, NULL);
                //pa_context_set_sink_volume_by_index(manager->context, sinkIndex, &volume, NULL, NULL);
            }
            
            isConnected = true;
        }
    }

    return isConnected;
}


bool Source::disconnectStream()
{
    if(stream && isConnected){
        if(isConnected){
            pa_stream_set_state_callback(stream, 0, 0);
            pa_stream_disconnect(stream);
            isConnected = false;
        }
        pa_stream_unref(stream);
        stream = 0;
        return true;
    }

    return false;
}


void Source::finalize()
{
    if(stream){
        if(isActive_){
            stop();
        }
        pa_threaded_mainloop_lock(manager->mainloop);
        waitForOperation();
        disconnectStream();
        pa_threaded_mainloop_unlock(manager->mainloop);
    }
}


void Source::seek(double time)
{
    currentFrame = floor((time - audioItem->offsetTime()) * audioItem->samplingRate());
    hasAllFramesWritten = false;
}


void Source::initializePlayback(double time)
{
    pa_threaded_mainloop_lock(manager->mainloop);

    waitForOperation();
    
    if(!isActive_ && connectStream()){

        seek(time);
        initialWritingDone = false;
        size_t writableSize = pa_stream_writable_size(stream);
        
        if(writableSize <= 0){
            manager->os <<
                format(_("PulseAudio stream for {} cannot be written."), audioItem->name()) << endl;
            disconnectStream();
            
        } else {
            timeToFinish = std::numeric_limits<double>::max();
            write(writableSize, true);
            isActive_ = true;
            pa_stream_set_write_callback(stream, pa_stream_write_callback, (void*)this);
            pa_stream_set_overflow_callback(stream, pa_stream_overflow_notify_callback, (void*)this);
            pa_stream_set_underflow_callback(stream, pa_stream_underflow_notify_callback, (void*)this);
        }
    }

    pa_threaded_mainloop_unlock(manager->mainloop);
}


void Source::startPlayback()
{
    if(stream && isActive_){
        pa_threaded_mainloop_lock(manager->mainloop);
        operation = pa_stream_cork(stream, 0, pa_stream_success_callback, this);
        pa_threaded_mainloop_unlock(manager->mainloop);
    }
}


void Source::write(size_t nbytes, bool isDoingInitialization)
{
    const int frameSize = sizeof(float) * sampleSpec.channels;

    int numBufFrames = nbytes / frameSize;

    pa_seek_mode_t seekMode;
    if(!initialWritingDone){
        seekMode = PA_SEEK_RELATIVE;
    } else if(doAdjustTime){
        seekMode = PA_SEEK_RELATIVE_ON_READ;
        doAdjustTime = false;
    } else {
        seekMode = PA_SEEK_RELATIVE;
    }

    int numSilentFrames = 0;
    if(currentFrame < 0){
        numSilentFrames = std::min((0 - currentFrame), numBufFrames);
        silenceBuf.resize(numSilentFrames * sampleSpec.channels, 0.0f);
        pa_stream_write(stream, &silenceBuf[0], (numSilentFrames * frameSize), NULL, 0, seekMode);
        currentFrame += numSilentFrames;
        numBufFrames = pa_stream_writable_size(stream);
        seekMode = PA_SEEK_RELATIVE;
    }

    const std::vector<float>& data = audioItem->samplingData();
    const int numAllFrames = data.size() / sampleSpec.channels;
    int numFrames = std::min(numAllFrames - currentFrame, numBufFrames);

    if(numFrames > 0){
        const float* p = &data[currentFrame * sampleSpec.channels];
        pa_stream_write(stream, p, (numFrames * frameSize), NULL, 0, seekMode);
        currentFrame += numFrames;

    } else if(numSilentFrames == 0){
        if(!isDoingInitialization && !hasAllFramesWritten){
            hasAllFramesWritten = true;
            pa_usec_t latency;
            int negative;
            pa_stream_get_latency(stream, &latency, &negative);
            if(negative){
                latency = 0;
            }
            callLater(std::bind(&Source::onAllFramesWritten, this, latency));
        }
    }
}


void Source::onAllFramesWritten(pa_usec_t latency)
{
    timeToFinish = manager->timeBar->realPlaybackTime() + latency / 1000000.0;
}


void Source::onBufferOverflow()
{
    adjustTime(_("overflowed"));
}

void Source::onBufferUnderflow()
{
    adjustTime(_("underflowed"));
}


void Source::adjustTime(const char* reason)
{
    pa_threaded_mainloop_lock(manager->mainloop);
    double time = manager->timeBar->realPlaybackTime();
    seek(time);
    doAdjustTime = true;
    pa_threaded_mainloop_unlock(manager->mainloop);

    manager->os <<
        format(_("PulseAudioManager: Buffer of {0} {1}. Its playback time is adjusted to {2}."),
               audioItem->name(), reason, time) << endl;
}


void Source::stop()
{
    if(stream && isActive_){

        pa_threaded_mainloop_lock(manager->mainloop);

        waitForOperation();
        
        pa_stream_set_write_callback(stream, 0, 0);
        pa_stream_set_overflow_callback(stream, 0, 0);
        pa_stream_set_underflow_callback(stream, 0, 0);

        operation = pa_stream_cork(stream, 1, pa_stream_success_callback, this);
        waitForOperation();

        if(manager->connectionKeepCheck->isChecked()){
            operation = pa_stream_flush(stream, pa_stream_success_callback, this);
            //waitForOperation();
        } else {
            disconnectStream();
        }

        pa_threaded_mainloop_unlock(manager->mainloop);
    }
    isActive_ = false;
}
