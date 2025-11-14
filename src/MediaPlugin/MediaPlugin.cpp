/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include "MediaItem.h"
#ifdef _WIN32
  #ifdef USE_MEDIA_FOUNDATION
  #include "MFMediaView.h"
  #else
  #include "DSMediaView.h"
  #endif
#else
#include "GSMediaView.h"
#endif
#include "AudioItem.h"

#ifdef CNOID_MEDIA_PLUGIN_USE_PULSEAUDIO
#include "PulseAudioManager.h"
#endif

#include <cnoid/Plugin>
#include <cnoid/App>
#include <cnoid/MessageView>
#include <iostream>

using namespace std;

using namespace cnoid;

namespace {
    
class MediaPlugin : public Plugin
{
public:
    MediaPlugin() : Plugin("Media")
        { 

        }
        
    virtual ~MediaPlugin()
        {

        }

    virtual bool initialize()
        {
            MediaItem::initialize(this);

#ifdef _WIN32
  #ifdef USE_MEDIA_FOUNDATION
            MFMediaView::initialize(this);
  #else
            DSMediaView::initialize(this);
  #endif
#else
            GSMediaView::initializeClass(this);
#endif
            AudioItem::initialize(this);

#ifdef CNOID_MEDIA_PLUGIN_USE_PULSEAUDIO
            PulseAudioManager::initialize(this);
#endif
            return true;
        }

    virtual bool finalize()
        {
#ifdef _WIN32
  #ifdef USE_MEDIA_FOUNDATION
            MFMediaView::finalize();
  #else
            DSMediaView::finalize();
  #endif
#endif
            return true;
        }
};
}

CNOID_IMPLEMENT_PLUGIN_ENTRY(MediaPlugin);
