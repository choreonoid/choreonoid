#include "FFmpegMovieRecorderEncoder.h"
#include <cnoid/Plugin>

using namespace cnoid;

namespace {
  
class FFmpegPlugin : public Plugin
{
public:
    FFmpegPlugin() : Plugin("FFmpeg")
    {

    }

    virtual bool initialize()
    {
        MovieRecorder::addEncoder(new FFmpegMovieRecorderEncoder);
        return true;
    }
};

}

CNOID_IMPLEMENT_PLUGIN_ENTRY(FFmpegPlugin);
