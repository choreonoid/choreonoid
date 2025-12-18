#include "FFmpegMovieRecorderEncoder.h"
#include <cnoid/Format>
#include <cstdio>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/frame.h>
#include <libavutil/opt.h>
#include <libavutil/imgutils.h>
}

#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

void YCbCrfromRGB(double& Y, double& Cb, double& Cr, const double R, const double G, const double B) 
{
    Y = 0.257 * R + 0.504 * G + 0.098 * B + 16.0;
    Cb = -0.148 * R - 0.291 * G + 0.439 * B + 128.0;
    Cr = 0.439 * R - 0.368 * G - 0.071 * B + 128.0;
}

}

namespace cnoid {

class FFmpegMovieRecorderEncoder::Impl
{
public:
    static bool copyCapturedImageToAVFrame(
        FFmpegMovieRecorderEncoder* self, CapturedImagePtr captured, AVFrame* avFrame);
};

}


std::string FFmpegMovieRecorderEncoder::formatName() const
{
    return "MP4";
}


bool FFmpegMovieRecorderEncoder::initializeEncoding(int width, int height, int frameRate)
{
    if(width % 8 || height % 8){
        setErrorMessage(_("Please set image height and width as a mutiple of 8."));
        return false;
    }

    this->width = width;
    this->height = height;
    this->frameRate = frameRate;
    
    return true;
}

    
bool FFmpegMovieRecorderEncoder::doEncoding(std::string fileBasename)
{
    string filename = fileBasename + ".mp4";

    AVIOContext* io_context = nullptr;
    if(avio_open(&io_context, filename.c_str(), AVIO_FLAG_WRITE)){
        setErrorMessage(_("Executing avio_open failed."));
        return false;
    }

    AVFormatContext* format_context = nullptr;
    if(avformat_alloc_output_context2(&format_context, nullptr, "mp4", nullptr) < 0){
        setErrorMessage(_("Executing avformat_alloc_output_context2 failed."));
        return false;
    }

    format_context->pb = io_context;

#ifdef _WIN32
    const char* encoderName = "libopenh264";
#else
    const char* encoderName = "libx264";
#endif
    const AVCodec* codec = avcodec_find_encoder_by_name(encoderName);
    if(!codec){
        setErrorMessage(formatR(_("Encoder \"{0}\" is not found."), encoderName));
        return false;
    }

    AVCodecContext* codec_context = avcodec_alloc_context3(codec);
    if(!codec_context){
        setErrorMessage(_("Executing avcodec_alloc_context3 failed."));
        return false;
    }

    codec_context->bit_rate = 40000;
    codec_context->width = width;
    codec_context->height = height;
    codec_context->time_base.num = 1;
    codec_context->time_base.den = frameRate;
    codec_context->framerate.num = frameRate;
    codec_context->framerate.den = 1;
    codec_context->gop_size = 10;
    codec_context->max_b_frames = 1;
    codec_context->pix_fmt = AVPixelFormat::AV_PIX_FMT_YUV420P;

    if(format_context->oformat->flags & AVFMT_GLOBALHEADER){
        codec_context->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;
    }

    // make codec options
    AVDictionary* codec_options = nullptr;
    int ret = avcodec_open2(codec_context, codec_context->codec, &codec_options);
    if(ret != 0){
        char error[64];
        av_make_error_string(error , AV_ERROR_MAX_STRING_SIZE, ret);
        setErrorMessage(formatR(_("Executing avcodec_open2 failed: {0}"), error));
        return false;
    }

    AVStream* stream = avformat_new_stream(format_context, codec);
    if(!stream){
        setErrorMessage(_("Executing avformat_new_stream failed."));
        return false;
    }

    stream->sample_aspect_ratio = codec_context->sample_aspect_ratio;
    stream->time_base = codec_context->time_base;

    if(avcodec_parameters_from_context(stream->codecpar, codec_context) < 0){
        setErrorMessage(_("Executing avcodec_parameters_from_context failed."));
        return false;
    }

    if(avformat_write_header(format_context, nullptr) < 0){
        setErrorMessage(_("Executing avformat_write_header failed"));
        return false;
    }

    AVFrame* avFrame = av_frame_alloc();
    if(!avFrame){
        setErrorMessage(_("A video frame data cannot be allocated."));
        return false;
    }
    avFrame->format = codec_context->pix_fmt;
    avFrame->width = codec_context->width;
    avFrame->height = codec_context->height;

    ret = av_frame_get_buffer(avFrame, 0);
    if(ret < 0){
        setErrorMessage(_("A video frame buffer cannot be allocated."));
        return false;
    }

    int img_size = codec_context->height * codec_context->width;
    bool failed = false;

    while(true){
        fflush(stdout);

        AVFrame* frameToSend = nullptr;
        CapturedImagePtr captured = getNextFrameImage();
        if(captured){
            if(!Impl::copyCapturedImageToAVFrame(this, captured, avFrame)){
                failed = true;
                break;
            }
            frameToSend = avFrame;
        }
        if(avcodec_send_frame(codec_context, frameToSend) != 0){
            setErrorMessage(_("Executing avcodec_send_frame failed."));
            failed = true;
            break;
        }
        // An AVPacket variable must be initialized in the following form to avoid craching with FFmpeg functions.
        AVPacket packet = AVPacket();
        while(avcodec_receive_packet(codec_context, &packet) == 0){
            packet.stream_index = 0;
            av_packet_rescale_ts(&packet, codec_context->time_base, stream->time_base);
            ret = av_interleaved_write_frame(format_context, &packet);
            if(ret != 0){
                setErrorMessage(formatR(_("Executing av_interleaved_write_frame failed: {0}"), ret));
                failed = true;
                break;
            }
        }
        if(!captured || failed){
            break;
        }
    }

    if(!failed){
        if(av_write_trailer(format_context) != 0){
            setErrorMessage(_("Executing av_write_trailer failed."));
            failed = true;
        }
    }

    av_frame_free(&avFrame);
    avcodec_free_context(&codec_context);
    avformat_free_context(format_context);
    avio_closep(&io_context);

    return !failed;
}


bool FFmpegMovieRecorderEncoder::Impl::copyCapturedImageToAVFrame
(FFmpegMovieRecorderEncoder* self, CapturedImagePtr captured, AVFrame* avFrame)
{
    /* make sure the frame data is writable */
    int ret = av_frame_make_writable(avFrame);
    if(ret < 0){
        self->setErrorMessage(_("A video frame data is not writable."));
        return false;
    }

    // copy into frame
    QImage image;
    if(captured->image.index() == 0){
        image = std::get<QPixmap>(captured->image).toImage();
    } else {
        image = std::get<QImage>(captured->image);
    }
    int width = image.width();
    int height = image.height();

    for(int y = 0; y < height; ++y){
        for (int x = 0; x < width; ++x){
            QRgb rgb = image.pixel(x, y);
            double Y, Cb, Cr;
            YCbCrfromRGB(Y, Cb, Cr, qRed(rgb), qGreen(rgb), qBlue(rgb));
            avFrame->data[0][y * avFrame->linesize[0] + x] = Y;
        }
    }
    for(int y = 0; y < height / 2; ++y){
        for(int x = 0; x < width / 2; ++x){
            QRgb rgb = image.pixel(2 * x, 2 * y);
            double Y, Cb, Cr;
            YCbCrfromRGB(Y, Cb, Cr, qRed(rgb), qGreen(rgb), qBlue(rgb));
            avFrame->data[1][y * avFrame->linesize[1] + x] = Cb;
            avFrame->data[2][y * avFrame->linesize[2] + x] = Cr;
        }
    }
    
    avFrame->pts = captured->frame;

    return true;
}
