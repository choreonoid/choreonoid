/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_MOVIE_RECORDER_H
#define CNOID_BASE_MOVIE_RECORDER_H

#include <cnoid/Referenced>
#include <cnoid/stdx/variant>
#include <QPixmap>
#include <QImage>
#include "exportdecl.h"

namespace cnoid {

class ExtensionManager;
class MovieRecorderEncoder;

class CNOID_EXPORT MovieRecorder
{
public:
    static void initializeClass(ExtensionManager* ext);
    static MovieRecorder* instance();
    static void addEncoder(MovieRecorderEncoder* encoder);

    ~MovieRecorder();
    void showDialog();

    class Impl;

private:
    MovieRecorder();
    
    Impl* impl;
};

class CNOID_EXPORT MovieRecorderEncoder : public Referenced
{
public:
    virtual std::string formatName() const = 0;
    virtual bool initializeEncoding(int width, int height, int frameRate);
    virtual bool doEncoding(std::string fileBasename) = 0;

protected:
    MovieRecorderEncoder();

    class CapturedImage : public Referenced
    {
    public:
        stdx::variant<QPixmap, QImage> image;
        int frame;
    };
    typedef ref_ptr<CapturedImage> CapturedImagePtr;
    
    CapturedImagePtr getNextFrameImage();
    void setErrorMessage(const std::string& message);

private:
    MovieRecorder::Impl* recorderImpl;

    friend class MovieRecorder;
};

typedef ref_ptr<MovieRecorderEncoder> MovieRecorderEncoderPtr;

}

#endif
