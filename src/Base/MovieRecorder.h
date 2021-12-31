#ifndef CNOID_BASE_MOVIE_RECORDER_H
#define CNOID_BASE_MOVIE_RECORDER_H

#include <cnoid/Referenced>
#include <cnoid/Signal>
#include <cnoid/stdx/variant>
#include <QPixmap>
#include <QImage>
#include "exportdecl.h"

namespace cnoid {

class View;
class MovieRecorderEncoder;
typedef ref_ptr<MovieRecorderEncoder> MovieRecorderEncoderPtr;

class CNOID_EXPORT MovieRecorder
{
public:
    static MovieRecorder* instance();
    static void addEncoder(MovieRecorderEncoder* encoder);

    ~MovieRecorder();

    View* targetView();
    std::string targetViewName();
    void setTargetView(View* view, bool isExplicitlySpecified);

    enum RecordingMode { OfflineMode, OnlineMode, DirectMode, NumRecordingModes };
    static const char* recordingModeLabel(RecordingMode mode);
    static const char* translatedRecordingModeLabel(RecordingMode mode);
    RecordingMode recordingMode() const;
    void setRecordingMode(RecordingMode mode);
    
    std::vector<MovieRecorderEncoderPtr> encoders();
    int currentEncoderIndex() const;
    void setCurrentEncoder(int index);
    
    std::string outputDirectory() const;
    void setOutputDirectory(const std::string& directory);
    std::string fileBaseName() const;
    void setFileBaseName(const std::string& baseName);
    double frameRate() const;
    void setFrameRate(const double frameRate);
    bool isStartingTimeSpecified() const;
    void setStartingTimeSpecified(bool on);
    double startingTime() const;
    void setStartingTime(double time);
    bool isFinishingTimeSpecified() const;
    void setFinishingTimeSpecified(bool on);
    double finishingTime() const;
    void setFinishingTime(double time);
    bool isImageSizeSpecified();
    void setImageSizeSpecified(bool on);
    int imageWidth() const;
    int imageHeight() const;
    void setImageSize(int width, int height);
    bool isCapturingMouseCursorEnabled() const;
    void setCapturingMouseCursorEnabled(bool on);
    
    bool isRecording() const;
    bool startRecording();
    void stopRecording();
    bool isViewMarkerVisible();
    void setViewMarkerVisible(bool on);

    SignalProxy<void(bool on)> sigRecordingStateChanged();
    SignalProxy<void()> sigRecordingConfigurationChanged();
    SignalProxy<void(bool isBlinked)> sigBlinking();

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

class CNOID_EXPORT SequentialNumberedImageFileEncoder : public MovieRecorderEncoder
{
public:
    virtual std::string formatName() const override;
    virtual bool doEncoding(std::string fileBasename) override;
};

}

#endif
