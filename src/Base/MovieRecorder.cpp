#include "MovieRecorder.h"
#include "ViewManager.h"
#include "ViewArea.h"
#include "App.h"
#include "AppConfig.h"
#include "MainWindow.h"
#include "MessageView.h"
#include "SceneView.h"
#include "SceneWidget.h"
#include "TimeBar.h"
#include "Archive.h"
#include "LazyCaller.h"
#include "Timer.h"
#include <cnoid/ConnectionSet>
#include <cnoid/UTF8>
#include <cnoid/stdx/filesystem>
#include <QPainter>
#include <QProgressDialog>
#include <QCoreApplication>
#include <fmt/format.h>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <deque>
#include <cstdlib>

// For the mouse cursor capture
#ifdef Q_OS_LINUX
#include <QX11Info>
#include <X11/extensions/Xfixes.h>
static constexpr bool hasMouseCursorCaptureFeature = true;
#else
static constexpr bool hasMouseCursorCaptureFeature = false;
#endif

#include "gettext.h"

using namespace std;
using namespace cnoid;
namespace filesystem = cnoid::stdx::filesystem;

namespace {

MovieRecorder* instance_ = nullptr;

const char* recordingModeSymbols[] = { "offline", "online", "direct" };

vector<MovieRecorderEncoderPtr> newEncoders;

class ViewMarker : public QWidget
{
public:
    MovieRecorder::Impl* recorderImpl;
    QPen pen;
    ViewMarker(MovieRecorder::Impl* recorderImpl);
    void setTargetView(View* view);
    virtual void paintEvent(QPaintEvent* event);
};

}

namespace cnoid {

class MovieRecorder::Impl
{
public:
    MovieRecorder* self;    
    RecordingMode recordingMode;
    int frame;
    bool isRecording;
    bool isBeforeFirstFrameCapture;
    bool requestStopRecording;
    double frameRate;
    double nextFrameTime;
    double timeStep;
    double startingTime;
    double finishingTime;
    double specifiedStartingTime;
    double specifiedFinishingTime;
    bool isStartingTimeSpecified;
    bool isFinishingTimeSpecified;
    bool isMouseCursorCaptureEnabled;
    bool isImageSizeSpecified;
    int imageWidth;
    int imageHeight;

    Signal<void(bool on)> sigRecordingStateChanged;
    Signal<void()> sigRecordingConfigurationChanged;
    Signal<void(bool isBlinked)> sigBlinking;

    string targetViewName;
    View* targetView;
    ScopedConnectionSet targetViewConnections;
    ScopedConnection viewCreationConnection;

    MessageView* mv;
    char* startingMessage;
    TimeBar* timeBar;
    ConnectionSet timeBarConnections;

    Timer directModeTimer;
    Timer blinkTimer;
    bool isBlinked;

    ViewMarker* viewMarker;
    bool isViewMarkerVisible;

    string directory;
    string fileBaseName;
    string fileBasePath;

    typedef MovieRecorderEncoder::CapturedImage CapturedImage;
    typedef MovieRecorderEncoder::CapturedImagePtr CapturedImagePtr;

    deque<CapturedImagePtr> capturedImages;
    vector<quint32> tmpImageBuf;
    std::thread encoderThread;
    std::mutex imageQueueMutex;
    std::condition_variable imageQueueCondition;

    vector<MovieRecorderEncoderPtr> encoders;
    int currentEncoderIndex;
    MovieRecorderEncoderPtr currentEncoder;
    string encodeErrorMessage;

    static MovieRecorder::Impl* instance();
    Impl(MovieRecorder* self);
    ~Impl();
    void addEncoder(MovieRecorderEncoder* encoder);
    void setTargetView(View* view, bool isExplicitlySpecified, bool doNotify);
    void setTargetView(const std::string& name, bool doNotify);
    void onViewCreated(View* view);
    void onTargetViewResized(View* view);
    void onTargetViewRemoved(View* view);
    void setCurrentEncoder(int index, bool doNotify);
    void setCurrentEncoderByFormatName(const std::string& formatName, bool doNotify);
    
    bool startRecording();
    bool initializeRecording();
    bool startOfflineModeRecording();
    void prepareForOnlineModeRecording();
    void startOnlineModeRecording();
    bool onTimeChanged(double time);
    void onPlaybackStopped(bool isStoppedManually);
    void startDirectModeRecording();
    void onDirectModeTimerTimeout();
    void captureViewImage(bool waitForPrevOutput);
    void drawMouseCursorImage(QPainter& painter);
    void captureSceneWidgets(QWidget* widget, QPixmap& pixmap);
    void startEncoding();
    MovieRecorderEncoder::CapturedImagePtr getNextFrameImage();
    void setEncodeErrorMessage(const std::string& message);
    void onEncodingFailed();
    void stopRecording(bool isFinished);
    void onViewMarkerToggled(bool on);
    void setViewMarkerVisible(bool on);
    void updateViewMarker();
    void startBlinking();
    void onBlinkingTimeout();
    void stopBlinking();
    void store(Mapping* archive);
    void restore(const Mapping* archive);
};

}


MovieRecorder* MovieRecorder::instance()
{
    if(!instance_){
        instance_ = App::baseModule()->manage(new MovieRecorder);
    }
    return instance_;
}


MovieRecorder::Impl* MovieRecorder::Impl::instance()
{
    return MovieRecorder::instance()->impl;
}


void MovieRecorder::addEncoder(MovieRecorderEncoder* encoder)
{
    if(!instance_){
        newEncoders.push_back(encoder);
    } else {
        instance_->impl->addEncoder(encoder);
    }
}


MovieRecorder::MovieRecorder()
{
    impl = new Impl(this);
}


MovieRecorder::Impl::Impl(MovieRecorder* self)
    : self(self)
{
    recordingMode = OfflineMode;
    frame = 0;
    isRecording = false;
    isBeforeFirstFrameCapture = false;
    requestStopRecording = false;
    frameRate = 30.0;
    specifiedStartingTime = 0.0;
    specifiedFinishingTime = 0.0;
    isStartingTimeSpecified = false;
    isFinishingTimeSpecified = false;
    isMouseCursorCaptureEnabled = false;
    isImageSizeSpecified = false;
    imageWidth = 640;
    imageHeight = 480;

    targetView = nullptr;

    mv = MessageView::instance();
    startingMessage = _("Recording of {0} has been started with the {1} mode.");
    timeBar = TimeBar::instance();

    directModeTimer.sigTimeout().connect([&](){ onDirectModeTimerTimeout(); });

    blinkTimer.sigTimeout().connect([&](){ onBlinkingTimeout(); });
    blinkTimer.setInterval(500);
    isBlinked = false;

    viewMarker = nullptr;
    isViewMarkerVisible = false;

#ifdef _WIN32
    if(auto userProfile = getenv("USERPROFILE")){
        directory = toUTF8(string(userProfile) + "\\Documents");
    }
#else
    if(auto home = getenv("HOME")){
        directory = string(home) + "/Videos";
    }
#endif

    fileBaseName = "movie";

    currentEncoderIndex = 0;

    for(auto& encoder : newEncoders){
        addEncoder(encoder);
    }
    newEncoders.clear();

    addEncoder(new SequentialNumberedImageFileEncoder);

    auto config = AppConfig::archive()->findMapping("MovieRecorder");
    if(config->isValid()){
        restore(config);
    }
}


MovieRecorder::~MovieRecorder()
{
    delete impl;
}


MovieRecorder::Impl::~Impl()
{
    if(encoderThread.joinable()){
        requestStopRecording = true;
        imageQueueCondition.notify_all();
        encoderThread.join();
    }
    
    timeBarConnections.disconnect();
    store(AppConfig::archive()->openMapping("MovieRecorder"));

    if(viewMarker){
        delete viewMarker;
    }
}


void MovieRecorder::Impl::addEncoder(MovieRecorderEncoder* encoder)
{
    encoder->recorderImpl = this;
    encoders.push_back(encoder);
    if(!currentEncoder){
        currentEncoder = encoder;
        currentEncoderIndex = 0;
    }
    sigRecordingConfigurationChanged();
}


View* MovieRecorder::targetView()
{
    return impl->targetView;
}


std::string MovieRecorder::targetViewName()
{
    return impl->targetViewName;
}


void MovieRecorder::setTargetView(View* view, bool isExplicitlySpecified)
{
    impl->setTargetView(view, isExplicitlySpecified, true);
}


void MovieRecorder::Impl::setTargetView(View* view, bool isExplicitlySpecified, bool doNotify)
{
    if(view != targetView){
        if(isRecording){
            stopRecording(false);
        }
        targetViewConnections.disconnect();
        targetView = view;

        if(targetView){
            auto view = targetView;
            targetViewName = targetView->name();
            targetViewConnections.add(
                targetView->sigResized().connect(
                    [this, view](){ onTargetViewResized(view); }));
            targetViewConnections.add(
                targetView->sigRemoved().connect(
                    [this, view](){ onTargetViewRemoved(view); }));
        }

        updateViewMarker();

        if(doNotify){
            sigRecordingConfigurationChanged();
        }
    }
    
    if(targetView){
        viewCreationConnection.disconnect();

    } else if(!view && isExplicitlySpecified){
        targetViewName.clear();
        viewCreationConnection.disconnect();

    } else if(!targetViewName.empty()){
        viewCreationConnection =
            ViewManager::sigViewCreated().connect(
                [&](View* view){ onViewCreated(view); });
    }
}


void MovieRecorder::Impl::setTargetView(const std::string& name, bool doNotify)
{
    if(name != targetViewName){
        bool found = false;
        for(auto& view : ViewManager::allViews()){
            if(view->name() == name){
                setTargetView(view, true, doNotify);
                found = true;
                break;
            }
        }
        if(!found){
            setTargetView(nullptr, false, doNotify);
            targetViewName = name;
        }
    }
}


void MovieRecorder::Impl::onViewCreated(View* view)
{
    if(!targetView && (view->name() == targetViewName)){
        setTargetView(view, true, true);
    }
}


void MovieRecorder::Impl::onTargetViewResized(View* view)
{
    if(view == targetView){
        updateViewMarker();
    }
}


void MovieRecorder::Impl::onTargetViewRemoved(View* view)
{
    if(view == targetView){
        setTargetView(nullptr, false, true);
    }
}


const char* MovieRecorder::recordingModeLabel(RecordingMode mode)
{
    switch(mode){
    case OfflineMode: return "Offline";
    case OnlineMode: return "Online";
    case DirectMode: return "Direct";
    }
    return nullptr;
}


const char* MovieRecorder::translatedRecordingModeLabel(RecordingMode mode)
{
    switch(mode){
    case OfflineMode: return _("Offline");
    case OnlineMode: return _("Online");
    case DirectMode: return _("Direct");
    }
    return nullptr;
}


MovieRecorder::RecordingMode MovieRecorder::recordingMode() const
{
    return impl->recordingMode;
}


void MovieRecorder::setRecordingMode(RecordingMode mode)
{
    impl->recordingMode = mode;
    impl->sigRecordingConfigurationChanged();
}


std::vector<MovieRecorderEncoderPtr> MovieRecorder::encoders()
{
    return impl->encoders;
}


int MovieRecorder::currentEncoderIndex() const
{
    return impl->currentEncoderIndex;
}


void MovieRecorder::setCurrentEncoder(int index)
{
    impl->setCurrentEncoder(index, true);
}


void MovieRecorder::Impl::setCurrentEncoder(int index, bool doNotify)
{
    if(index < 0){
        index = 0;
    }
    if(index < encoders.size()){
        currentEncoder = encoders[index];
        currentEncoderIndex = index;
        if(doNotify){
            sigRecordingConfigurationChanged();
        }
    }
}


void MovieRecorder::Impl::setCurrentEncoderByFormatName(const std::string& formatName, bool doNotify)
{
    for(size_t i=0; i < encoders.size(); ++i){
        if(encoders[i]->formatName() == formatName){
            setCurrentEncoder(i, doNotify);
            break;
        }
    }
}


std::string MovieRecorder::outputDirectory() const
{
    return impl->directory;
}


void MovieRecorder::setOutputDirectory(const std::string& directory)
{
    impl->directory = directory;
}


std::string MovieRecorder::fileBaseName() const
{
    return impl->fileBaseName;
}


void MovieRecorder::setFileBaseName(const std::string& baseName)
{
    impl->fileBaseName = baseName;
}


double MovieRecorder::frameRate() const
{
    return impl->frameRate;
}


void MovieRecorder::setFrameRate(const double frameRate)
{
    impl->frameRate = frameRate;
}


bool MovieRecorder::isStartingTimeSpecified() const
{
    return impl->isStartingTimeSpecified;
}


void MovieRecorder::setStartingTimeSpecified(bool on)
{
    impl->isStartingTimeSpecified = on;
}


double MovieRecorder::startingTime() const
{
    return impl->specifiedStartingTime;
}


void MovieRecorder::setStartingTime(double time)
{
    impl->specifiedStartingTime = time;
}


bool MovieRecorder::isFinishingTimeSpecified() const
{
    return impl->isFinishingTimeSpecified;
}


void MovieRecorder::setFinishingTimeSpecified(bool on)
{
    impl->isFinishingTimeSpecified = on;
}


double MovieRecorder::finishingTime() const
{
    return impl->specifiedFinishingTime;
}


void MovieRecorder::setFinishingTime(double time)
{
    impl->specifiedFinishingTime = time;
}


bool MovieRecorder::isImageSizeSpecified()
{
    return impl->isImageSizeSpecified;
}


void MovieRecorder::setImageSizeSpecified(bool on)
{
    impl->isImageSizeSpecified = on;
}


int MovieRecorder::imageWidth() const
{
    return impl->imageWidth;
}


int MovieRecorder::imageHeight() const
{
    return impl->imageHeight;
}


void MovieRecorder::setImageSize(int width, int height)
{
    impl->imageWidth = width;
    impl->imageHeight = height;
}


bool MovieRecorder::isMouseCursorCaptureAvailable()
{
    return hasMouseCursorCaptureFeature;
}


bool MovieRecorder::isMouseCursorCaptureEnabled() const
{
    return impl->isMouseCursorCaptureEnabled;
}


bool MovieRecorder::isCapturingMouseCursorEnabled() const
{
    return isMouseCursorCaptureEnabled();
}


void MovieRecorder::setMouseCursorCaptureEnabled(bool on)
{
    if(hasMouseCursorCaptureFeature){
        impl->isMouseCursorCaptureEnabled = on;
    }
}


void MovieRecorder::setCapturingMouseCursorEnabled(bool on)
{
    setMouseCursorCaptureEnabled(on);
}


bool MovieRecorder::isRecording() const
{
    return impl->isRecording;
}


bool MovieRecorder::startRecording()
{
    return impl->startRecording();
}


bool MovieRecorder::Impl::startRecording()
{
    if(isRecording){
        return false;
    }
    if(!initializeRecording()){
        stopRecording(false);
        return false;
    }
    switch(recordingMode){
    case OfflineMode:
        startOfflineModeRecording();
        break;
    case OnlineMode:
        prepareForOnlineModeRecording();
        break;
    case DirectMode:
        startDirectModeRecording();
        break;
    default:
        isRecording = false;
        break;
    }
    return isRecording;
}


bool MovieRecorder::Impl::initializeRecording()
{
    if(!targetView){
        showWarningDialog(_("Target view is not specified."));
        return false;
    }

    if(directory.empty()){
        showWarningDialog(_("Please set a directory to output image files."));
        return false;
    }
    filesystem::path dirPath = fromUTF8(directory);
    if(filesystem::exists(dirPath)){
        if(!filesystem::is_directory(dirPath)){
            showWarningDialog(fmt::format(_("{} is not a directory."), toUTF8(dirPath.string())));
            return false;
        }
    } else {
        filesystem::create_directories(dirPath);
    }

    filesystem::path basePath(dirPath / fromUTF8(fileBaseName));
    fileBasePath = toUTF8(basePath.string());

    int width, height;
    QSize viewSize = targetView->size();
    if(isImageSizeSpecified){
        width = imageWidth;
        height = imageHeight;
    } else {
        width = viewSize.width();
        height = viewSize.height();
    }
    {
        std::lock_guard<std::mutex> lock(imageQueueMutex);
        capturedImages.clear();
    }

    frame = 0;
    requestStopRecording = false;
    timeStep = 1.0 / frameRate;
    startingTime = isStartingTimeSpecified ? specifiedStartingTime : 0.0;
    finishingTime = isFinishingTimeSpecified ? specifiedFinishingTime : std::numeric_limits<double>::max();

    bool initialized = currentEncoder->initializeEncoding(width, height, frameRate);

    if(initialized){
        if(isImageSizeSpecified){
            int x = (viewSize.width() - width) / 2;
            int y = (viewSize.height() - height) / 2;
            targetView->setGeometry(x, y, width, height);
        }
    } else {
        if(!encodeErrorMessage.empty()){
            showWarningDialog(encodeErrorMessage);
        }
    }
    
    return initialized;
}


bool MovieRecorder::Impl::startOfflineModeRecording()
{
    double time = startingTime;
    bool doContinue = true;

    mv->putln(
        fmt::format(startingMessage,
                    targetView->windowTitle().toStdString(),
                    translatedRecordingModeLabel(recordingMode)));
    
    isRecording = true;
    sigRecordingStateChanged(true);

    startBlinking();
    startEncoding();

    while(time <= finishingTime && doContinue){

        doContinue = timeBar->setTime(time);

        QCoreApplication::processEvents();

        if(requestStopRecording){
            break;
        }

        captureViewImage(true);

        time += timeStep;
        frame++;
    }

    stopRecording(!requestStopRecording);
    bool result = !requestStopRecording;
    requestStopRecording = false;

    return result;
}


void MovieRecorder::Impl::prepareForOnlineModeRecording()
{
    timeBarConnections.disconnect();
        
    isBeforeFirstFrameCapture = true;
    nextFrameTime = startingTime;
            
    if(timeBar->isDoingPlayback()){
        startOnlineModeRecording();
    } else {
        timeBarConnections.add(
            timeBar->sigPlaybackStarted().connect(
                [&](double /* time */){ startOnlineModeRecording(); }));
        
        mv->putln(
            fmt::format(_("The online mode recording for {} is ready."),
                        targetView->windowTitle().toStdString()));
    }
}


void MovieRecorder::Impl::startOnlineModeRecording()
{
    timeBarConnections.disconnect();
    
    timeBarConnections.add(
        timeBar->sigTimeChanged().connect(
            [&](double time){ return onTimeChanged(time); }));

    timeBarConnections.add(
        timeBar->sigPlaybackStopped().connect(
            [&](double /* time */, bool isStoppedManually){ onPlaybackStopped(isStoppedManually); }));

    mv->putln(
        fmt::format(
            startingMessage,
            targetView->windowTitle().toStdString(),
            translatedRecordingModeLabel(recordingMode)));

    isRecording = true;
    sigRecordingStateChanged(true);
    startEncoding();
}


bool MovieRecorder::Impl::onTimeChanged(double time)
{
    if(isRecording){
        if(isBeforeFirstFrameCapture){
            if(time >= startingTime){
                isBeforeFirstFrameCapture = false;
                startBlinking();
            }
        }
        if(time > finishingTime){
            stopRecording(true);
        } else {
            while(time >= nextFrameTime){
                captureViewImage(false);
                ++frame;
                nextFrameTime += timeStep;
            }
        }
    }
    return false;
}


void MovieRecorder::Impl::onPlaybackStopped(bool isStoppedManually)
{
    stopRecording(!isStoppedManually);
}


void MovieRecorder::Impl::startDirectModeRecording()
{
    mv->putln(
        fmt::format(
            startingMessage,
            targetView->windowTitle().toStdString(),
            translatedRecordingModeLabel(recordingMode)));

    isRecording = true;
    sigRecordingStateChanged(true);

    startBlinking();
    startEncoding();
    directModeTimer.setInterval(1000.0 / frameRate);
    directModeTimer.start();
}


void MovieRecorder::Impl::onDirectModeTimerTimeout()
{
    captureViewImage(false);
    ++frame;
}


void MovieRecorder::Impl::captureViewImage(bool waitForPrevOutput)
{
    CapturedImagePtr captured = new CapturedImage;
    captured->frame = frame;
    
    if(SceneView* sceneView = dynamic_cast<SceneView*>(targetView)){
        captured->image = sceneView->sceneWidget()->getImage();
        if(isMouseCursorCaptureEnabled){
            QPainter painter(&stdx::get<QImage>(captured->image));
            drawMouseCursorImage(painter);
        }
    } else {
        captured->image = targetView->grab();
        QPixmap& pixmap = stdx::get<QPixmap>(captured->image);
        captureSceneWidgets(targetView, pixmap);

        if(isMouseCursorCaptureEnabled){
            QPainter painter(&pixmap);
            drawMouseCursorImage(painter);
        }
    }

    {
        std::unique_lock<std::mutex> lock(imageQueueMutex);
        if(waitForPrevOutput){
            while(!capturedImages.empty()){
                imageQueueCondition.wait(lock);
            }
        }
        capturedImages.push_back(captured);
    }
    imageQueueCondition.notify_all();
}


void MovieRecorder::Impl::drawMouseCursorImage(QPainter& painter)
{
#ifdef Q_OS_LINUX
    XFixesCursorImage* cursor = XFixesGetCursorImage(QX11Info::display());
    if(cursor){
        if(cursor->pixels){
            QImage cursorImage;
            if(sizeof(long) == 4){
                cursorImage = QImage((uchar*)cursor->pixels,
                                     cursor->width, cursor->height,
                                     QImage::Format_ARGB32);
            } else {
                tmpImageBuf.resize(cursor->width * cursor->height);
                for(size_t i=0; i < tmpImageBuf.size(); ++i){
                    tmpImageBuf[i] = (quint32)cursor->pixels[i];
                }
                cursorImage = QImage((uchar*)(&tmpImageBuf.front()),
                                     cursor->width, cursor->height,
                                     QImage::Format_ARGB32);
            }
            QPoint mousePos = QCursor::pos() - targetView->mapToGlobal(QPoint(0, 0));
            mousePos -= QPoint(cursor->xhot, cursor->yhot);
            painter.drawImage(mousePos, cursorImage);
        }
        XFree(cursor);
    }
#endif
}


void MovieRecorder::Impl::captureSceneWidgets(QWidget* widget, QPixmap& pixmap)
{
    const QObjectList objs = widget->children();
    for(int i=0; i < objs.size(); ++i){
        if(QWidget* widget = dynamic_cast<QWidget*>(objs[i])){
            if(SceneWidget* sceneWidget = dynamic_cast<SceneWidget*>(widget)){
                QPainter painter(&pixmap);
                QImage image = sceneWidget->getImage();
                QPoint pos = sceneWidget->mapTo(targetView, QPoint(0, 0));
                painter.drawImage(pos, image);
            }
            captureSceneWidgets(widget, pixmap);
        }
    }
}


void MovieRecorder::Impl::startEncoding()
{
    encodeErrorMessage.clear();
    
    if(!encoderThread.joinable()){
        MovieRecorderEncoderPtr encoder = currentEncoder;
        string path = fileBasePath;
        encoderThread = std::thread(
            [this, encoder, path](){
                if(!encoder->doEncoding(path)){
                    {
                        std::lock_guard<std::mutex> lock(imageQueueMutex);
                        capturedImages.clear();
                    }
                    imageQueueCondition.notify_all();

                    callLater([this](){ onEncodingFailed(); });
                }
            });
    }
}


MovieRecorderEncoder::CapturedImagePtr MovieRecorder::Impl::getNextFrameImage()
{
    CapturedImagePtr captured;
    {
        std::unique_lock<std::mutex> lock(imageQueueMutex);
        while(isRecording && capturedImages.empty()){
            imageQueueCondition.wait(lock);
        }
        if(capturedImages.empty() && !isRecording){
            return nullptr;
        }
        captured = capturedImages.front();
        capturedImages.pop_front();
    }
    imageQueueCondition.notify_all();

    return captured;
}


void MovieRecorder::Impl::setEncodeErrorMessage(const std::string& message)
{
    std::unique_lock<std::mutex> lock(imageQueueMutex);
    this->encodeErrorMessage = message;
}


void MovieRecorder::Impl::onEncodingFailed()
{
    if(!encodeErrorMessage.empty()){
        showWarningDialog(encodeErrorMessage);
    }
    stopRecording(false);
}


void MovieRecorder::stopRecording()
{
    impl->stopRecording(false);
}


void MovieRecorder::Impl::stopRecording(bool isFinished)
{
    directModeTimer.stop();
    
    if(isRecording){

        stopBlinking();
        
        int numRemainingImages = 0;
        {
            std::lock_guard<std::mutex> lock(imageQueueMutex);
            numRemainingImages = capturedImages.size();
        }
        if(numRemainingImages > 1){
            QProgressDialog progress(
                _("Outputting sequential image files..."), _("Abort Output"),
                0, numRemainingImages, MainWindow::instance());
            progress.setWindowTitle(_("Movie Recorder's Output Status"));
            progress.setWindowModality(Qt::WindowModal);
            while(true){
                int index;
                {
                    std::lock_guard<std::mutex> lock(imageQueueMutex);
                    index = numRemainingImages - capturedImages.size();
                }
                progress.setValue(index);

                if(progress.wasCanceled()){
                    std::lock_guard<std::mutex> lock(imageQueueMutex);
                    capturedImages.clear();
                    break;
                }
                if(index < numRemainingImages){
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                } else {
                    break;
                }
            }
        }

        isRecording = false;
        requestStopRecording = true;
        imageQueueCondition.notify_all();
        encoderThread.join();

        auto viewName = targetView->windowTitle().toStdString();
        if(isFinished){
            mv->putln(fmt::format(_("Recording of {} has been finished."), viewName));
        } else {
            mv->putln(fmt::format(_("Recording of {} has been stopped."), viewName));
        }

        targetView->updateGeometry();

        sigRecordingStateChanged(false);
    }
    
    timeBarConnections.disconnect();
}


bool MovieRecorder::isViewMarkerVisible()
{
    return impl->isViewMarkerVisible;
}


void MovieRecorder::setViewMarkerVisible(bool on)
{
    impl->setViewMarkerVisible(on);
}


void MovieRecorder::Impl::setViewMarkerVisible(bool on)
{
    if(on != isViewMarkerVisible){
        isViewMarkerVisible = on;
        updateViewMarker();
        sigRecordingConfigurationChanged();
    }
}


void MovieRecorder::Impl::updateViewMarker()
{
    if(isViewMarkerVisible && targetView && targetView->isActive()){
        if(!viewMarker){
            viewMarker = new ViewMarker(this);
        }
        viewMarker->setTargetView(targetView);
        viewMarker->show();
    } else {
        if(viewMarker){
            viewMarker->hide();
            if(!targetView){
                viewMarker->setParent(nullptr);
            }
        }
    }
}


void MovieRecorder::Impl::startBlinking()
{
    updateViewMarker();
    isBlinked = false;
    blinkTimer.start();
    sigBlinking(isBlinked);
}


void MovieRecorder::Impl::onBlinkingTimeout()
{
    if(isViewMarkerVisible && viewMarker){
        if(viewMarker->isVisible()){
            viewMarker->hide();
        } else {
            viewMarker->show();
        }
    }
    isBlinked = !isBlinked;
    sigBlinking(isBlinked);
}


void MovieRecorder::Impl::stopBlinking()
{
    if(isViewMarkerVisible && viewMarker){
        viewMarker->show();
    }
    blinkTimer.stop();
    sigBlinking(false);
}


SignalProxy<void(bool on)> MovieRecorder::sigRecordingStateChanged()
{
    return impl->sigRecordingStateChanged;
}


SignalProxy<void()> MovieRecorder::sigRecordingConfigurationChanged()
{
    return impl->sigRecordingConfigurationChanged;
}


SignalProxy<void(bool isBlinked)> MovieRecorder::sigBlinking()
{
    return impl->sigBlinking;
}


void MovieRecorder::Impl::store(Mapping* archive)
{
    if(targetViewName.empty()){
        archive->remove("target");
    } else {
        archive->write("target", targetViewName);
    }
    archive->write("recordingMode", recordingModeSymbols[recordingMode]);
    archive->write("format", currentEncoder->formatName(), DOUBLE_QUOTED);
    archive->write("showViewMarker", isViewMarkerVisible);
    archive->write("directory", directory, DOUBLE_QUOTED);
    archive->write("basename", fileBaseName, DOUBLE_QUOTED);
    archive->write("checkStartTime", isStartingTimeSpecified);
    archive->write("startTime", specifiedStartingTime);
    archive->write("checkFinishTime", isFinishingTimeSpecified);
    archive->write("finishTime", specifiedFinishingTime);
    archive->write("fps", frameRate);
    archive->write("setSize", isImageSizeSpecified);
    archive->write("width", imageWidth);
    archive->write("height", imageHeight);
    if(hasMouseCursorCaptureFeature){
        archive->write("mouseCursor", isMouseCursorCaptureEnabled);
    }
}


void MovieRecorder::Impl::restore(const Mapping* archive)
{
    string symbol;
    if(archive->read("target", symbol)){
        setTargetView(symbol, false);
    }
    if(archive->read("recordingMode", symbol)){
        for(int i=0; i < 3; ++i){
            if(symbol == recordingModeSymbols[i]){
                recordingMode = static_cast<RecordingMode>(i);
                break;
            }
        }
    }
    string format;
    if(archive->read("format", format)){
        setCurrentEncoderByFormatName(format, false);
    }
    archive->read("showViewMarker", isViewMarkerVisible);
    archive->read("directory", directory);
    archive->read("basename", fileBaseName);
    archive->read("checkStartTime", isStartingTimeSpecified);
    archive->read("startTime", specifiedStartingTime);
    archive->read("checkFinishTime", isFinishingTimeSpecified);
    archive->read("finishTime", specifiedFinishingTime);
    archive->read("fps", frameRate);
    archive->read("setSize", isImageSizeSpecified);
    archive->read("width", imageWidth);
    archive->read("height", imageHeight);
    if(hasMouseCursorCaptureFeature){
        archive->read("mouseCursor", isMouseCursorCaptureEnabled);
    }

    updateViewMarker();
    sigRecordingConfigurationChanged();
}


ViewMarker::ViewMarker(MovieRecorder::Impl* recorderImpl)
    : recorderImpl(recorderImpl)
{
    setWindowFlags(Qt::Widget | Qt::FramelessWindowHint);
    setAttribute(Qt::WA_NoSystemBackground);
    setAttribute(Qt::WA_TransparentForMouseEvents);

    pen.setStyle(Qt::SolidLine);
    pen.setColor(QColor(Qt::red));
    pen.setWidthF(8.0);
}


void ViewMarker::setTargetView(View* view)
{
    ViewArea* viewArea = view->viewArea();
    setParent(viewArea);

    QPoint p = view->viewAreaPos();
    setGeometry(p.x(), p.y(), view->width(), view->height());

    QRegion rect(view->rect());
    setMask(rect.xored(QRegion(4, 4, view->width() - 8, view->height() - 8)));
}


void ViewMarker::paintEvent(QPaintEvent*)
{
    QPainter painter(this);
    painter.setPen(pen);
    painter.setBrush(Qt::NoBrush);
    painter.drawRect(0, 0, width(), height());
}


MovieRecorderEncoder::MovieRecorderEncoder()
{
    recorderImpl = nullptr;
}


bool MovieRecorderEncoder::initializeEncoding
(int /* width */, int /* height */, int /* frameRate */)
{
    return true;
}


MovieRecorderEncoder::CapturedImagePtr MovieRecorderEncoder::getNextFrameImage()
{
    return recorderImpl->getNextFrameImage();
}


void MovieRecorderEncoder::setErrorMessage(const std::string& message)
{
    recorderImpl->setEncodeErrorMessage(message);
}


std::string SequentialNumberedImageFileEncoder::formatName() const
{
    return _("Sequential numbered PNG image files");
}


bool SequentialNumberedImageFileEncoder::doEncoding(std::string fileBaseName)
{
    bool failed = false;
    
    string fFilename(fileBaseName + "{:08d}.png");

    while(true){
        CapturedImagePtr captured = getNextFrameImage();
        if(!captured){
            break;
        }
        string filename = fmt::format(fFilename, captured->frame);
        bool saved = false;
        if(stdx::get_variant_index(captured->image) == 0){
            QPixmap& pixmap = stdx::get<QPixmap>(captured->image);
            saved = pixmap.save(filename.c_str());
        } else {
            QImage& image = stdx::get<QImage>(captured->image);
            saved = image.save(filename.c_str());
        }
        if(!saved){
            setErrorMessage(fmt::format(_("Saving an image to \"{}\" failed."), filename));
            failed = false;
            break;
        }
    }

    return !failed;
}
