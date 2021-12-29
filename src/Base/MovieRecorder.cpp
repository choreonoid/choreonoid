/**
   @author Shin'ichiro Nakaoka
*/

#include "MovieRecorder.h"
#include "ExtensionManager.h"
#include "ViewManager.h"
#include "ViewArea.h"
#include "MessageView.h"
#include "AppConfig.h"
#include "MainWindow.h"
#include "SceneView.h"
#include "SceneWidget.h"
#include "ToolBar.h"
#include "TimeBar.h"
#include "Archive.h"
#include "SpinBox.h"
#include "Buttons.h"
#include "ButtonGroup.h"
#include "LineEdit.h"
#include "CheckBox.h"
#include "ComboBox.h"
#include "Dialog.h"
#include "FileDialog.h"
#include "Separator.h"
#include "Timer.h"
#include "LazyCaller.h"
#include <cnoid/ConnectionSet>
#include <cnoid/Selection>
#include <cnoid/UTF8>
#include <cnoid/stdx/filesystem>
#include <QPainter>
#include <QDialogButtonBox>
#include <QFileDialog>
#include <QProgressDialog>
#include <QCoreApplication>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <deque>
#include <fmt/format.h>
#include <fmt/ostream.h>

#ifdef Q_OS_LINUX
#include <QX11Info>
#include <X11/extensions/Xfixes.h>
const bool ENABLE_MOUSE_CURSOR_CAPTURE = true;
#else
const bool ENABLE_MOUSE_CURSOR_CAPTURE = false;
#endif

#include "gettext.h"

using namespace std;
using namespace cnoid;
namespace filesystem = cnoid::stdx::filesystem;

namespace {

ExtensionManager* extensionManager = nullptr;
MovieRecorder* instance_ = nullptr;

vector<MovieRecorderEncoderPtr> newEncoders;

enum RecordinMode { OFFLINE_MODE, ONLINE_MODE, DIRECT_MODE, N_RECORDING_MODES };

class MovieRecorderBar : public ToolBar
{
public:
    ToolButton* recordingToggle;
    ToolButton* viewMarkerToggle;
    bool isFlashed;
    
    MovieRecorderBar();
    void flashRecordingToggle(bool setNormal);
    void setRecordingToggle(bool on);
    void changeViewMarkerToggleState(bool on);
};

MovieRecorderBar* movieRecorderBar = nullptr;

class ConfigDialog : public Dialog
{
public:
    MovieRecorder::Impl* recorderImpl;
    ScopedConnectionSet viewManagerConnections;
    LazyCaller updateViewComboLater;
    vector<View*> activeViews;
    ComboBox targetViewCombo;
    CheckBox viewMarkerCheck;
    RadioButton modeRadioButtons[N_RECORDING_MODES];
    ComboBox encoderCombo;
    LineEdit directoryEntry;
    PushButton directoryButton;
    LineEdit basenameEntry;
    CheckBox startTimeCheck;
    DoubleSpinBox startTimeSpin;
    CheckBox finishTimeCheck;
    DoubleSpinBox finishTimeSpin;
    DoubleSpinBox fpsSpin;
    CheckBox imageSizeCheck;
    SpinBox imageWidthSpin;
    SpinBox imageHeightSpin;
    CheckBox mouseCursorCheck;
    ToggleButton recordingToggle;

    ConfigDialog(MovieRecorder::Impl* recorderImpl);
    virtual void showEvent(QShowEvent* event) override;
    virtual void hideEvent(QHideEvent* event) override;
    void updateViewCombo();
    void onTargetViewIndexChanged(int index);
    void onRecordingModeRadioClicked(int mode);
    void showDirectorySelectionDialog();
    double startTime() const;
    double finishTime() const;
    double frameRate() const;
    double timeStep() const;
    void setViewMarkerCheck(bool on);
    void checkRecordingModeRadio(int mode);
    void setRecordingToggle(bool on);
    bool store(Mapping* archive);
    void restore(const Mapping* archive);
};

class ViewMarker : public QWidget
{
public:
    MovieRecorder::Impl* recorderImpl;
    QPen pen;
    ViewMarker(MovieRecorder::Impl* recorderImpl);
    void setTargetView(View* view);
    virtual void paintEvent(QPaintEvent* event);
};

class SequentialNumberedImageFileEncoder : public MovieRecorderEncoder
{
public:
    virtual std::string formatName() const override;
    virtual bool doEncoding(std::string fileBasename) override;
};

}

namespace cnoid {

class MovieRecorder::Impl
{
public:
    MovieRecorder* self;    
    Selection recordingMode;
    bool isRecording;
    bool isBeforeFirstFrameCapture;
    bool requestStopRecording;
    int frame;
    double nextFrameTime;
    double timeStep;
    double startTime;
    double finishTime;

    string targetViewName;
    View* targetView;
    ScopedConnectionSet targetViewConnections;

    MessageView* mv;
    char* startMessage;

    ConfigDialog* dialog;
    MovieRecorderBar* toolBar;
    TimeBar* timeBar;
    ConnectionSet timeBarConnections;

    Timer directModeTimer;

    ViewMarker* viewMarker;
    bool isViewMarkerEnabled;
    
    Timer flashTimer;

    filesystem::path directoryPath;
    string fileBasename;

    typedef MovieRecorderEncoder::CapturedImage CapturedImage;
    typedef MovieRecorderEncoder::CapturedImagePtr CapturedImagePtr;

    deque<CapturedImagePtr> capturedImages;
    vector<quint32> tmpImageBuf;
    std::thread encoderThread;
    std::mutex imageQueueMutex;
    std::condition_variable imageQueueCondition;

    vector<MovieRecorderEncoderPtr> encoders;
    MovieRecorderEncoderPtr currentEncoder;
    string encodeErrorMessage;

    static MovieRecorder::Impl* instance();
    Impl(MovieRecorder* self);
    ~Impl();
    void addEncoder(MovieRecorderEncoder* encoder);
    void setTargetView(View* view);
    void onTargetViewResized(View* view);
    void onTargetViewRemoved(View* view);
    void setTargetView(const std::string& name);
    void onViewCreated(View* view);
    void setRecordingMode(const std::string& symbol);
    void setCurrentEncoder(int index, bool doUpdateCombo);
    void setCurrentFormat(const std::string& formatName);
    void activateRecording(bool on, bool isActivatedByDialog);
    bool initializeRecording();
    bool doOfflineModeRecording();
    void setupOnlineModeRecording();
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
    bool showViewMarker();
    void startFlash();
    void onFlashTimeout();
    void stopFlash();
    bool store(Mapping* archive);
    void restore(const Mapping* archive);
};

}


void MovieRecorder::initializeClass(ExtensionManager* ext)
{
    extensionManager = ext;
    movieRecorderBar = new MovieRecorderBar;
    ext->addToolBar(movieRecorderBar);
}


MovieRecorder* MovieRecorder::instance()
{
    if(!instance_){
        instance_ = extensionManager->manage(new MovieRecorder);
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
    : self(self),
      recordingMode(N_RECORDING_MODES, CNOID_GETTEXT_DOMAIN_NAME),
      mv(MessageView::instance())
{
    recordingMode.setSymbol(OFFLINE_MODE, N_("Offline"));
    recordingMode.setSymbol(ONLINE_MODE, N_("Online"));
    recordingMode.setSymbol(DIRECT_MODE, N_("Direct"));
    recordingMode.select(OFFLINE_MODE);

    dialog = new ConfigDialog(this);
    toolBar = movieRecorderBar;
    timeBar = TimeBar::instance();

    targetView = nullptr;
    
    isRecording = false;
    isBeforeFirstFrameCapture = false;
    requestStopRecording = false;

    directModeTimer.sigTimeout().connect([&](){ onDirectModeTimerTimeout(); });

    startMessage = _("Recording of {0} has been started with the {1} mode.");

    viewMarker = nullptr;
    isViewMarkerEnabled = false;
    flashTimer.setInterval(500);
    flashTimer.sigTimeout().connect([&](){ onFlashTimeout(); });

    auto config = AppConfig::archive()->findMapping("MovieRecorder");
    if(config->isValid()){
        restore(config);
    }

    for(auto& encoder : newEncoders){
        addEncoder(encoder);
    }
    newEncoders.clear();

    addEncoder(new SequentialNumberedImageFileEncoder);
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
    delete dialog;

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
    }
    dialog->encoderCombo.addItem(encoder->formatName().c_str());
}


void MovieRecorder::showDialog()
{
    impl->dialog->show();
}


void MovieRecorder::Impl::setTargetView(View* view)
{
    if(view != targetView){
        if(isRecording){
            stopRecording(false);
        }
        targetViewConnections.disconnect();
        targetViewName.clear();
        targetView = view;

        dialog->updateViewCombo();
    
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

        showViewMarker();
    }
}


void MovieRecorder::Impl::onTargetViewResized(View* view)
{
    if(targetView == view){
        showViewMarker();
    }
}


void MovieRecorder::Impl::onTargetViewRemoved(View* view)
{
    if(targetView == view){
        setTargetView(0);
    }
}


void MovieRecorder::Impl::setTargetView(const std::string& name)
{
    if(name != targetViewName){
        targetViewName = name;
        if(!isRecording){
            vector<View*> views = ViewManager::allViews();
            for(size_t i=0; i <views.size(); ++i){
                View* view = views[i];
                if(view->name() == name){
                    setTargetView(view);
                    break;
                }
            }
            if(!targetView){
                targetViewConnections.add(
                    ViewManager::sigViewCreated().connect(
                        [&](View* view){ onViewCreated(view); }));
            }
        }
    }
}


void MovieRecorder::Impl::onViewCreated(View* view)
{
    if(view->name() == targetViewName){
        setTargetView(view);
    }
}


void MovieRecorder::Impl::setRecordingMode(const std::string& symbol)
{
    recordingMode.select(symbol);
    dialog->checkRecordingModeRadio(recordingMode.which());
}


void MovieRecorder::Impl::setCurrentEncoder(int index, bool doUpdateCombo)
{
    if(index < encoders.size()){
        currentEncoder = encoders[index];

        if(doUpdateCombo){
            auto& combo = dialog->encoderCombo;
            combo.blockSignals(true);
            combo.setCurrentIndex(index);
            combo.blockSignals(false);
        }
    }
}


void MovieRecorder::Impl::setCurrentFormat(const std::string& formatName)
{
    for(size_t i=0; i < encoders.size(); ++i){
        if(encoders[i]->formatName() == formatName){
            setCurrentEncoder(i, true);
            break;
        }
    }
}


void MovieRecorder::Impl::activateRecording(bool on, bool isActivatedByDialog)
{
    if(isActivatedByDialog){
        toolBar->setRecordingToggle(on);
    } else {
        dialog->setRecordingToggle(on);
    }

    if(!on){
        stopRecording(false);
        
    } else {
        if(isRecording){
            return;
        }
        if(!initializeRecording()){
            stopRecording(false);

        } else {
            switch(recordingMode.which()){
                
            case OFFLINE_MODE:
                if(doOfflineModeRecording()){
                    if(isActivatedByDialog){
                        dialog->hide();
                    }
                }
                break;
                
            case ONLINE_MODE:
                setupOnlineModeRecording();
                break;
                
            case DIRECT_MODE:
                startDirectModeRecording();
                break;
                
            default:
                isRecording = false;
                break;
            }
        }
    }
}


bool MovieRecorder::Impl::initializeRecording()
{
    if(!targetView){
        showWarningDialog(_("Target view is not specified."));
        return false;
    }

    directoryPath = fromUTF8(dialog->directoryEntry.string());
    if(directoryPath.empty()){
        showWarningDialog(_("Please set a directory to output image files."));
        return false;

    } else {
        if(filesystem::exists(directoryPath)){
            if(!filesystem::is_directory(directoryPath)){
                showWarningDialog(fmt::format(_("{} is not a directory."), directoryPath));
                return false;
            }
        } else {
            filesystem::create_directories(directoryPath);
        }
    }

    filesystem::path basePath(directoryPath / fromUTF8(dialog->basenameEntry.string()));
    fileBasename = toUTF8(basePath.string());

    int width, height;
    QSize viewSize = targetView->size();
    if(dialog->imageSizeCheck.isChecked()){
        width = dialog->imageWidthSpin.value();
        height = dialog->imageHeightSpin.value();
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
    timeStep = dialog->timeStep();
    startTime = dialog->startTime();
    finishTime = dialog->finishTime();

    bool initialized = currentEncoder->initializeEncoding(width, height, dialog->frameRate());

    if(initialized){
        if(dialog->imageSizeCheck.isChecked()){
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


bool MovieRecorder::Impl::doOfflineModeRecording()
{
    double time = startTime;
    bool doContinue = true;

    isRecording = true;

    startFlash();
    startEncoding();

    mv->putln(
        fmt::format(startMessage,
                    targetView->windowTitle().toStdString(),
                    recordingMode.selectedLabel()));
    
    while(time <= finishTime && doContinue){

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


void MovieRecorder::Impl::setupOnlineModeRecording()
{
    timeBarConnections.disconnect();
        
    isBeforeFirstFrameCapture = true;
    nextFrameTime = startTime;
            
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

    isRecording = true;
    startEncoding();

    mv->putln(
        fmt::format(
            startMessage,
            targetView->windowTitle().toStdString(),
            recordingMode.selectedLabel()));
}


bool MovieRecorder::Impl::onTimeChanged(double time)
{
    if(isRecording){
        if(isBeforeFirstFrameCapture){
            if(time >= startTime){
                isBeforeFirstFrameCapture = false;
                startFlash();
            }
        }
        if(time > finishTime){
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
    isRecording = true;
    startFlash();
    startEncoding();
    directModeTimer.setInterval(1000 / dialog->frameRate());
    directModeTimer.start();

    mv->putln(
        fmt::format(
            startMessage,
            targetView->windowTitle().toStdString(),
            recordingMode.selectedLabel()));
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
        if(dialog->mouseCursorCheck.isChecked()){
            QPainter painter(&stdx::get<QImage>(captured->image));
            drawMouseCursorImage(painter);
        }
    } else {
        captured->image = targetView->grab();
        QPixmap& pixmap = stdx::get<QPixmap>(captured->image);
        captureSceneWidgets(targetView, pixmap);

        if(dialog->mouseCursorCheck.isChecked()){
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
        string basename = fileBasename;
        encoderThread = std::thread(
            [this, encoder, basename](){
                if(!encoder->doEncoding(basename)){
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


void MovieRecorder::Impl::stopRecording(bool isFinished)
{
    directModeTimer.stop();
    
    if(isRecording){

        stopFlash();
        
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
    }
    
    timeBarConnections.disconnect();

    toolBar->setRecordingToggle(false);
    dialog->setRecordingToggle(false);

    
}


void MovieRecorder::Impl::onViewMarkerToggled(bool on)
{
    toolBar->changeViewMarkerToggleState(on);
    dialog->setViewMarkerCheck(on);
    
    isViewMarkerEnabled = on;

    if(on){
        if(!targetView){
            if(!targetViewName.empty()){
                setTargetView(targetViewName);
            }
        }
        showViewMarker();
    } else {
        if(viewMarker){
            viewMarker->hide();
        }
    }
}


bool MovieRecorder::Impl::showViewMarker()
{
    if(isViewMarkerEnabled && targetView && targetView->isActive()){
        if(!viewMarker){
            viewMarker = new ViewMarker(this);
        }
        viewMarker->setTargetView(targetView);
        viewMarker->show();
        return true;
    }
    return false;
}


void MovieRecorder::Impl::startFlash()
{
    showViewMarker();
    flashTimer.start();
}


void MovieRecorder::Impl::onFlashTimeout()
{
    toolBar->flashRecordingToggle(false);
    
    if(isViewMarkerEnabled && viewMarker){
        if(viewMarker->isVisible()){
            viewMarker->hide();
        } else {
            viewMarker->show();
        }
    }
}


void MovieRecorder::Impl::stopFlash()
{
    flashTimer.stop();
    toolBar->flashRecordingToggle(true);
    if(isViewMarkerEnabled && viewMarker){
        viewMarker->show();
    }
}


bool MovieRecorder::Impl::store(Mapping* archive)
{
    if(targetView){
        archive->write("target", targetView->name());
    }
    archive->write("recordingMode", recordingMode.selectedSymbol());
    return dialog->store(archive);
}


void MovieRecorder::Impl::restore(const Mapping* archive)
{
    string symbol;
    if(archive->read("target", symbol)){
        setTargetView(symbol);
    }
    if(archive->read("recordingMode", symbol)){
        setRecordingMode(symbol);
    }
    dialog->restore(archive);
}


MovieRecorderBar::MovieRecorderBar()
    : ToolBar(N_("MovieRecorderBar"))
{
    recordingToggle = addToggleButton(QIcon(":/Base/icon/record.svg"));
    recordingToggle->setToolTip(_("Toggle Recording"));
    recordingToggle->sigToggled().connect(
        [this](bool on){ MovieRecorder::Impl::instance()->activateRecording(on, false); });

    viewMarkerToggle = addToggleButton(QIcon(":/Base/icon/recordtarget.svg"));
    viewMarkerToggle->setToolTip(_("Toggle Target View Marker"));
    viewMarkerToggle->sigToggled().connect(
        [this](bool on){ MovieRecorder::Impl::instance()->onViewMarkerToggled(on); });
    
    auto configButton = addButton(QIcon(":/Base/icon/setup.svg"));
    configButton->setToolTip(_("Show the config dialog"));
    configButton->sigClicked().connect(
        [this](){ MovieRecorder::Impl::instance()->dialog->show(); });
}


void MovieRecorderBar::flashRecordingToggle(bool setNormal)
{
    if(isFlashed || setNormal){
        recordingToggle->setIcon(QIcon(":/Base/icon/record.svg"));
        isFlashed = false;
    } else {
        recordingToggle->setIcon(QIcon(":/Base/icon/record2.svg"));
        isFlashed = true;
    }
}


void MovieRecorderBar::setRecordingToggle(bool on)
{
    recordingToggle->blockSignals(true);
    recordingToggle->setChecked(on);
    recordingToggle->blockSignals(false);
}


void MovieRecorderBar::changeViewMarkerToggleState(bool on)
{
    viewMarkerToggle->blockSignals(true);
    viewMarkerToggle->setChecked(on);
    viewMarkerToggle->blockSignals(false);
}



ConfigDialog::ConfigDialog(MovieRecorder::Impl* recorderImpl_)
    : recorderImpl(recorderImpl_),
      updateViewComboLater([&](){ updateViewCombo(); })
{
    setWindowTitle(_("Movie Recorder"));
    
    QVBoxLayout* vbox = new QVBoxLayout;
    setLayout(vbox);

    QHBoxLayout* hbox = new QHBoxLayout;
    hbox->addWidget(new QLabel(_("Target view")));

    targetViewCombo.sigCurrentIndexChanged().connect(
        [this](int index){ onTargetViewIndexChanged(index); });
    hbox->addWidget(&targetViewCombo);

    viewMarkerCheck.setText(_("Show the marker"));
    viewMarkerCheck.sigToggled().connect(
        [this](bool on){ recorderImpl->onViewMarkerToggled(on); });
    hbox->addWidget(&viewMarkerCheck);
    
    hbox->addStretch();
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout;
    hbox->addWidget(new QLabel(_("Recording mode")));
    ButtonGroup* modeGroup = new ButtonGroup;
    for(int i=0; i < N_RECORDING_MODES; ++i){
        RadioButton* radio = &modeRadioButtons[i];
        radio->setText(recorderImpl->recordingMode.label(i));
        modeGroup->addButton(radio, i);
        hbox->addWidget(radio);
    }
    modeRadioButtons[0].setChecked(true);
    modeGroup->sigButtonClicked().connect(
        [&](int mode){ onRecordingModeRadioClicked(mode); });
    hbox->addStretch();
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout;
    hbox->addWidget(new QLabel(_("Format")));
    for(auto& encoder : recorderImpl->encoders){
        encoderCombo.addItem(encoder->formatName().c_str());
    }
    encoderCombo.sigCurrentIndexChanged().connect(
        [this](int index){ recorderImpl->setCurrentEncoder(index, false); });
    hbox->addWidget(&encoderCombo);
    hbox->addStretch();
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout;
    hbox->addWidget(new QLabel(_("Directory")));
    hbox->addWidget(&directoryEntry);

    QIcon folderIcon = QIcon::fromTheme("folder");
    if(folderIcon.isNull()){
        directoryButton.setText(_("Select"));
    } else {
        directoryButton.setIcon(folderIcon);
    }
    directoryButton.sigClicked().connect(
        [this](){ showDirectorySelectionDialog(); });
    hbox->addWidget(&directoryButton);
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout;
    hbox->addWidget(new QLabel(_("Basename")));
    basenameEntry.setText("scene");
    hbox->addWidget(&basenameEntry);
    hbox->addStretch();
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout;
    hbox->addWidget(new QLabel(_("Frame rate")));
    fpsSpin.setDecimals(1);
    fpsSpin.setRange(1.0, 9999.9);
    fpsSpin.setValue(30.0);
    fpsSpin.setSingleStep(0.1);
    hbox->addWidget(&fpsSpin);
    hbox->addWidget(new QLabel(_("[fps]")));
    hbox->addStretch();
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout;
    startTimeCheck.setText(_("Start time"));
    hbox->addWidget(&startTimeCheck);
    startTimeSpin.setDecimals(2);
    startTimeSpin.setRange(0.00, 9999.99);
    startTimeSpin.setSingleStep(0.1);
    hbox->addWidget(&startTimeSpin);
    hbox->addWidget(new QLabel(_("[s]")));
    hbox->addSpacing(4);

    finishTimeCheck.setText(_("Finish time"));
    hbox->addWidget(&finishTimeCheck);
    finishTimeSpin.setDecimals(2);
    finishTimeSpin.setRange(0.00, 9999.99);
    finishTimeSpin.setSingleStep(0.1);
    hbox->addWidget(&finishTimeSpin);
    hbox->addWidget(new QLabel(_("[s]")));
    hbox->addStretch();
    vbox->addLayout(hbox);
    
    hbox = new QHBoxLayout;
    imageSizeCheck.setText(_("Image size"));
    hbox->addWidget(&imageSizeCheck);
    
    imageWidthSpin.setRange(1, 9999);
    imageWidthSpin.setValue(640);
    hbox->addWidget(&imageWidthSpin);

    hbox->addWidget(new QLabel("x"));
    imageHeightSpin.setRange(1, 9999);
    imageHeightSpin.setValue(480);
    hbox->addWidget(&imageHeightSpin);
    hbox->addStretch();
    vbox->addLayout(hbox);

    if(ENABLE_MOUSE_CURSOR_CAPTURE){
        hbox = new QHBoxLayout;
        mouseCursorCheck.setText(_("Capture the mouse cursor"));
        hbox->addWidget(&mouseCursorCheck);
        hbox->addStretch();
        vbox->addLayout(hbox);
    }

    vbox->addWidget(new HSeparator);
    QDialogButtonBox* buttonBox = new QDialogButtonBox(this);

    recordingToggle.setText(_("&Record"));
    recordingToggle.setDefault(true);
    recordingToggle.sigToggled().connect(
        [this](bool on){ recorderImpl->activateRecording(on, true); });
    buttonBox->addButton(&recordingToggle, QDialogButtonBox::ActionRole);

    vbox->addWidget(buttonBox);
}


void ConfigDialog::showEvent(QShowEvent* event)
{
    updateViewCombo();
    viewManagerConnections.disconnect();
    viewManagerConnections.add(
        ViewManager::sigViewActivated().connect(
            [&](View*){ updateViewComboLater(); }));
    viewManagerConnections.add(
        ViewManager::sigViewDeactivated().connect(
            [&](View*){ updateViewComboLater(); }));
    
    Dialog::showEvent(event);
}


void ConfigDialog::hideEvent(QHideEvent* event)
{
    viewManagerConnections.disconnect();
    updateViewComboLater.cancel();
    Dialog::hideEvent(event);
}


void ConfigDialog::updateViewCombo()
{
    activeViews = ViewManager::activeViews();

    targetViewCombo.blockSignals(true);
    targetViewCombo.clear();
    targetViewCombo.addItem("None");

    bool selected = false;
    for(size_t i=0; i < activeViews.size(); ++i){
        View* view = activeViews[i];
        targetViewCombo.addItem(view->name().c_str());
        if(!selected){
            if((recorderImpl->targetView && (view == recorderImpl->targetView)) ||
               view->name() == recorderImpl->targetViewName){
                targetViewCombo.setCurrentIndex(i + 1);
                selected = true;
            }
        }
    }

    targetViewCombo.blockSignals(false);
}


void ConfigDialog::onTargetViewIndexChanged(int index)
{
    if(index == 0){
        recorderImpl->setTargetView(0);
    } else {
        size_t viewIndex = index - 1;
        if(viewIndex < activeViews.size()){
            recorderImpl->setTargetView(activeViews[viewIndex]);
        }
    }
}


void ConfigDialog::onRecordingModeRadioClicked(int mode)
{
    recorderImpl->recordingMode.select(mode);
}


void ConfigDialog::showDirectorySelectionDialog()
{
    FileDialog dialog;
    dialog.setWindowTitle(_("Select Directory"));
    dialog.setViewMode(QFileDialog::List);
    dialog.setFileMode(QFileDialog::Directory);
    dialog.setOption(QFileDialog::ShowDirsOnly);
    dialog.updatePresetDirectories();

    if(dialog.exec()){
        directoryEntry.setText(dialog.selectedFiles().at(0));
    }
}


double ConfigDialog::startTime() const
{
    return startTimeCheck.isChecked() ? startTimeSpin.value() : 0.0;
}


double ConfigDialog::finishTime() const
{
    return finishTimeCheck.isChecked() ? finishTimeSpin.value() : std::numeric_limits<double>::max();
}


double ConfigDialog::frameRate() const
{
    return fpsSpin.value();
}


double ConfigDialog::timeStep() const
{
    return 1.0 / fpsSpin.value();
}


void ConfigDialog::setViewMarkerCheck(bool on)
{
    viewMarkerCheck.blockSignals(true);
    viewMarkerCheck.setChecked(on);
    viewMarkerCheck.blockSignals(false);
}


void ConfigDialog::checkRecordingModeRadio(int mode)
{
    RadioButton& radio = modeRadioButtons[mode];
    radio.blockSignals(true);
    radio.setChecked(true);
    radio.blockSignals(false);
}


void ConfigDialog::setRecordingToggle(bool on)
{
    recordingToggle.blockSignals(true);
    recordingToggle.setChecked(on);
    recordingToggle.blockSignals(false);
}


bool ConfigDialog::store(Mapping* archive)
{
    archive->write("format", recorderImpl->currentEncoder->formatName(), DOUBLE_QUOTED);
    archive->write("showViewMarker", viewMarkerCheck.isChecked());
    archive->write("directory", directoryEntry.string());
    archive->write("basename", basenameEntry.string());
    archive->write("checkStartTime", startTimeCheck.isChecked());
    archive->write("startTime", startTimeSpin.value());
    archive->write("checkFinishTime", finishTimeCheck.isChecked());
    archive->write("finishTime", finishTimeSpin.value());
    archive->write("fps", fpsSpin.value());
    archive->write("setSize", imageSizeCheck.isChecked());
    archive->write("width", imageWidthSpin.value());
    archive->write("height", imageHeightSpin.value());
    archive->write("mouseCursor", mouseCursorCheck.isChecked());
    return true;
}


void ConfigDialog::restore(const Mapping* archive)
{
    string format;
    if(archive->read("format", format)){
        recorderImpl->setCurrentFormat(format);
    }
    viewMarkerCheck.setChecked(archive->get("showViewMarker", viewMarkerCheck.isChecked()));
    directoryEntry.setText(archive->get("directory", directoryEntry.string()));
    basenameEntry.setText(archive->get("basename", basenameEntry.string()));
    startTimeCheck.setChecked(archive->get("checkStartTime", startTimeCheck.isChecked()));
    startTimeSpin.setValue(archive->get("startTime", startTimeSpin.value()));
    finishTimeCheck.setChecked(archive->get("checkFinishTime", finishTimeCheck.isChecked()));
    finishTimeSpin.setValue(archive->get("finishTime", finishTimeSpin.value()));
    fpsSpin.setValue(archive->get("fps", fpsSpin.value()));
    imageSizeCheck.setChecked(archive->get("setSize", imageSizeCheck.isChecked()));
    imageWidthSpin.setValue(archive->get("width", imageWidthSpin.value()));
    imageHeightSpin.setValue(archive->get("height", imageHeightSpin.value()));
    mouseCursorCheck.setChecked(archive->get("mouseCursor", mouseCursorCheck.isChecked()));
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


bool SequentialNumberedImageFileEncoder::doEncoding(std::string fileBasename)
{
    bool failed = false;
    
    string fFilename(fileBasename + "{:08d}.png");

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
