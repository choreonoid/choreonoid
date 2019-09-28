/**
   @author Shin'ichiro Nakaoka
*/

#include "MovieRecorder.h"
#include "ExtensionManager.h"
#include "ViewManager.h"
#include "MenuManager.h"
#include "ViewArea.h"
#include "MessageView.h"
#include "AppConfig.h"
#include "MainWindow.h"
#include "SceneView.h"
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
#include "Separator.h"
#include "Timer.h"
#include "LazyCaller.h"
#include <cnoid/ConnectionSet>
#include <cnoid/Selection>
#include <cnoid/stdx/variant>
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
using fmt::format;

namespace {

enum RecordinMode { OFFLINE_MODE, ONLINE_MODE, DIRECT_MODE, N_RECORDING_MODES };

MovieRecorder* movieRecorder = nullptr;

class MovieRecorderBar : public ToolBar
{
public:
    MovieRecorderImpl* recorder;
    ToolButton* recordingToggle;
    ToolButton* viewMarkerToggle;
    bool isFlashed;
    
    MovieRecorderBar(MovieRecorderImpl* recorder);

    void flashRecordingToggle(bool setNormal){
        if(isFlashed || setNormal){
            recordingToggle->setText(_("O"));
            isFlashed = false;
        } else {
            recordingToggle->setText(_("*"));
            isFlashed = true;
        }
    }

    void setRecordingToggle(bool on){
        recordingToggle->blockSignals(true);
        recordingToggle->setChecked(on);
        recordingToggle->blockSignals(false);
    }
    
    void changeViewMarkerToggleState(bool on){
        viewMarkerToggle->blockSignals(true);
        viewMarkerToggle->setChecked(on);
        viewMarkerToggle->blockSignals(false);
    }
};

class ConfigDialog : public Dialog
{
public:
    MovieRecorderImpl* recorder;
    ScopedConnectionSet viewManagerConnections;
    LazyCaller updateViewComboLater;
    vector<View*> activeViews;
    ComboBox targetViewCombo;
    CheckBox viewMarkerCheck;
    RadioButton modeRadioButtons[N_RECORDING_MODES];
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

    double startTime() const {
        return startTimeCheck.isChecked() ? startTimeSpin.value() : 0.0;
    }
    double finishTime() const {
        return finishTimeCheck.isChecked() ? finishTimeSpin.value() : std::numeric_limits<double>::max();
    }
    double frameRate() const {
        return fpsSpin.value();
    }
    double timeStep() const {
        return 1.0 / fpsSpin.value();
    }
    void setViewMarkerCheck(bool on){
        viewMarkerCheck.blockSignals(true);
        viewMarkerCheck.setChecked(on);
        viewMarkerCheck.blockSignals(false);
    }
    void checkRecordingModeRadio(int mode){
        RadioButton& radio = modeRadioButtons[mode];
        radio.blockSignals(true);
        radio.setChecked(true);
        radio.blockSignals(false);
    }
    void setRecordingToggle(bool on){
        recordingToggle.blockSignals(true);
        recordingToggle.setChecked(on);
        recordingToggle.blockSignals(false);
    }

    ConfigDialog(MovieRecorderImpl* recorder);
    virtual void showEvent(QShowEvent* event);
    virtual void hideEvent(QHideEvent* event);
    void updateViewCombo();
    void onTargetViewIndexChanged(int index);
    void onRecordingModeRadioClicked(int mode);
    void showDirectorySelectionDialog();
    bool store(Mapping& archive);
    void restore(const Mapping& archive);
};


class ViewMarker : public QWidget
{
public:
    MovieRecorderImpl* recorder;
    QPen pen;
    ViewMarker(MovieRecorderImpl* recorder);
    void setTargetView(View* view);
    virtual void paintEvent(QPaintEvent* event);
};

}

namespace cnoid {

class MovieRecorderImpl
{
public:
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

    typedef stdx::variant<QPixmap, QImage> ImageVariant;

    class CapturedImage : public Referenced {
    public:
        ImageVariant image;
        int frame;
    };
    typedef ref_ptr<CapturedImage> CapturedImagePtr;

    deque<CapturedImagePtr> capturedImages;
    vector<quint32> tmpImageBuf;
    std::thread imageOutputThread;
    std::mutex imageQueueMutex;
    std::condition_variable imageQueueCondition;
   string filenameFormat;

    MovieRecorderImpl(ExtensionManager* ext);
    ~MovieRecorderImpl();
    void setTargetView(View* view);
    void onTargetViewResized(View* view);
    void onTargetViewRemoved(View* view);
    void setTargetView(const std::string& name);
    void onViewCreated(View* view);
    void setRecordingMode(const std::string& symbol);
    void activateRecording(bool on, bool isActivatedByDialog);
    bool setupViewAndFilenameFormat();
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
    void startImageOutput();
    void outputImages();
    void onImageOutputFailed(std::string message);
    void stopRecording(bool isFinished);
    void onViewMarkerToggled(bool on);
    bool showViewMarker();
    void startFlash();
    void onFlashTimeout();
    void stopFlash();
    bool store(Mapping& archive);
    void restore(const Mapping& archive);
};

}


void MovieRecorder::initialize(ExtensionManager* ext)
{
    if(!movieRecorder){
        movieRecorder = ext->manage(new MovieRecorder(ext));

        MenuManager& mm = ext->menuManager();
        mm.setPath("/Tools");
        mm.addItem(_("Movie Recorder"))
            ->sigTriggered().connect([](){ movieRecorder->impl->dialog->show(); });
    }
}


MovieRecorder* MovieRecorder::instance()
{
    return movieRecorder;
}


MovieRecorder::MovieRecorder(ExtensionManager* ext)
{
    impl = new MovieRecorderImpl(ext);
}


MovieRecorderImpl::MovieRecorderImpl(ExtensionManager* ext)
    : recordingMode(N_RECORDING_MODES, CNOID_GETTEXT_DOMAIN_NAME),
      mv(MessageView::instance())

{
    recordingMode.setSymbol(OFFLINE_MODE, N_("Offline"));
    recordingMode.setSymbol(ONLINE_MODE, N_("Online"));
    recordingMode.setSymbol(DIRECT_MODE, N_("Direct"));
    
    dialog = new ConfigDialog(this);
    toolBar = new MovieRecorderBar(this);
    ext->addToolBar(toolBar);
    timeBar = TimeBar::instance();

    targetView = 0;
    
    isRecording = false;
    isBeforeFirstFrameCapture = false;
    requestStopRecording = false;

    directModeTimer.sigTimeout().connect([&](){ onDirectModeTimerTimeout(); });

    startMessage = _("Recording of {0} has been started with the {1} mode.");

    viewMarker = 0;
    isViewMarkerEnabled = false;
    flashTimer.setInterval(500);
    flashTimer.sigTimeout().connect([&](){ onFlashTimeout(); });

    Mapping& config = *AppConfig::archive()->findMapping("MovieRecorder");
    if(config.isValid()){
        restore(config);
    }
}


ConfigDialog::ConfigDialog(MovieRecorderImpl* recorder)
    : recorder(recorder),
      updateViewComboLater([&](){ updateViewCombo(); })
{
    setWindowTitle(_("Movie Recorder"));
    
    QVBoxLayout* vbox = new QVBoxLayout();
    setLayout(vbox);

    QHBoxLayout* hbox = new QHBoxLayout();
    hbox->addWidget(new QLabel(_("Target view:")));

    targetViewCombo.sigCurrentIndexChanged().connect(
        [&](int index){ onTargetViewIndexChanged(index); });
    hbox->addWidget(&targetViewCombo);

    viewMarkerCheck.setText(_("Show the marker"));
    viewMarkerCheck.sigToggled().connect(
        [this](bool on){ this->recorder->onViewMarkerToggled(on); });
    hbox->addWidget(&viewMarkerCheck);
    
    hbox->addStretch();
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout();
    hbox->addWidget(new QLabel(_("Recording mode: ")));
    ButtonGroup* modeGroup = new ButtonGroup();
    for(int i=0; i < N_RECORDING_MODES; ++i){
        RadioButton* radio = &modeRadioButtons[i];
        radio->setText(recorder->recordingMode.label(i));
        modeGroup->addButton(radio, i);
        hbox->addWidget(radio);
    }
    modeRadioButtons[0].setChecked(true);
    modeGroup->sigButtonClicked().connect(
        [&](int mode){ onRecordingModeRadioClicked(mode); });
    hbox->addStretch();
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout();
    hbox->addWidget(new QLabel(_("Directory")));
    hbox->addWidget(&directoryEntry);

    QIcon folderIcon = QIcon::fromTheme("folder");
    if(folderIcon.isNull()){
        directoryButton.setText(_("Select"));
    } else {
        directoryButton.setIcon(folderIcon);
    }
    directoryButton.sigClicked().connect(
        [&](){ showDirectorySelectionDialog(); });
    hbox->addWidget(&directoryButton);
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout();
    hbox->addWidget(new QLabel(_("Basename")));
    basenameEntry.setText("scene");
    hbox->addWidget(&basenameEntry);
    hbox->addStretch();
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout();
    hbox->addWidget(new QLabel(_("Frame rate")));
    fpsSpin.setDecimals(1);
    fpsSpin.setRange(1.0, 9999.9);
    fpsSpin.setValue(30.0);
    fpsSpin.setSingleStep(0.1);
    hbox->addWidget(&fpsSpin);
    hbox->addWidget(new QLabel(_("[fps]")));
    hbox->addStretch();
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout();
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
    
    hbox = new QHBoxLayout();
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
        hbox = new QHBoxLayout();
        mouseCursorCheck.setText(_("Capture the mouse cursor"));
        hbox->addWidget(&mouseCursorCheck);
        hbox->addStretch();
        vbox->addLayout(hbox);
    }

    vbox->addWidget(new HSeparator());
    QDialogButtonBox* buttonBox = new QDialogButtonBox(this);

    recordingToggle.setText(_("&Record"));
    recordingToggle.setDefault(true);
    recordingToggle.sigToggled().connect(
        [this](bool on){ this->recorder->activateRecording(on, true); });
    buttonBox->addButton(&recordingToggle, QDialogButtonBox::ActionRole);

    vbox->addWidget(buttonBox);
}


MovieRecorderBar::MovieRecorderBar(MovieRecorderImpl* recorder)
    : ToolBar(N_("MovieRecorderBar")),
      recorder(recorder)
{
    
    recordingToggle = addToggleButton("O", _("Toggle Recording"));
    recordingToggle->sigToggled().connect(
        [this](bool on){ this->recorder->activateRecording(on, false); });

    viewMarkerToggle = addToggleButton("[ ]", _("Toggle Target View Marker"));
    viewMarkerToggle->sigToggled().connect(
        [this](bool on){ this->recorder->onViewMarkerToggled(on); });
    
    addButton(QIcon(":/Base/icons/setup.png"), _("Show the config dialog"))
        ->sigClicked().connect([this](){ this->recorder->dialog->show(); });
}


MovieRecorder::~MovieRecorder()
{
    delete impl;
}


MovieRecorderImpl::~MovieRecorderImpl()
{
    if(imageOutputThread.joinable()){
        requestStopRecording = true;
        imageQueueCondition.notify_all();
        imageOutputThread.join();
    }
    
    timeBarConnections.disconnect();
    store(*AppConfig::archive()->openMapping("MovieRecorder"));
    delete dialog;

    if(viewMarker){
        delete viewMarker;
    }
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
            if((recorder->targetView && (view == recorder->targetView)) ||
               view->name() == recorder->targetViewName){
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
        recorder->setTargetView(0);
    } else {
        size_t viewIndex = index - 1;
        if(viewIndex < activeViews.size()){
            recorder->setTargetView(activeViews[viewIndex]);
        }
    }
}


void MovieRecorderImpl::setTargetView(View* view)
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


void MovieRecorderImpl::onTargetViewResized(View* view)
{
    if(targetView == view){
        showViewMarker();
    }
}


void MovieRecorderImpl::onTargetViewRemoved(View* view)
{
    if(targetView == view){
        setTargetView(0);
    }
}


void MovieRecorderImpl::setTargetView(const std::string& name)
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


void MovieRecorderImpl::onViewCreated(View* view)
{
    if(view->name() == targetViewName){
        setTargetView(view);
    }
}


void MovieRecorderImpl::setRecordingMode(const std::string& symbol)
{
    recordingMode.select(symbol);
    dialog->checkRecordingModeRadio(recordingMode.which());
}


void ConfigDialog::onRecordingModeRadioClicked(int mode)
{
    recorder->recordingMode.select(mode);
}


void ConfigDialog::showDirectorySelectionDialog()
{
    QString directory =
        QFileDialog::getExistingDirectory(
            MainWindow::instance(),
            _("Select Directory"),
            directoryEntry.text(),
            0);
    if(!directory.isNull()){
        directoryEntry.setText(directory);
    }
}


void MovieRecorderImpl::activateRecording(bool on, bool isActivatedByDialog)
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

        if(setupViewAndFilenameFormat()){
            frame = 0;
            requestStopRecording = false;
            timeStep = dialog->timeStep();
            startTime = dialog->startTime();
            finishTime = dialog->finishTime();
            
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


bool MovieRecorderImpl::setupViewAndFilenameFormat()
{
    if(!targetView){
        showWarningDialog(_("Target view is not specified."));
        return false;
    }

    filesystem::path directory(dialog->directoryEntry.string());
    filesystem::path basename(dialog->basenameEntry.string() + "{:08d}.png");

    if(directory.empty()){
        showWarningDialog(_("Please set a directory to output image files."));
        return false;

    } else {
        if(filesystem::exists(directory)){
            if(!filesystem::is_directory(directory)){
                showWarningDialog(format(_("{} is not a directory."), directory));
                return false;
            }
        } else {
            filesystem::create_directories(directory);
        }
    }

    filenameFormat = (directory / basename).string();

    if(dialog->imageSizeCheck.isChecked()){
        int width = dialog->imageWidthSpin.value();
        int height = dialog->imageHeightSpin.value();
        QSize s = targetView->size();
        int x = (s.width() - width) / 2;
        int y = (s.height() - height) / 2;
        targetView->setGeometry(x, y, width, height);
    }

    std::lock_guard<std::mutex> lock(imageQueueMutex);
    capturedImages.clear();
    
    return true;
}


bool MovieRecorderImpl::doOfflineModeRecording()
{
    double time = startTime;
    bool doContinue = true;

    isRecording = true;

    startFlash();
    startImageOutput();

    mv->putln(format(startMessage, targetView->name(), recordingMode.selectedLabel()));
    
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


void MovieRecorderImpl::setupOnlineModeRecording()
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
        
        mv->putln(format(_("The online mode recording for {} is ready."), targetView->name()));
    }
}


void MovieRecorderImpl::startOnlineModeRecording()
{
    timeBarConnections.disconnect();
    
    timeBarConnections.add(
        timeBar->sigTimeChanged().connect(
            [&](double time){ return onTimeChanged(time); }));

    timeBarConnections.add(
        timeBar->sigPlaybackStopped().connect(
            [&](double /* time */, bool isStoppedManually){ onPlaybackStopped(isStoppedManually); }));

    isRecording = true;
    startImageOutput();

    mv->putln(format(startMessage, targetView->name(), recordingMode.selectedLabel()));
}


bool MovieRecorderImpl::onTimeChanged(double time)
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


void MovieRecorderImpl::onPlaybackStopped(bool isStoppedManually)
{
    stopRecording(!isStoppedManually);
}


void MovieRecorderImpl::startDirectModeRecording()
{
    isRecording = true;
    startFlash();
    startImageOutput();
    directModeTimer.setInterval(1000 / dialog->frameRate());
    directModeTimer.start();

    mv->putln(format(startMessage, targetView->name(), recordingMode.selectedLabel()));
}


void MovieRecorderImpl::onDirectModeTimerTimeout()
{
    captureViewImage(false);
    ++frame;
}


void MovieRecorderImpl::captureViewImage(bool waitForPrevOutput)
{
    CapturedImagePtr captured = new CapturedImage();
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


void MovieRecorderImpl::drawMouseCursorImage(QPainter& painter)
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


void MovieRecorderImpl::captureSceneWidgets(QWidget* widget, QPixmap& pixmap)
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


void MovieRecorderImpl::startImageOutput()
{
    if(!imageOutputThread.joinable()){
        imageOutputThread = std::thread(
            [&](){ outputImages(); });
    }
}


void MovieRecorderImpl::outputImages()
{
    while(true){
        CapturedImagePtr captured;
        {
            std::unique_lock<std::mutex> lock(imageQueueMutex);
            while(isRecording && capturedImages.empty()){
                imageQueueCondition.wait(lock);
            }
            if(capturedImages.empty() && !isRecording){
                break;
            }
            captured = capturedImages.front();
            capturedImages.pop_front();
        }
        imageQueueCondition.notify_all();
        
        bool saved = false;

        string filename = format(filenameFormat, captured->frame);
        
        if(stdx::get_variant_index(captured->image) == 0){
            QPixmap& pixmap = stdx::get<QPixmap>(captured->image);
            saved = pixmap.save(filename.c_str());
        } else {
            QImage& image = stdx::get<QImage>(captured->image);
            saved = image.save(filename.c_str());
        }

        if(!saved){
            string message = format(_("Saving an image to \"{}\" failed."), filename);
            callLater([this, message](){ onImageOutputFailed(message); });
            {
                std::lock_guard<std::mutex> lock(imageQueueMutex);
                capturedImages.clear();
            }
            imageQueueCondition.notify_all();
            break;
        }
    }
}


void MovieRecorderImpl::onImageOutputFailed(std::string message)
{
    showWarningDialog(message);
    stopRecording(false);
}


void MovieRecorderImpl::stopRecording(bool isFinished)
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
            QProgressDialog progress(_("Outputting sequential image files..."), _("Abort Output"), 0, numRemainingImages, MainWindow::instance());
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
        imageOutputThread.join();

        if(isFinished){
            mv->putln(format(_("Recording of {} has been finished."), targetView->name()));
        } else {
            mv->putln(format(_("Recording of {} has been stopped."), targetView->name()));
        }
    }
    
    timeBarConnections.disconnect();

    toolBar->setRecordingToggle(false);
    dialog->setRecordingToggle(false);
}


void MovieRecorderImpl::onViewMarkerToggled(bool on)
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


bool MovieRecorderImpl::showViewMarker()
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


void MovieRecorderImpl::startFlash()
{
    showViewMarker();
    flashTimer.start();
}


void MovieRecorderImpl::onFlashTimeout()
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


void MovieRecorderImpl::stopFlash()
{
    flashTimer.stop();
    toolBar->flashRecordingToggle(true);
    if(isViewMarkerEnabled && viewMarker){
        viewMarker->show();
    }
}


ViewMarker::ViewMarker(MovieRecorderImpl* recorder)
    : recorder(recorder)
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


bool MovieRecorderImpl::store(Mapping& archive)
{
    if(targetView){
        archive.write("target", targetView->name());
    }
    archive.write("recordingMode", recordingMode.selectedSymbol());
    return dialog->store(archive);
}


bool ConfigDialog::store(Mapping& archive)
{
    archive.write("showViewMarker", viewMarkerCheck.isChecked());
    archive.write("directory", directoryEntry.string());
    archive.write("basename", basenameEntry.string());
    archive.write("checkStartTime", startTimeCheck.isChecked());
    archive.write("startTime", startTimeSpin.value());
    archive.write("checkFinishTime", finishTimeCheck.isChecked());
    archive.write("finishTime", finishTimeSpin.value());
    archive.write("fps", fpsSpin.value());
    archive.write("setSize", imageSizeCheck.isChecked());
    archive.write("width", imageWidthSpin.value());
    archive.write("height", imageHeightSpin.value());
    archive.write("mouseCursor", mouseCursorCheck.isChecked());
    return true;
}


void MovieRecorderImpl::restore(const Mapping& archive)
{
    string symbol;
    if(archive.read("target", symbol)){
        setTargetView(symbol);
    }
    if(archive.read("recordingMode", symbol)){
        setRecordingMode(symbol);
    }
    dialog->restore(archive);
}


void ConfigDialog::restore(const Mapping& archive)
{
    viewMarkerCheck.setChecked(archive.get("showViewMarker", viewMarkerCheck.isChecked()));
    directoryEntry.setText(archive.get("directory", directoryEntry.string()));
    basenameEntry.setText(archive.get("basename", basenameEntry.string()));
    startTimeCheck.setChecked(archive.get("checkStartTime", startTimeCheck.isChecked()));
    startTimeSpin.setValue(archive.get("startTime", startTimeSpin.value()));
    finishTimeCheck.setChecked(archive.get("checkFinishTime", finishTimeCheck.isChecked()));
    finishTimeSpin.setValue(archive.get("finishTime", finishTimeSpin.value()));
    fpsSpin.setValue(archive.get("fps", fpsSpin.value()));
    imageSizeCheck.setChecked(archive.get("setSize", imageSizeCheck.isChecked()));
    imageWidthSpin.setValue(archive.get("width", imageWidthSpin.value()));
    imageHeightSpin.setValue(archive.get("height", imageHeightSpin.value()));
    mouseCursorCheck.setChecked(archive.get("mouseCursor", mouseCursorCheck.isChecked()));
}
