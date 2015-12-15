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
#include <cnoid/ConnectionSet>
#include <cnoid/Selection>
#include <QPainter>
#include <QDialogButtonBox>
#include <QFileDialog>
#include <QProgressDialog>
#include <boost/thread.hpp>
#include <boost/filesystem.hpp>
#include <boost/bind.hpp>
#include <deque>
#include "gettext.h"

using namespace std;
using namespace cnoid;
namespace filesystem = boost::filesystem;

namespace {

enum RecordinMode { INITIATIVE_MODE, PASSIVE_MODE, DIRECT_MODE, N_RECORDING_MODES };

MovieRecorder* movieRecorder = 0;

class MovieRecorderBar : public ToolBar
{
public:
    MovieRecorderImpl* recorder;
    ToolButton* recordingToggle;
    ToolButton* viewMarkerToggle;
    
    MovieRecorderBar(MovieRecorderImpl* recorder);

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
    ScopedConnection targetViewConnection;

    MessageView* mv;
    char* startMessage;

    ConfigDialog* dialog;
    MovieRecorderBar* toolBar;

    TimeBar* timeBar;
    ConnectionSet timeBarConnections;

    Timer directModeTimer;

    
    ViewMarker* viewMarker;
    Timer viewMarkerBlinkTimer;
    bool isViewMarkerEnabled;

    typedef boost::variant<QPixmap, QImage> ImageVariant;

    class CapturedImage : public Referenced {
    public:
        ImageVariant image;
        int frame;
    };
    typedef ref_ptr<CapturedImage> CapturedImagePtr;

    deque<CapturedImagePtr> capturedImages;
    boost::thread imageOutputThread;
    boost::mutex imageQueueMutex;
    boost::condition_variable imageQueueCondition;
    boost::format filenameFormat;

    MovieRecorderImpl(ExtensionManager* ext);
    ~MovieRecorderImpl();
    void setTargetView(View* view);
    void setTargetView(const std::string& name);
    void onTargetViewRemoved(View* view);
    void setRecordingMode(const std::string& symbol);
    void activateRecording(bool on, bool isActivatedByDialog);
    bool setupViewAndFilenameFormat();
    bool doInitiativeModeRecording();
    void setupPassiveModeRecording();
    void onPlaybackStarted(double time);
    void startPassiveModeRecording();
    bool onTimeChanged(double time);
    void onPlaybackStopped(bool isStoppedManually);
    void startDirectModeRecording();
    void onDirectModeTimerTimeout();
    void captureViewImage();
    void captureSceneWidgets(QWidget* widget, QPixmap& pixmap);
    void startImageOutput();
    void outputImages();
    void stopRecording(bool isFinished);
    void onViewMarkerToggled(bool on);
    bool showViewMarker();
    void stopViewMarkerBlinking();
    void onViewMarkerBlinkTimeout();
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
            ->sigTriggered().connect(boost::bind(&QDialog::show, movieRecorder->impl->dialog));
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
    recordingMode.setSymbol(INITIATIVE_MODE, N_("Initiative"));
    recordingMode.setSymbol(PASSIVE_MODE, N_("Passive"));
    recordingMode.setSymbol(DIRECT_MODE, N_("Direct"));
    
    dialog = new ConfigDialog(this);
    toolBar = new MovieRecorderBar(this);
    ext->addToolBar(toolBar);
    timeBar = TimeBar::instance();

    targetView = 0;
    
    isRecording = false;
    isBeforeFirstFrameCapture = false;
    requestStopRecording = false;

    directModeTimer.sigTimeout().connect(
        boost::bind(&MovieRecorderImpl::onDirectModeTimerTimeout, this));

    startMessage = _("Recording of %1% has been started with the %2% mode.");

    viewMarker = 0;
    isViewMarkerEnabled = false;
    viewMarkerBlinkTimer.setInterval(500);
    viewMarkerBlinkTimer.sigTimeout().connect(
        boost::bind(&MovieRecorderImpl::onViewMarkerBlinkTimeout, this));

    Mapping& config = *AppConfig::archive()->findMapping("MovieRecorder");
    if(config.isValid()){
        restore(config);
    }
}


ConfigDialog::ConfigDialog(MovieRecorderImpl* recorder)
    : recorder(recorder)
{
    setWindowTitle(_("Movie Recorder"));
    
    QVBoxLayout* vbox = new QVBoxLayout();
    setLayout(vbox);

    QHBoxLayout* hbox = new QHBoxLayout();
    hbox->addWidget(new QLabel(_("Target view:")));

    targetViewCombo.sigCurrentIndexChanged().connect(
        boost::bind(&ConfigDialog::onTargetViewIndexChanged, this, _1));
    hbox->addWidget(&targetViewCombo);

    viewMarkerCheck.setText(_("Show the marker"));
    viewMarkerCheck.sigToggled().connect(
        boost::bind(&MovieRecorderImpl::onViewMarkerToggled, recorder, _1));
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
        boost::bind(&ConfigDialog::onRecordingModeRadioClicked, this, _1));
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
        boost::bind(&ConfigDialog::showDirectorySelectionDialog, this));
    hbox->addWidget(&directoryButton);
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout();
    hbox->addWidget(new QLabel(_("Basename")));
    basenameEntry.setText("scene");
    hbox->addWidget(&basenameEntry);
    hbox->addStretch();
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout();
    hbox->addWidget(new QLabel("Frame rate"));
    fpsSpin.setDecimals(1);
    fpsSpin.setRange(1.0, 9999.9);
    fpsSpin.setValue(30.0);
    fpsSpin.setSingleStep(0.1);
    hbox->addWidget(&fpsSpin);
    hbox->addWidget(new QLabel("[fps]"));
    hbox->addStretch();
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout();
    startTimeCheck.setText(_("Start time"));
    hbox->addWidget(&startTimeCheck);
    startTimeSpin.setDecimals(2);
    startTimeSpin.setRange(0.00, 9999.99);
    startTimeSpin.setSingleStep(0.1);
    hbox->addWidget(&startTimeSpin);
    hbox->addWidget(new QLabel("[s]"));
    hbox->addSpacing(4);

    finishTimeCheck.setText(_("Finish time"));
    hbox->addWidget(&finishTimeCheck);
    finishTimeSpin.setDecimals(2);
    finishTimeSpin.setRange(0.00, 9999.99);
    finishTimeSpin.setSingleStep(0.1);
    hbox->addWidget(&finishTimeSpin);
    hbox->addWidget(new QLabel("[s]"));
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

    vbox->addWidget(new HSeparator());
    QDialogButtonBox* buttonBox = new QDialogButtonBox(this);

    recordingToggle.setText(_("&Record"));
    recordingToggle.setDefault(true);
    recordingToggle.sigToggled().connect(
        boost::bind(&MovieRecorderImpl::activateRecording, recorder, _1, true));
    buttonBox->addButton(&recordingToggle, QDialogButtonBox::ActionRole);

    QPushButton* closeButton = new QPushButton(_("&Close"));
    closeButton->setDefault(true);
    buttonBox->addButton(closeButton, QDialogButtonBox::AcceptRole);
    connect(buttonBox,SIGNAL(accepted()), this, SLOT(accept()));

    vbox->addWidget(buttonBox);
}


MovieRecorderBar::MovieRecorderBar(MovieRecorderImpl* recorder)
    : ToolBar(N_("MovieRecorderBar")),
      recorder(recorder)
{
    recordingToggle = addToggleButton("R", _("Toggle Recording"));
    recordingToggle->sigToggled().connect(
        boost::bind(&MovieRecorderImpl::activateRecording, recorder, _1, false));

    viewMarkerToggle = addToggleButton("[ ]", _("Toggle Target View Marker"));
    viewMarkerToggle->sigToggled().connect(
        boost::bind(&MovieRecorderImpl::onViewMarkerToggled, recorder, _1));
    
    addButton(QIcon(":/Base/icons/setup.png"), _("Show the config dialog"))
        ->sigClicked().connect(boost::bind(&QDialog::show, recorder->dialog));
}


MovieRecorder::~MovieRecorder()
{
    delete impl;
}


MovieRecorderImpl::~MovieRecorderImpl()
{
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
    Dialog::showEvent(event);
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
        int viewIndex = index - 1;
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
        targetViewConnection.disconnect();
        targetViewName.clear();
        targetView = view;

        dialog->updateViewCombo();
    
        if(targetView){
            targetViewName = targetView->name();
            targetViewConnection.reset(
                targetView->sigRemoved().connect(
                    boost::bind(&MovieRecorderImpl::onTargetViewRemoved, this, targetView)));
        }
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
        }
    }
}


void MovieRecorderImpl::onTargetViewRemoved(View* view)
{
    if(targetView == view){
        setTargetView(0);
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
            // show a warning dialog
            // stopRecording();
            return;
        }

        if(!setupViewAndFilenameFormat()){
            // show a warning dialog
        } else {
            frame = 0;
            requestStopRecording = false;
            timeStep = dialog->timeStep();
            startTime = dialog->startTime();
            finishTime = dialog->finishTime();
            
            switch(recordingMode.which()){
                
            case INITIATIVE_MODE:
                if(doInitiativeModeRecording()){
                    if(isActivatedByDialog){
                        dialog->hide();
                    }
                }
                break;
                
            case PASSIVE_MODE:
                setupPassiveModeRecording();
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
        return false;
    }

    filesystem::path directory(dialog->directoryEntry.string());
    filesystem::path basename(dialog->basenameEntry.string() + "%08u.png");

    if(directory.empty()){
        showWarningDialog(_("Please set a directory to output image files."));
        return false;

    } else {
        if(filesystem::exists(directory)){
            if(!filesystem::is_directory(directory)){
                showWarningDialog(fmt(_("%1% is not a directory.")) % directory);
                return false;
            }
        } else {
            filesystem::create_directories(directory);
        }
    }

    filenameFormat = boost::format((directory / basename).string());

    if(dialog->imageSizeCheck.isChecked()){
        int width = dialog->imageWidthSpin.value();
        int height = dialog->imageHeightSpin.value();
        QSize s = targetView->size();
        int x = (s.width() - width) / 2;
        int y = (s.height() - height) / 2;
        targetView->setGeometry(x, y, width, height);
    }

    boost::unique_lock<boost::mutex> lock(imageQueueMutex);
    capturedImages.clear();
    
    return true;
}


bool MovieRecorderImpl::doInitiativeModeRecording()
{
    double time = startTime;
    bool doContinue = true;

    isRecording = true;

    showViewMarker();
    startImageOutput();

    mv->putln(boost::format(startMessage) % targetView->name() % recordingMode.selectedLabel());
    
    while(time <= finishTime && doContinue){

        doContinue = timeBar->setTime(time);

        MessageView::instance()->flush();

        if(requestStopRecording){
            break;
        }

        captureViewImage();

        /*
        if(isSaveFailed){
            requestStopRecording = true;
            break;
        }
        */

        time += timeStep;
        frame++;
    }

    stopRecording(!requestStopRecording);
    bool result = !requestStopRecording;
    requestStopRecording = false;

    return result;
}


void MovieRecorderImpl::setupPassiveModeRecording()
{
    timeBarConnections.disconnect();
        
    isBeforeFirstFrameCapture = true;
            
    if(timeBar->isDoingPlayback()){
        startPassiveModeRecording();
    } else {
        timeBarConnections.add(
            timeBar->sigPlaybackStarted().connect(
                boost::bind(&MovieRecorderImpl::onPlaybackStarted, this, _1)));
        
        mv->putln(boost::format(_("The passive mode recording for %1% is ready.")) % targetView->name());
    }
}


void MovieRecorderImpl::onPlaybackStarted(double time)
{
    startPassiveModeRecording();
}


void MovieRecorderImpl::startPassiveModeRecording()
{
    timeBarConnections.disconnect();
    
    timeBarConnections.add(
        timeBar->sigTimeChanged().connect(
            boost::bind(&MovieRecorderImpl::onTimeChanged, this, _1)));

    timeBarConnections.add(
        timeBar->sigPlaybackStopped().connect(
            boost::bind(&MovieRecorderImpl::onPlaybackStopped, this, _2)));

    isRecording = true;
    showViewMarker();
    startImageOutput();

    mv->putln(boost::format(startMessage) % targetView->name() % recordingMode.selectedLabel());
}


bool MovieRecorderImpl::onTimeChanged(double time)
{
    if(isRecording){
        if(isBeforeFirstFrameCapture){
            if(time > startTime){
                nextFrameTime = time;
            } else {
                nextFrameTime = startTime;
            }
        }
        if(time > finishTime){
            stopRecording(true);
        } else {
            while(time >= nextFrameTime){
                captureViewImage();
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
    showViewMarker();
    startImageOutput();
    directModeTimer.setInterval(1000 / dialog->frameRate());
    directModeTimer.start();

    mv->putln(boost::format(startMessage) % targetView->name() % recordingMode.selectedLabel());
}


void MovieRecorderImpl::onDirectModeTimerTimeout()
{
    captureViewImage();
    ++frame;
}


void MovieRecorderImpl::captureViewImage()
{
    CapturedImagePtr captured = new CapturedImage();
    captured->frame = frame;
    
    if(SceneView* sceneView = dynamic_cast<SceneView*>(targetView)){
        captured->image = sceneView->sceneWidget()->getImage();
    } else {
        captured->image = QPixmap();
        QPixmap& pixmap = boost::get<QPixmap>(captured->image);
        pixmap.grabWidget(targetView);
        captureSceneWidgets(targetView, pixmap);
    }

    {
        boost::unique_lock<boost::mutex> lock(imageQueueMutex);
        capturedImages.push_back(captured);
    }
    imageQueueCondition.notify_all();
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
        imageOutputThread = boost::thread(
            boost::bind(&MovieRecorderImpl::outputImages, this));
    }
}


void MovieRecorderImpl::outputImages()
{
    while(true){
        CapturedImagePtr captured;
        {
            boost::unique_lock<boost::mutex> lock(imageQueueMutex);
            while(isRecording && capturedImages.empty()){
                imageQueueCondition.wait(lock);
            }
            if(capturedImages.empty() && !isRecording){
                break;
            }
            captured = capturedImages.front();
            capturedImages.pop_front();
        }
        bool saved = false;

        string filename = str(filenameFormat % captured->frame);
        
        if(captured->image.which() == 0){
            QPixmap& pixmap = boost::get<QPixmap>(captured->image);
            saved = pixmap.save(filename.c_str());
        } else {
            QImage& image = boost::get<QImage>(captured->image);
            saved = image.save(filename.c_str());
        }
        /*
        if(!saved){
            showWarningDialog(fmt(_("Saving an image to \"%1%\" failed.")) % filename);
        }
        */
    }
}


void MovieRecorderImpl::stopRecording(bool isFinished)
{
    directModeTimer.stop();
    
    if(isRecording){

        stopViewMarkerBlinking();
        
        int numRemainingImages = 0;
        {
            boost::unique_lock<boost::mutex> lock(imageQueueMutex);
            numRemainingImages = capturedImages.size();
        }
        if(numRemainingImages > 1){
            QProgressDialog progress(_("Outputting sequential image files..."), _("Abort Output"), 0, numRemainingImages, MainWindow::instance());
            progress.setWindowTitle(_("Movie Recorder's Output Status"));
            progress.setWindowModality(Qt::WindowModal);
            while(true){
                int index;
                {
                    boost::unique_lock<boost::mutex> lock(imageQueueMutex);
                    index = numRemainingImages - capturedImages.size();
                }
                progress.setValue(index);

                if(progress.wasCanceled()){
                    boost::unique_lock<boost::mutex> lock(imageQueueMutex);
                    capturedImages.clear();
                    break;
                }
                if(index < numRemainingImages){
                    boost::this_thread::sleep(boost::posix_time::milliseconds(10));
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
            mv->putln(boost::format(_("Recording of %1% has been finished.")) % targetView->name());
        } else {
            mv->putln(boost::format(_("Recording of %1% has been stopped.")) % targetView->name());
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
    
    stopViewMarkerBlinking();
    
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
        if(isRecording){
            viewMarkerBlinkTimer.start();
        }
        return true;
    }
    return false;
}


void MovieRecorderImpl::stopViewMarkerBlinking()
{
    viewMarkerBlinkTimer.stop();
}

    
void MovieRecorderImpl::onViewMarkerBlinkTimeout()
{
    if(viewMarker){
        if(viewMarker->isVisible()){
            viewMarker->hide();
        } else {
            viewMarker->show();
        }
    }
}


ViewMarker::ViewMarker(MovieRecorderImpl* recorder)
    : recorder(recorder)
{
    setWindowFlags(Qt::Widget | Qt::FramelessWindowHint);
    setAttribute(Qt::WA_NoSystemBackground);
    setAttribute(Qt::WA_PaintOnScreen);
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


void ViewMarker::paintEvent(QPaintEvent* event)
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
}
