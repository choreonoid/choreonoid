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
#include <QPainter>
#include <QDialogButtonBox>
#include <QFileDialog>
#include <boost/filesystem.hpp>
#include <boost/bind.hpp>
#include "gettext.h"

using namespace std;
using namespace cnoid;
namespace filesystem = boost::filesystem;
using boost::format;

namespace {

MovieRecorder* movieRecorder = 0;

class MovieRecorderBar : public ToolBar
{
public:
    MovieRecorderImpl* recorder;
    ToolButton* recordingToggle;
    ToolButton* viewMarkerToggle;
    
    MovieRecorderBar(MovieRecorderImpl* recorder);
    void onRecordingButtonToggled(bool on);

    void changeRecordingToggleState(bool on){
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
    RadioButton initiativeModeRadio;
    RadioButton passiveModeRadio;
    RadioButton directModeRadio;

    double startTime() const {
        return startTimeCheck.isChecked() ? startTimeSpin.value() : 0.0;
    }
    double finishTime() const {
        return finishTimeCheck.isChecked() ? finishTimeSpin.value() : std::numeric_limits<double>::max();
    }
    double timeStep() const {
        return 1.0 / fpsSpin.value();
    }
    void changeViewMarkerCheckState(bool on){
        viewMarkerCheck.blockSignals(true);
        viewMarkerCheck.setChecked(on);
        viewMarkerCheck.blockSignals(false);
    }

    ConfigDialog(MovieRecorderImpl* recorder);
    virtual void showEvent(QShowEvent* event);
    virtual void hideEvent(QHideEvent* event);
    void updateViewCombo();
    void onTargetViewIndexChanged(int index);
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
    ConfigDialog* config;
    MovieRecorderBar* toolBar;
    TimeBar* timeBar;
    string targetViewName;
    View* targetView;
    ScopedConnection targetViewConnection;
    bool isRecording;
    bool isBeforeFirstFrameCapture;
    bool requestStopRecording;
    int frame;
    double nextFrameTime;
    double timeStep;
    double startTime;
    double finishTime;
    boost::format filenameFormat;
    ConnectionSet timeBarConnections;

    ViewMarker* viewMarker;
    Timer viewMarkerBlinkTimer;
    bool isViewMarkerActive;

    MovieRecorderImpl(ExtensionManager* ext);
    ~MovieRecorderImpl();
    void setTargetView(View* view);
    void setTargetView(const std::string& name);
    void onTargetViewRemoved(View* view);
    void onViewMarkerToggled(bool on);
    void startViewMarkerBlinking();
    void stopViewMarkerBlinking();
    void onViewMarkerBlinkTimeout();
    void requestRecording(bool isFromConfigDialog);
    void stopRecording();
    bool doInitiativeModeRecording();
    void setupPassiveModeRecording();
    void onPlaybackStarted(double time);
    void startPassiveModeRecording();
    bool setupViewAndFilenameFormat();
    bool onTimeChanged(double time);
    bool saveViewImage();
    void captureSceneWidgets(QWidget* widget, QPixmap& pixmap);
    void onPlaybackStopped();
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
            ->sigTriggered().connect(boost::bind(&QDialog::show, movieRecorder->impl->config));
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
{
    config = new ConfigDialog(this);
    toolBar = new MovieRecorderBar(this);
    ext->addToolBar(toolBar);
    timeBar = TimeBar::instance();

    targetView = 0;
    
    isRecording = false;
    isBeforeFirstFrameCapture = false;
    requestStopRecording = false;

    viewMarker = 0;
    isViewMarkerActive = false;
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
    setWindowTitle(_("Config and Execution of the Movie Recorder"));
    
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
    initiativeModeRadio.setText(_("Initiative"));
    initiativeModeRadio.setChecked(true);
    modeGroup->addButton(&initiativeModeRadio);
    hbox->addWidget(&initiativeModeRadio);
    passiveModeRadio.setText(_("Passive"));
    modeGroup->addButton(&passiveModeRadio);
    hbox->addWidget(&passiveModeRadio);
    directModeRadio.setText(_("Direct"));
    directModeRadio.setEnabled(false);
    modeGroup->addButton(&directModeRadio);
    hbox->addWidget(&directModeRadio);
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

    PushButton* recordButton = new PushButton(_("&Record"));
    recordButton->setDefault(true);
    buttonBox->addButton(recordButton, QDialogButtonBox::ActionRole);
    recordButton->sigClicked().connect(boost::bind(&MovieRecorderImpl::requestRecording, recorder, true));

    PushButton* stopButton = new PushButton(_("&Stop"));
    stopButton->setDefault(true);
    buttonBox->addButton(stopButton, QDialogButtonBox::ActionRole);
    stopButton->sigClicked().connect(boost::bind(&MovieRecorderImpl::stopRecording, recorder));
    
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
            boost::bind(&MovieRecorderBar::onRecordingButtonToggled, this, _1));

    viewMarkerToggle = addToggleButton("[ ]", _("Toggle Target View Marker"));
    viewMarkerToggle->sigToggled().connect(
            boost::bind(&MovieRecorderImpl::onViewMarkerToggled, recorder, _1));
    
    addButton(QIcon(":/Base/icons/setup.png"), _("Show the config dialog"))
        ->sigClicked().connect(boost::bind(&QDialog::show, recorder->config));
}


MovieRecorder::~MovieRecorder()
{
    delete impl;
}


MovieRecorderImpl::~MovieRecorderImpl()
{
    timeBarConnections.disconnect();
    store(*AppConfig::archive()->openMapping("MovieRecorder"));
    delete config;

    if(viewMarker){
        delete viewMarker;
    }
}


void ConfigDialog::showEvent(QShowEvent* event)
{
    updateViewCombo();
}


void ConfigDialog::hideEvent(QHideEvent* event)
{

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
            stopRecording();
        }
        targetViewConnection.disconnect();
        targetViewName.clear();
        targetView = view;

        config->updateViewCombo();
    
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


void MovieRecorderBar::onRecordingButtonToggled(bool on)
{
    if(on){
        recorder->requestRecording(false);
    } else {
        recorder->stopRecording();
    }
}


void MovieRecorderImpl::requestRecording(bool isFromConfigDialog)
{
    toolBar->changeRecordingToggleState(true);
    
    if(config->initiativeModeRadio.isChecked()){
        if(doInitiativeModeRecording()){
            if(isFromConfigDialog){
                config->hide();
            }
        }
    } else if(config->passiveModeRadio.isChecked()){
        setupPassiveModeRecording();
    } else if(config->directModeRadio.isChecked()){

    }
}


void MovieRecorderImpl::stopRecording()
{
    if(isRecording){
        isRecording = false;
        requestStopRecording = true;
    }
    timeBarConnections.disconnect();
    stopViewMarkerBlinking();

    toolBar->changeRecordingToggleState(false);
}


bool MovieRecorderImpl::doInitiativeModeRecording()
{
    if(!setupViewAndFilenameFormat()){
        return false;
    }
    
    isRecording = true;
    requestStopRecording = false;
    frame = 0;
    timeStep = config->timeStep();

    double time = config->startTime();
    double finishTime = config->finishTime();
    bool doContinue = true;

    startViewMarkerBlinking();
    
    while(time <= finishTime && doContinue){

        doContinue = timeBar->setTime(time);

        MessageView::instance()->flush();

        if(requestStopRecording){
            break;
        }

        if(!saveViewImage()){
            requestStopRecording = true;
            break;
        }

        time += timeStep;
        frame++;
    }

    startViewMarkerBlinking();
    stopRecording();
    bool result = !requestStopRecording;
    requestStopRecording = false;

    return result;
}


void MovieRecorderImpl::setupPassiveModeRecording()
{
    if(!isRecording){

        timeBarConnections.disconnect();
        
        if(setupViewAndFilenameFormat()){
            isBeforeFirstFrameCapture = true;
            frame = 0;
            startTime = config->startTime();
            finishTime = config->finishTime();
            timeStep = config->timeStep();
            
            if(timeBar->isDoingPlayback()){
                startPassiveModeRecording();
            } else {
                timeBarConnections.add(
                    timeBar->sigPlaybackStarted().connect(
                        boost::bind(&MovieRecorderImpl::onPlaybackStarted, this, _1)));
            }
        }
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
            boost::bind(&MovieRecorderImpl::onPlaybackStopped, this)));

    isRecording = true;
    startViewMarkerBlinking();
}


bool MovieRecorderImpl::setupViewAndFilenameFormat()
{
    if(!targetView){
        return false;
    }

    filesystem::path directory(config->directoryEntry.string());
    filesystem::path basename(config->basenameEntry.string() + "%08u.png");

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

    filenameFormat = format((directory / basename).string());

    if(config->imageSizeCheck.isChecked()){
        int width = config->imageWidthSpin.value();
        int height = config->imageHeightSpin.value();
        QSize s = targetView->size();
        int x = (s.width() - width) / 2;
        int y = (s.height() - height) / 2;
        targetView->setGeometry(x, y, width, height);
    }

    return true;
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
            stopRecording();
        } else {
            while(time >= nextFrameTime){
                saveViewImage();
                ++frame;
                nextFrameTime += timeStep;
            }
        }
    }
    return false;
}


bool MovieRecorderImpl::saveViewImage()
{
    bool result = false;
    string filename = str(filenameFormat % frame);
    
    if(SceneView* sceneView = dynamic_cast<SceneView*>(targetView)){
        result = sceneView->sceneWidget()->saveImage(filename);
    } else {
        QPixmap pixmap(targetView->size());
        targetView->render(&pixmap);
        captureSceneWidgets(targetView, pixmap);
        result = pixmap.save(filename.c_str());
    }
    if(!result){
        showWarningDialog(fmt(_("Saving an image to \"%1%\" failed.")) % filename);
    }

    return result;
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


void MovieRecorderImpl::onPlaybackStopped()
{
    stopRecording();
}


void MovieRecorderImpl::onViewMarkerToggled(bool on)
{
    toolBar->changeViewMarkerToggleState(on);
    config->changeViewMarkerCheckState(on);
    
    isViewMarkerActive = false;
    stopViewMarkerBlinking();
    
    if(on){
        if(!targetView){
            if(!targetViewName.empty()){
                setTargetView(targetViewName);
            }
        }
        if(targetView){
            if(!viewMarker){
                viewMarker = new ViewMarker(this);
            }
            viewMarker->setTargetView(targetView);
            viewMarker->show();
            isViewMarkerActive = true;
            if(isRecording){
                startViewMarkerBlinking();
            }
        }
    } else {
        if(viewMarker){
            viewMarker->hide();
        }
    }
}


void MovieRecorderImpl::startViewMarkerBlinking()
{
    if(isViewMarkerActive && isRecording){
        viewMarkerBlinkTimer.start();
    }
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
    return config->store(archive);
}


bool ConfigDialog::store(Mapping& archive)
{
    archive.write("directory", directoryEntry.string());
    archive.write("basename", basenameEntry.string());
    archive.write("startTime", startTimeSpin.value());
    if(finishTimeCheck.isChecked()){
        archive.write("finishTime", finishTimeSpin.value());
    }
    archive.write("fps", fpsSpin.value());
    archive.write("setSize", imageSizeCheck.isChecked());
    archive.write("width", imageWidthSpin.value());
    archive.write("height", imageHeightSpin.value());
    return true;
}


void MovieRecorderImpl::restore(const Mapping& archive)
{
    string name;
    if(archive.read("target", name)){
        setTargetView(name);
    }
    config->restore(archive);
}


void ConfigDialog::restore(const Mapping& archive)
{
    directoryEntry.setText(archive.get("directory", directoryEntry.string()));
    basenameEntry.setText(archive.get("basename", basenameEntry.string()));
    startTimeSpin.setValue(archive.get("startTime", startTimeSpin.value()));
    double finishTime;
    if(archive.read("finishTime", finishTime)){
        finishTimeSpin.setValue(finishTime);
        finishTimeCheck.setChecked(true);
    } else {
        finishTimeCheck.setChecked(false);
    }
    fpsSpin.setValue(archive.get("fps", fpsSpin.value()));
    imageSizeCheck.setChecked(archive.get("setSize", imageSizeCheck.isChecked()));
    imageWidthSpin.setValue(archive.get("width", imageWidthSpin.value()));
    imageHeightSpin.setValue(archive.get("height", imageHeightSpin.value()));
}
