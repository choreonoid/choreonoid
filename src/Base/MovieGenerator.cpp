/**
   @author Shin'ichiro Nakaoka
*/

#include "MovieGenerator.h"
#include <cnoid/AppConfig>
#include <cnoid/TimeBar>
#include <cnoid/ConnectionSet>
#include <cnoid/SceneWidget>
#include <cnoid/SceneView>
#include <cnoid/ExtensionManager>
#include <cnoid/MenuManager>
#include <cnoid/Archive>
#include <cnoid/MainWindow>
#include <cnoid/SpinBox>
#include <cnoid/Button>
#include <cnoid/LineEdit>
#include <cnoid/MessageView>
#include <cnoid/Dialog>
#include <cnoid/Separator>
#include <QLabel>
#include <QDialogButtonBox>
#include <QFileDialog>
#include <QPainter>
#include <boost/filesystem.hpp>
#include <boost/bind.hpp>
#include "gettext.h"

using namespace std;
using namespace cnoid;
namespace filesystem = boost::filesystem;
using boost::format;


namespace {
    
class MovieGenerator : public Dialog
{
public:
    boost::signals::connection focusViewChangedConnection;

    TimeBar* timeBar;
    View* targetView;
    bool isRecording;
    bool isBeforeFirstFrameCapture;
    bool requestStopRecording;
    int frame;
    double nextFrameTime;
    double timeStep;
    double endingTime;
    format filenameFormat;
    QLabel targetViewLabel;
    LineEdit directoryEntry;
    PushButton directoryButton;
    LineEdit basenameEntry;
    DoubleSpinBox beginningTimeSpin;
    CheckBox endingTimeCheck;
    DoubleSpinBox endingTimeSpin;
    DoubleSpinBox fpsSpin;
    CheckBox imageSizeCheck;
    SpinBox imageWidthSpin;
    SpinBox imageHeightSpin;

    ConnectionSet timeBarConnections;

    MovieGenerator();
    ~MovieGenerator();
    bool store(Mapping& archive);
    void restore(const Mapping& archive);
    virtual void showEvent(QShowEvent* event);
    virtual void hideEvent(QHideEvent* event);
    void onFocusViewChanged(View* view);
    void showDirectorySelectionDialog();
    void generate();
    void onStopRequest();
    bool setupViewAndFilenameFormat();
    bool doRecordingLoop();
    bool saveViewImage();
    void captureSceneWidgets(QWidget* widget, QPixmap& pixmap);
    void capture();
    void onPlaybackStarted(double time);
    bool onTimeChanged(double time);
    void onPlaybackStopped();
    void stopRecording();
};
}


void cnoid::initializeMovieGenerator(ExtensionManager* ext)
{
    static MovieGenerator* movieGenerator = 0;

    if(!movieGenerator){
        movieGenerator = ext->manage(new MovieGenerator());

        MenuManager& mm = ext->menuManager();
        mm.setPath("/Tools");
        mm.addItem(N_("Movie Generator"))
            ->sigTriggered().connect(boost::bind(&MovieGenerator::show, movieGenerator));
    }
}


MovieGenerator::MovieGenerator()
{
    setWindowTitle(_("Movie Generator"));

    timeBar = TimeBar::instance();

    isRecording = false;
    isBeforeFirstFrameCapture = false;
    requestStopRecording = false;

    QVBoxLayout* vbox = new QVBoxLayout();
    setLayout(vbox);

    QHBoxLayout* hbox = new QHBoxLayout();
    hbox->addWidget(new QLabel(_("Target view:")));
    targetViewLabel.setText(_("Not specified"));
    hbox->addWidget(&targetViewLabel);
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
    directoryButton.sigClicked().connect(boost::bind(&MovieGenerator::showDirectorySelectionDialog, this));
    hbox->addWidget(&directoryButton);

    hbox->addWidget(new QLabel(_("Basename")));
    basenameEntry.setText("scene");
    hbox->addWidget(&basenameEntry);
    vbox->addStretch();
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout();
    hbox->addWidget(new QLabel(_("Begin")));
    beginningTimeSpin.setDecimals(2);
    beginningTimeSpin.setRange(0.00, 9999.99);
    beginningTimeSpin.setSingleStep(0.1);
    hbox->addWidget(&beginningTimeSpin);

    endingTimeCheck.setText(_("End"));
    hbox->addWidget(&endingTimeCheck);
    
    endingTimeSpin.setDecimals(2);
    endingTimeSpin.setRange(0.00, 9999.99);
    endingTimeSpin.setSingleStep(0.1);
    hbox->addWidget(&endingTimeSpin);

    hbox->addWidget(new QLabel("FPS"));
    fpsSpin.setDecimals(1);
    fpsSpin.setRange(1.0, 9999.9);
    fpsSpin.setValue(30.0);
    fpsSpin.setSingleStep(0.1);
    hbox->addWidget(&fpsSpin);
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

    vbox->addWidget(new HSeparator);
    QDialogButtonBox* buttonBox = new QDialogButtonBox(this);
    vbox->addWidget(buttonBox);

    PushButton* applyButton = new PushButton(_("&Generate"));
    applyButton->setDefault(true);
    buttonBox->addButton(applyButton, QDialogButtonBox::ActionRole);
    applyButton->sigClicked().connect(boost::bind(&MovieGenerator::generate, this));
    hbox->addWidget(applyButton);

    PushButton* captureButton = new PushButton(_("&Capture"));
    buttonBox->addButton(captureButton, QDialogButtonBox::ActionRole);
    captureButton->sigClicked().connect(boost::bind(&MovieGenerator::capture, this));
    hbox->addWidget(captureButton);

    PushButton* stopButton = new PushButton(_("&Stop"));
    buttonBox->addButton(stopButton, QDialogButtonBox::ActionRole);
    stopButton->sigClicked().connect(boost::bind(&MovieGenerator::onStopRequest, this));

    Mapping& config = *AppConfig::archive()->findMapping("MovieGenerator");
    if(config.isValid()){
        restore(config);
    }
}


MovieGenerator::~MovieGenerator()
{
    timeBarConnections.disconnect();

    store(*AppConfig::archive()->openMapping("MovieGenerator"));
}


bool MovieGenerator::store(Mapping& archive)
{
    archive.write("directory", directoryEntry.string());
    archive.write("basename", basenameEntry.string());
    archive.write("begin", beginningTimeSpin.value());
    if(endingTimeCheck.isChecked()){
        archive.write("end", endingTimeSpin.value());
    }
    archive.write("fps", fpsSpin.value());
    archive.write("setSize", imageSizeCheck.isChecked());
    archive.write("width", imageWidthSpin.value());
    archive.write("heiht", imageHeightSpin.value());
    return true;
}


void MovieGenerator::restore(const Mapping& archive)
{
    directoryEntry.setText(archive.get("directory", directoryEntry.string()));
    basenameEntry.setText(archive.get("basename", basenameEntry.string()));
    beginningTimeSpin.setValue(archive.get("begin", beginningTimeSpin.value()));
    double endingTime;
    if(archive.read("end", endingTime)){
        endingTimeSpin.setValue(endingTime);
        endingTimeCheck.setChecked(true);
    } else {
        endingTimeCheck.setChecked(false);
    }
    fpsSpin.setValue(archive.get("fps", fpsSpin.value()));
    imageSizeCheck.setChecked(archive.get("setSize", imageSizeCheck.isChecked()));
    imageWidthSpin.setValue(archive.get("width", imageWidthSpin.value()));
    imageHeightSpin.setValue(archive.get("height", imageHeightSpin.value()));
}


void MovieGenerator::showEvent(QShowEvent* event)
{
    if(!focusViewChangedConnection.connected()){
        focusViewChangedConnection =
            View::sigFocusChanged().connect(
                boost::bind(&MovieGenerator::onFocusViewChanged, this, _1));
    }
    if(!isRecording){
        targetView = View::lastFocusView();
    }
}


void MovieGenerator::hideEvent(QHideEvent* event)
{
    focusViewChangedConnection.disconnect();
}


void MovieGenerator::onFocusViewChanged(View* view)
{
    if(!isRecording){
        if(view){
            targetViewLabel.setText(view->windowTitle());
        } else {
            targetViewLabel.setText(_("Not specified"));
        }
        targetView = view;
    }
}

void MovieGenerator::showDirectorySelectionDialog()
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


void MovieGenerator::generate()
{
    if(!isRecording){
        if(doRecordingLoop()){
            hide();
        }
    }
}


void MovieGenerator::onStopRequest()
{
    if(isRecording){
        requestStopRecording = true;
    }
}


bool MovieGenerator::setupViewAndFilenameFormat()
{
    if(!targetView){
        return false;
    }

    filesystem::path directory(directoryEntry.string());
    filesystem::path basename(basenameEntry.string() + "%08u.png");

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

    targetView->resize(imageWidthSpin.value(), imageHeightSpin.value());

    return true;
}


bool MovieGenerator::doRecordingLoop()
{
    if(!setupViewAndFilenameFormat()){
        return false;
    }
    
    requestStopRecording = false;
    frame = 0;
    
    double time = beginningTimeSpin.value();
    endingTime = endingTimeCheck.isChecked() ? endingTimeSpin.value() : std::numeric_limits<double>::max();
    timeStep = 1.0 / fpsSpin.value();
    bool doContinue = true;

    isRecording = true;

    while(time < endingTime && doContinue){

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

    isRecording = false;

    return !requestStopRecording;
}


bool MovieGenerator::saveViewImage()
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


void MovieGenerator::captureSceneWidgets(QWidget* widget, QPixmap& pixmap)
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


void MovieGenerator::capture()
{
    if(!isRecording){
        timeBarConnections.disconnect();
        if(setupViewAndFilenameFormat()){
            isBeforeFirstFrameCapture = true;
            frame = 0;
            endingTime = endingTimeCheck.isChecked() ? endingTimeSpin.value() : std::numeric_limits<double>::max();
            timeStep = 1.0 / fpsSpin.value();
            requestStopRecording = false;
            
            const bool isDoingPlayback = timeBar->isDoingPlayback();
            if(isDoingPlayback){
                isRecording = true;
            } else {
                timeBarConnections.add(
                    timeBar->sigPlaybackStarted().connect(
                        boost::bind(&MovieGenerator::onPlaybackStarted, this, _1)));
            }

            timeBarConnections.add(
                timeBar->sigTimeChanged().connect(
                    boost::bind(&MovieGenerator::onTimeChanged, this, _1)));
            timeBarConnections.add(
                timeBar->sigPlaybackStopped().connect(
                    boost::bind(&MovieGenerator::onPlaybackStopped, this)));
        }
    }
}


void MovieGenerator::onPlaybackStarted(double time)
{
    isRecording = true;
}


bool MovieGenerator::onTimeChanged(double time)
{
    if(isRecording){
        if(isBeforeFirstFrameCapture){
            if(time > beginningTimeSpin.value()){
                nextFrameTime = time;
            } else {
                nextFrameTime = beginningTimeSpin.value();
            }
        }
        if(requestStopRecording || time > endingTime){
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


void MovieGenerator::onPlaybackStopped()
{
    stopRecording();
}


void MovieGenerator::stopRecording()
{
    isRecording = false;
    timeBarConnections.disconnect();
}
