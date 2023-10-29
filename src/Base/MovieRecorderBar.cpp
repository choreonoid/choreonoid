#include "MovieRecorderBar.h"
#include "MovieRecorder.h"
#include "MovieRecorderDialog.h"
#include "ExtensionManager.h"
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace cnoid {

MovieRecorderBar* MovieRecorderBar::instance_ = nullptr;

}


void MovieRecorderBar::initializeClass(ExtensionManager* ext)
{
    instance_ = new MovieRecorderBar;
    ext->addToolBar(instance_);
}


MovieRecorderBar::MovieRecorderBar()
    : ToolBar(N_("MovieRecorderBar")),
      recordIcon(":/Base/icon/record.svg"),
      recordBlinkIcon(":/Base/icon/record2.svg")
{
    recorder = nullptr;
    
    recordingToggle = addToggleButton(recordIcon);
    recordingToggle->setToolTip(_("Toggle Recording"));

    /**
       Use QObject::connect function directly instead of sigToggled so that toggle off
       to stop recording can be processed during QCoreApplication::processEvents
       called inside the slot function.
    */
    QObject::connect(recordingToggle, &QPushButton::toggled,
                     [this](bool on){ onRecordingButtonToggled(on); });
    
    viewMarkerToggle = addToggleButton(QIcon(":/Base/icon/recordtarget.svg"));
    viewMarkerToggle->setToolTip(_("Toggle Target View Marker"));
    viewMarkerToggle->sigToggled().connect(
        [this](bool on){ onViewMarkerButtonToggled(on); });
    
    auto configButton = addButton(QIcon(":/Base/icon/setup.svg"));
    configButton->setToolTip(_("Show the config dialog"));
    configButton->sigClicked().connect(
        [this](){ showMovieRecorderDialog(); });
}


void MovieRecorderBar::showEvent(QShowEvent* event)
{
    ToolBar::showEvent(event);
    
    if(!recorder){
        recorder = MovieRecorder::instance();

        recorderConnections.add(
            recorder->sigRecordingStateChanged().connect(
                [this](bool on){ onRecordingStateChanged(on); }));
        onRecordingStateChanged(recorder->isRecording());
        
        recorderConnections.add(
            recorder->sigRecordingConfigurationChanged().connect(
                [this](){ onRecordingConfigurationChanged(); }));
        onRecordingConfigurationChanged();

        recorderConnections.add(
            recorder->sigBlinking().connect(
                [this](bool isBlinked){
                    recordingToggle->setIcon(isBlinked ? recordBlinkIcon : recordIcon);
                }));
    }
}


void MovieRecorderBar::onRecordingStateChanged(bool on)
{
    recordingToggle->blockSignals(true);
    recordingToggle->setChecked(on);
    recordingToggle->blockSignals(false);
}


void MovieRecorderBar::onRecordingConfigurationChanged()
{
    viewMarkerToggle->blockSignals(true);
    viewMarkerToggle->setChecked(recorder->isViewMarkerVisible());
    viewMarkerToggle->blockSignals(false);
}


void MovieRecorderBar::onRecordingButtonToggled(bool on)
{
    auto recorder = MovieRecorder::instance();
    if(on){
        recorder->startRecording();
    } else {
        recorder->stopRecording();
    }
}


void MovieRecorderBar::onViewMarkerButtonToggled(bool on)
{
    auto scopedBlock = recorderConnections.scopedBlock();
    MovieRecorder::instance()->setViewMarkerVisible(on);
}


void MovieRecorderBar::showMovieRecorderDialog()
{
    MovieRecorderDialog::instance()->show();
}
