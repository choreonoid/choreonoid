/**
   @author Shin'ichiro Nakaoka
*/

#include "MovieRecorder.h"
#include "ExtensionManager.h"
#include "MessageView.h"
#include "MainWindow.h"
#include "ToolBar.h"
#include "TimeBar.h"
#include "Archive.h"
#include "SpinBox.h"
#include "Buttons.h"
#include "LineEdit.h"
#include "CheckBox.h"
#include "Dialog.h"
#include "Separator.h"
#include <cnoid/ConnectionSet>
#include <QDialogButtonBox>
#include <QFileDialog>
#include <boost/filesystem.hpp>
#include <boost/bind.hpp>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

MovieRecorder* movieRecorder = 0;

class MovieRecorderBar : public ToolBar
{
public:
    MovieRecorderImpl* recorder;
    MovieRecorderBar(MovieRecorderImpl* recorder);
};

class SetupDialog : public Dialog
{
public:
    MovieRecorderImpl* recorder;
    Connection focusViewChangedConnection;

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

    SetupDialog(MovieRecorderImpl* recorder);
    ~SetupDialog();
    virtual void showEvent(QShowEvent* event);
    virtual void hideEvent(QHideEvent* event);
    void onFocusViewChanged(View* view);    
    void showDirectorySelectionDialog();
};

}

namespace cnoid {

class MovieRecorderImpl
{
public:
    MovieRecorderBar* toolBar;
    TimeBar* timeBar;
    View* targetView;
    bool isRecording;
    bool isBeforeFirstFrameCapture;
    bool requestStopRecording;
    int frame;
    double nextFrameTime;
    double timeStep;
    double endingTime;
    boost::format filenameFormat;
    ConnectionSet timeBarConnections;
    SetupDialog* setup;

    MovieRecorderImpl(ExtensionManager* ext);

    bool store(Mapping& archive);
    void restore(const Mapping& archive);
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


void MovieRecorder::initialize(ExtensionManager* ext)
{
    if(!movieRecorder){
        movieRecorder = new MovieRecorder(ext);
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
    setup = new SetupDialog(this);
    toolBar = new MovieRecorderBar(this);
    ext->addToolBar(toolBar);
}


MovieRecorderBar::MovieRecorderBar(MovieRecorderImpl* recorder)
    : ToolBar(N_("MovieRecorderBar")),
      recorder(recorder)
{
    addButton("P", _("Start Playback & Recording"));
    addButton("R", _("Start Recording"));

    addButton(QIcon(":/Base/icons/setup.png"), _("Open the setup dialog"))
        ->sigClicked().connect(boost::bind(&QDialog::show, recorder->setup));
}


SetupDialog::SetupDialog(MovieRecorderImpl* recorder)
{
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
    directoryButton.sigClicked().connect(boost::bind(&SetupDialog::showDirectorySelectionDialog, this));
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

    vbox->addStretch();

    vbox->addWidget(new HSeparator());
    QPushButton* okButton = new QPushButton(_("&Ok"));
    okButton->setDefault(true);
    QDialogButtonBox* buttonBox = new QDialogButtonBox(this);
    buttonBox->addButton(okButton, QDialogButtonBox::AcceptRole);
    connect(buttonBox,SIGNAL(accepted()), this, SLOT(accept()));
    vbox->addWidget(buttonBox);
}


SetupDialog::~SetupDialog()
{

}


void SetupDialog::showEvent(QShowEvent* event)
{
    if(!focusViewChangedConnection.connected()){
        focusViewChangedConnection =
            View::sigFocusChanged().connect(
                boost::bind(&SetupDialog::onFocusViewChanged, this, _1));
    }
    if(!recorder->isRecording){
        recorder->targetView = View::lastFocusView();
    }
}


void SetupDialog::hideEvent(QHideEvent* event)
{
    focusViewChangedConnection.disconnect();
}


void SetupDialog::onFocusViewChanged(View* view)
{
    if(!recorder->isRecording){
        if(view){
            targetViewLabel.setText(view->windowTitle());
        } else {
            targetViewLabel.setText(_("Not specified"));
        }
        recorder->targetView = view;
    }
}


void SetupDialog::showDirectorySelectionDialog()
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
