#include "MovieRecorderDialog.h"
#include "ViewManager.h"
#include "SpinBox.h"
#include "Buttons.h"
#include "ButtonGroup.h"
#include "LineEdit.h"
#include "CheckBox.h"
#include "ComboBox.h"
#include "Dialog.h"
#include "FileDialog.h"
#include "Separator.h"
#include <QDialogButtonBox>
#include <QFileDialog>
#include "gettext.h"

using namespace std;
using namespace cnoid;


MovieRecorderDialog* MovieRecorderDialog::instance()
{
    static MovieRecorderDialog* dialog = new MovieRecorderDialog;
    return dialog;
}


MovieRecorderDialog::MovieRecorderDialog()
    : updateViewComboLater([&](){ updateViewCombo(); })
{
    recorder_ = MovieRecorder::instance();

    setWindowTitle(_("Movie Recorder"));
    
    auto vbox = new QVBoxLayout;
    setLayout(vbox);

    auto hbox = new QHBoxLayout;
    hbox->addWidget(new QLabel(_("Target view")));

    targetViewCombo = new ComboBox(this);
    widgetConnections.add(
        targetViewCombo->sigCurrentIndexChanged().connect(
            [this](int index){ onTargetViewComboIndexChanged(index); }));
    hbox->addWidget(targetViewCombo);

    viewMarkerCheck = new CheckBox(_("Show the marker"), this);
    widgetConnections.add(
        viewMarkerCheck->sigToggled().connect(
            [this](bool on){
                auto block = recorderConfConnection.scopedBlock();
                recorder_->setViewMarkerVisible(on);
            }));
    hbox->addWidget(viewMarkerCheck);
    
    hbox->addStretch();
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout;
    hbox->addWidget(new QLabel(_("Recording mode")));
    auto modeGroup = new ButtonGroup;
    modeRadioButtons.resize(MovieRecorder::NumRecordingModes);
    for(int i=0; i < MovieRecorder::NumRecordingModes; ++i){
        auto radio = new RadioButton(this);
        auto mode = static_cast<MovieRecorder::RecordingMode>(i);
        radio->setText(MovieRecorder::translatedRecordingModeLabel(mode));
        modeGroup->addButton(radio, i);
        hbox->addWidget(radio);
        modeRadioButtons[i] = radio;
    }
    widgetConnections.add(
        modeGroup->sigButtonToggled().connect(
            [this](int mode, bool on){
                if(on){
                    recorder_->setRecordingMode(static_cast<MovieRecorder::RecordingMode>(mode));
                }
            }));
    hbox->addStretch();
    vbox->addLayout(hbox);

    isRecordingModeRadioToolTipEnabled = true;

    modeDescriptions.resize(MovieRecorder::NumRecordingModes);
    setRecordingModeDescription(
        MovieRecorder::OfflineMode,
        _("Record animation of existing playback data in a frame-by-frame manner."));
    setRecordingModeDescription(
        MovieRecorder::OnlineMode,
        _("Record animation of ongoing process in realtime."));
    setRecordingModeDescription(
        MovieRecorder::DirectMode,
        _("Record the display contents of a view directly in realtime."));
    
    hbox = new QHBoxLayout;
    hbox->addWidget(new QLabel(_("Format")));
    encoderCombo = new ComboBox(this);
    widgetConnections.add(
        encoderCombo->sigCurrentIndexChanged().connect(
            [this](int index){
                auto block = recorderConfConnection.scopedBlock();
                int encoderIndex = encoderCombo->currentData().toInt();
                recorder_->setCurrentEncoder(encoderIndex);
            }));
    hbox->addWidget(encoderCombo);
    hbox->addStretch();
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout;
    hbox->addWidget(new QLabel(_("Directory")));
    directoryEntry = new LineEdit(this);
    widgetConnections.add(
        directoryEntry->sigTextChanged().connect(
            [this](const QString& directory){
                auto block = recorderConfConnection.scopedBlock();
                recorder_->setOutputDirectory(directory.toStdString());
            }));
    hbox->addWidget(directoryEntry);

    directoryButton = new PushButton;
    QIcon folderIcon = QIcon::fromTheme("folder");
    if(folderIcon.isNull()){
        directoryButton->setText(_("Select"));
    } else {
        directoryButton->setIcon(folderIcon);
    }
    directoryButton->sigClicked().connect(
        [this](){ showDirectorySelectionDialog(); });
    hbox->addWidget(directoryButton);
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout;
    hbox->addWidget(new QLabel(_("File base name")));
    baseNameEntry = new LineEdit(this);
    widgetConnections.add(
        baseNameEntry->sigTextChanged().connect(
            [this](const QString& baseName){
                auto block = recorderConfConnection.scopedBlock();
                recorder_->setFileBaseName(baseName.toStdString());
            }));
    hbox->addWidget(baseNameEntry);
    hbox->addStretch();
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout;
    hbox->addWidget(new QLabel(_("Frame rate")));
    fpsSpin = new DoubleSpinBox(this);
    fpsSpin->setDecimals(1);
    fpsSpin->setRange(1.0, 9999.9);
    fpsSpin->setSingleStep(0.1);
    widgetConnections.add(
        fpsSpin->sigValueChanged().connect(
            [this](double fps){
                auto block = recorderConfConnection.scopedBlock();
                recorder_->setFrameRate(fps);
            }));
    hbox->addWidget(fpsSpin);
    hbox->addWidget(new QLabel(_("[fps]")));
    hbox->addStretch();
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout;
    startingTimeCheck = new CheckBox(_("Start time"), this);
    widgetConnections.add(
        startingTimeCheck->sigToggled().connect(
            [this](bool on){
                auto block = recorderConfConnection.scopedBlock();
                recorder_->setStartingTimeSpecified(on);
                startingTimeSpin->setEnabled(on);
            }));
    hbox->addWidget(startingTimeCheck);

    startingTimeSpin = new DoubleSpinBox(this);
    startingTimeSpin->setDecimals(2);
    startingTimeSpin->setRange(0.00, 9999.99);
    startingTimeSpin->setSingleStep(0.1);
    widgetConnections.add(
        startingTimeSpin->sigValueChanged().connect(
            [this](double time){
                auto block = recorderConfConnection.scopedBlock();
                recorder_->setStartingTime(time);
            }));
    hbox->addWidget(startingTimeSpin);
    hbox->addWidget(new QLabel(_("[s]")));
    hbox->addSpacing(4);

    finishingTimeCheck = new CheckBox(_("Finish time"), this);
    widgetConnections.add(
        finishingTimeCheck->sigToggled().connect(
            [this](bool on){
                auto block = recorderConfConnection.scopedBlock();
                recorder_->setFinishingTimeSpecified(on);
                finishingTimeSpin->setEnabled(on);
            }));
    hbox->addWidget(finishingTimeCheck);

    finishingTimeSpin = new DoubleSpinBox(this);
    finishingTimeSpin->setDecimals(2);
    finishingTimeSpin->setRange(0.00, 9999.99);
    finishingTimeSpin->setSingleStep(0.1);
    widgetConnections.add(
        finishingTimeSpin->sigValueChanged().connect(
            [this](double time){
                auto block = recorderConfConnection.scopedBlock();
                recorder_->setFinishingTime(time);
            }));
    hbox->addWidget(finishingTimeSpin);
    hbox->addWidget(new QLabel(_("[s]")));
    hbox->addStretch();
    vbox->addLayout(hbox);
    
    hbox = new QHBoxLayout;
    imageSizeCheck = new CheckBox(_("Image size"), this);
    widgetConnections.add(
        imageSizeCheck->sigToggled().connect(
            [this](bool on){
                auto block = recorderConfConnection.scopedBlock();
                recorder_->setImageSizeSpecified(on);
                imageWidthSpin->setEnabled(on);
                imageHeightSpin->setEnabled(on);
            }));
    hbox->addWidget(imageSizeCheck);

    imageWidthSpin = new SpinBox(this);
    imageWidthSpin->setRange(1, 9999);
    imageWidthSpin->setValue(640);
    widgetConnections.add(
        imageWidthSpin->sigValueChanged().connect(
            [this](int width){
                auto block = recorderConfConnection.scopedBlock();
                recorder_->setImageSize(width, imageHeightSpin->value());
            }));
    hbox->addWidget(imageWidthSpin);

    hbox->addWidget(new QLabel("x"));

    imageHeightSpin = new SpinBox(this);
    imageHeightSpin->setRange(1, 9999);
    widgetConnections.add(
        imageHeightSpin->sigValueChanged().connect(
            [this](int height){
                auto block = recorderConfConnection.scopedBlock();
                recorder_->setImageSize(imageWidthSpin->value(), height);
            }));
    hbox->addWidget(imageHeightSpin);
    hbox->addStretch();
    vbox->addLayout(hbox);

    if(MovieRecorder::isMouseCursorCaptureAvailable()){
        hbox = new QHBoxLayout;
        mouseCursorCheck = new CheckBox(_("Capture the mouse cursor"), this);
        widgetConnections.add(
            mouseCursorCheck->sigToggled().connect(
                [this](bool on){
                    auto block = recorderConfConnection.scopedBlock();
                    recorder_->setMouseCursorCaptureEnabled(on);
                }));
        hbox->addWidget(mouseCursorCheck);
        hbox->addStretch();
        vbox->addLayout(hbox);
    }

    vbox->addWidget(new HSeparator);

    hbox = new QHBoxLayout;
    modeDescriptionLabel = new QLabel(this);
    modeDescriptionLabel->hide();
    hbox->addWidget(modeDescriptionLabel, 1);
    
    auto buttonBox = new QDialogButtonBox(this);
    recordingToggle = new ToggleButton(_("&Record"), this);
    recordingToggle->setDefault(true);
    recordingToggle->sigToggled().connect(
        [this](bool on){ onRecordingButtonToggled(on); });
    buttonBox->addButton(recordingToggle, QDialogButtonBox::ActionRole);
    hbox->addWidget(buttonBox);

    vbox->addLayout(hbox);
}


void MovieRecorderDialog::setTargetViewFilter(std::function<bool(View* view)> filter)
{
    targetViewFilter = filter;
}


void MovieRecorderDialog::setRecordingModeRadioVisible(MovieRecorder::RecordingMode mode, bool on)
{
    auto radio = modeRadioButtons[mode];
    radio->setVisible(on);

    if(radio->isChecked() && !on){
        for(int i=0; i < 3; ++i){
            radio = modeRadioButtons[i];
            if(radio->isVisible()){
                radio->setChecked(true);
                break;
            }
        }
    }
}


void MovieRecorderDialog::setRecordingModeDescription(MovieRecorder::RecordingMode mode, const std::string& description)
{
    modeDescriptions[mode] = description;
    if(isRecordingModeRadioToolTipEnabled){
        modeRadioButtons[mode]->setToolTip(description.c_str());
    } else {
        modeRadioButtons[mode]->setToolTip("");
    }
}


void MovieRecorderDialog::setRecordingModeRadioToolTipEnabled(bool on)
{
    if(on != isRecordingModeRadioToolTipEnabled){
        isRecordingModeRadioToolTipEnabled = on;
        for(int i=0; i < MovieRecorder::NumRecordingModes; ++i){
            if(isRecordingModeRadioToolTipEnabled){
                modeRadioButtons[i]->setToolTip(modeDescriptions[i].c_str());
            } else {
                modeRadioButtons[i]->setToolTip("");
            }
        }
    }
}


void MovieRecorderDialog::setRecordingModeDescriptionVisible(bool on)
{
    modeDescriptionLabel->setVisible(on);
}


void MovieRecorderDialog::setSelectableEncoderFilter(std::function<bool(MovieRecorderEncoder* encoder)> filter)
{
    selectableEncoderFilter = filter;
}


void MovieRecorderDialog::showEvent(QShowEvent* event)
{
    updateViewCombo();

    recorderConfConnection =
        recorder_->sigRecordingConfigurationChanged().connect(
            [this](){ updateWidgetsWithRecorderConfigurations(); });
    updateWidgetsWithRecorderConfigurations();
    
    recorderStateConnection =
        recorder_->sigRecordingStateChanged().connect(
            [this](bool on){ onRecordingStateChanged(on); });
    onRecordingStateChanged(recorder_->isRecording());
    
    viewManagerConnections.disconnect();
    viewManagerConnections.add(
        ViewManager::sigViewActivated().connect(
            [&](View*){ updateViewComboLater(); }));
    viewManagerConnections.add(
        ViewManager::sigViewDeactivated().connect(
            [&](View*){ updateViewComboLater(); }));

    Dialog::showEvent(event);
}


void MovieRecorderDialog::hideEvent(QHideEvent* event)
{
    recorderConfConnection.disconnect();
    recorderStateConnection.disconnect();
    viewManagerConnections.disconnect();
    updateViewComboLater.cancel();
    Dialog::hideEvent(event);
}


void MovieRecorderDialog::updateWidgetsWithRecorderConfigurations()
{
    auto scopedBlock = widgetConnections.scopedBlock();

    updateViewCombo();

    viewMarkerCheck->setChecked(recorder_->isViewMarkerVisible());

    int recordingMode = recorder_->recordingMode();
    modeRadioButtons[recordingMode]->setChecked(true);
    modeDescriptionLabel->setText(modeDescriptions[recordingMode].c_str());

    encoderCombo->clear();
    int currentEncoderIndex = recorder_->currentEncoderIndex();
    int encoderIndex = 0;
    int validEncoderIndex = -1;
    bool encoderComboIndexUpdated = false;
    for(auto& encoder : recorder_->encoders()){
        if(!selectableEncoderFilter || selectableEncoderFilter(encoder)){
            if(validEncoderIndex < 0){
                validEncoderIndex = encoderIndex;
            }
            encoderCombo->addItem(encoder->formatName().c_str(), encoderIndex);
            if(encoderIndex == currentEncoderIndex){
                encoderCombo->setCurrentIndex(encoderCombo->count() - 1);
                encoderComboIndexUpdated = true;
            }
        }
        ++encoderIndex;
    }
    if(!encoderComboIndexUpdated){
        auto block = recorderConfConnection.scopedBlock();
        recorder_->setCurrentEncoder(validEncoderIndex);
    }

    directoryEntry->blockSignals(true);
    directoryEntry->setText(recorder_->outputDirectory().c_str());
    directoryEntry->blockSignals(false);

    baseNameEntry->blockSignals(true);
    baseNameEntry->setText(recorder_->fileBaseName().c_str());
    baseNameEntry->blockSignals(false);

    fpsSpin->setValue(recorder_->frameRate());

    bool isStartingTimeSpecified = recorder_->isStartingTimeSpecified(); 
    startingTimeCheck->setChecked(isStartingTimeSpecified);
    startingTimeSpin->setEnabled(isStartingTimeSpecified);
    startingTimeSpin->setValue(recorder_->startingTime());

    bool isFinishingTimeSpecified = recorder_->isFinishingTimeSpecified(); 
    finishingTimeCheck->setChecked(isFinishingTimeSpecified);
    finishingTimeSpin->setEnabled(isFinishingTimeSpecified);
    finishingTimeSpin->setValue(recorder_->finishingTime());

    bool isImageSizeSpecified = recorder_->isImageSizeSpecified();
    imageSizeCheck->setChecked(isImageSizeSpecified);
    imageWidthSpin->setEnabled(isImageSizeSpecified);
    imageWidthSpin->setValue(recorder_->imageWidth());
    imageHeightSpin->setEnabled(isImageSizeSpecified);
    imageHeightSpin->setValue(recorder_->imageHeight());

    if(MovieRecorder::isMouseCursorCaptureAvailable()){
        mouseCursorCheck->setChecked(recorder_->isMouseCursorCaptureEnabled());
    }
}


void MovieRecorderDialog::updateViewCombo()
{
    targetViewCombo->blockSignals(true);

    targetViewCombo->clear();
    targetViewCombo->addItem(_("None"));

    viewCandidates.clear();
    auto targetView = recorder_->targetView();
    bool selected = false;
    for(auto& view : ViewManager::activeViews()){
        if(targetViewFilter && !targetViewFilter(view)){
            continue;
        }
        viewCandidates.push_back(view);
        targetViewCombo->addItem(view->windowTitle());
        if(!selected && targetView){
            if(view == targetView || view->name() == targetView->name()){
                targetViewCombo->setCurrentIndex(targetViewCombo->count() - 1);
                selected = true;
            }
        }
    }

    if(!selected){
        onTargetViewComboIndexChanged(0);
    }

    targetViewCombo->blockSignals(false);
}


void MovieRecorderDialog::onTargetViewComboIndexChanged(int index)
{
    auto block = recorderConfConnection.scopedBlock();
    if(index == 0){
        recorder_->setTargetView(nullptr, true);
    } else {
        size_t viewIndex = index - 1;
        if(viewIndex < viewCandidates.size()){
            recorder_->setTargetView(viewCandidates[viewIndex], true);
        }
    }
}


void MovieRecorderDialog::onRecordingStateChanged(bool on)
{
    recordingToggle->blockSignals(true);
    recordingToggle->setChecked(on);

    /*
      In the offline mode, Qt events are processed by QCoreApplication:processEvents called in
      the MoveRecorder::Impl::startOfflineModeRecording function. In this case, if the recording toggle
      is toggled, the sigToggled signal is recursively emitted, and the corresponding slot is called
      after the recording is finished. This means a user cannot stop the recording immediately by toggling
      the recording toggle manually. To avoid confusion, the recording toggle button should be disabled
      in this case.
    */
    recordingToggle->setDisabled(on && recorder_->recordingMode() == MovieRecorder::OfflineMode);
    
    recordingToggle->blockSignals(false);
}


void MovieRecorderDialog::onRecordingButtonToggled(bool on)
{
    if(on){
        if(!recorder_->startRecording()){
            onRecordingStateChanged(false);
        }
    } else {
        recorder_->stopRecording();
    }
}


void MovieRecorderDialog::showDirectorySelectionDialog()
{
    FileDialog dialog;
    dialog.setWindowTitle(_("Select Directory"));
    dialog.setViewMode(QFileDialog::List);
    dialog.setFileMode(QFileDialog::Directory);
    dialog.setOption(QFileDialog::ShowDirsOnly);
    dialog.updatePresetDirectories();
    dialog.setDirectory(directoryEntry->text());

    if(dialog.exec()){
        directoryEntry->setText(dialog.selectedFiles().at(0));
    }
}
