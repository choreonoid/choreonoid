#ifndef CNOID_BASE_MOVIE_RECORDER_DIALOG_H
#define CNOID_BASE_MOVIE_RECORDER_DIALOG_H

#include "LazyCaller.h"
#include <cnoid/Dialog>
#include <cnoid/MovieRecorder>
#include <cnoid/ConnectionSet>
#include <QLabel>
#include <vector>
#include <string>
#include "exportdecl.h"

namespace cnoid {

class View;
class ComboBox;
class CheckBox;
class LineEdit;
class PushButton;
class ToggleButton;
class RadioButton;
class SpinBox;
class DoubleSpinBox;

class CNOID_EXPORT MovieRecorderDialog : public Dialog
{
public:
    static MovieRecorderDialog* instance();

    MovieRecorder* recorder(){ return recorder_; }
    void setTargetViewFilter(std::function<bool(View* view)> filter);
    void setRecordingModeRadioVisible(MovieRecorder::RecordingMode mode, bool on);
    void setRecordingModeDescription(MovieRecorder::RecordingMode mode, const std::string& description);
    const std::string& recordingModeDescription(MovieRecorder::RecordingMode mode){
        return modeDescriptions[mode];
    }
    void setRecordingModeRadioToolTipEnabled(bool on);
    void setRecordingModeDescriptionVisible(bool on);
    
    void setSelectableEncoderFilter(std::function<bool(MovieRecorderEncoder* encoder)> filter);

protected:
    MovieRecorderDialog();
    virtual void showEvent(QShowEvent* event) override;
    virtual void hideEvent(QHideEvent* event) override;

private:
    void updateWidgetsWithRecorderConfigurations();
    void updateViewCombo();
    void onTargetViewComboIndexChanged(int index);
    void onRecordingStateChanged(bool on);
    void onRecordingButtonToggled(bool on);
    void showDirectorySelectionDialog();

    MovieRecorder* recorder_;
    ScopedConnection recorderStateConnection;
    ScopedConnection recorderConfConnection;
    ScopedConnectionSet widgetConnections;
    ScopedConnectionSet viewManagerConnections;
    LazyCaller updateViewComboLater;
    std::vector<View*> viewCandidates;
    ComboBox* targetViewCombo;
    CheckBox* viewMarkerCheck;
    std::function<bool(View* view)> targetViewFilter;
    std::vector<RadioButton*> modeRadioButtons;
    QLabel* modeDescriptionLabel;
    bool isRecordingModeRadioToolTipEnabled;
    std::vector<std::string> modeDescriptions;
    ComboBox* encoderCombo;
    std::function<bool(MovieRecorderEncoder* encoder)> selectableEncoderFilter;
    LineEdit* directoryEntry;
    PushButton* directoryButton;
    LineEdit* baseNameEntry;
    CheckBox* startingTimeCheck;
    DoubleSpinBox* startingTimeSpin;
    CheckBox* finishingTimeCheck;
    DoubleSpinBox* finishingTimeSpin;
    DoubleSpinBox* fpsSpin;
    CheckBox* imageSizeCheck;
    SpinBox* imageWidthSpin;
    SpinBox* imageHeightSpin;
    CheckBox* mouseCursorCheck;
    ToggleButton* recordingToggle;
};

}

#endif
