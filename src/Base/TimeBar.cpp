#include "TimeBar.h"
#include "ExtensionManager.h"
#include "Archive.h"
#include "OptionManager.h"
#include "QtSvgUtil.h"
#include "SpinBox.h"
#include "Slider.h"
#include "Buttons.h"
#include "CheckBox.h"
#include "Dialog.h"
#include "Separator.h"
#include <QDialogButtonBox>
#include <QGridLayout>
#include <QElapsedTimer>
#include <cmath>
#include <limits>
#include <iostream>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

const bool TRACE_FUNCTIONS = false;

const double DEFAULT_FRAME_RATE = 1000.0;

// The following value shoud be same as the display refresh rate to make the animation smooth
const double DEFAULT_PLAYBACK_FRAMERATE = 60.0;

bool isNegativeTimeEnabled = true;

enum ElementId {
    PlayButton = 0,
    ResumeButton = 1,
    RefreshButton = 2,
    TimeSpin = 3,
    TimeSlider = 4,
    TimeRangeMinSpin = 5,
    TimeRangeMaxSpin = 6,
    ConfigButton = 7,

    // The following elements were added later and assigned new IDs,
    // so the IDs do not match the order in which they appear on the toolbar.
    DayTimeSpin = 8,
    DayTimeLabel = 9,
    HourTimeSpin = 10,
    HourTimeLabel = 11,
    MinuteTimeSpin = 12,
    MinuteTimeLabel = 13,
    SecondTimeLabel = 14,
    MaxTimeUnitLabel = 15,
};

enum TimeUnit {
    UnitSecond  = 1,
    UnitMinute  = 2,
    UnitHour    = 4,
    UnitDay     = 8
};

}

namespace cnoid {

class TimeBar::Impl : public QObject
{
public:
    Impl(TimeBar* self);
    ~Impl();

    double quantizedTime(double time) const;
    bool setTime(double time, int options = Truncate, bool calledFromPlaybackLoop = false, QWidget* callerWidget = nullptr);
    bool setTimeBarTime(double time, bool doAutoExpansion = false, bool calledFromPlaybackLoop = false, QWidget* callerWidget = nullptr);
    bool setTimeRange(double minTime, double maxTime, bool doUpdateTimeSpinSlider);
    void onMinTimeSpinValueChanged(double minTime);
    void onMaxTimeSpinValueChanged(double maxTime);
    bool setFrameRate(double rate, bool isSpinBoxSignal, bool doUpdateTimeSpinSlider);
    void updateTimeSpinSlider();
    void setPlaybackFrameRate(double rate, bool isSpinBoxSignal);
    void setIdleEventDrivenMode(bool on, bool isCheckBoxSignal);
    void setPlaybackSpeedRatio(double ratio, bool isSpinBoxSignal);
    void setAutoExpansionMode(bool on, bool isCheckBoxSignal);
    void setOngoingTimeSyncEnabled(bool on, bool isCheckBoxSignal);
    void onPlayActivated();
    void onResumeActivated();
    void startPlayback();
    void startPlayback(double time);
    double stopPlayback(bool isStoppedManually);
    virtual void timerEvent(QTimerEvent* event) override;
    int startOngoingTimeUpdate(double time);
    void updateOngoingTime(int id, double time);
    void updateMinOngoingTime();
    void stopOngoingTimeUpdate(int id);
    bool storeState(Archive& archive);
    bool restoreState(const Archive& archive);

    void onTimeUnitSpinValueChanged(QWidget* callerSpin);
    void updateTimeUnitDisplay();
    void updateTimeUnitChecks();
    void decomposeTime(double totalSeconds, double& days, double& hours, double& minutes, double& seconds);
    double composeTimeFromSpins();

    TimeBar* self;

    ToolButton* resumeButton;
    ToolButton* frameModeToggle;
    QIcon resumeIcon;
    QIcon stopIcon;
    DoubleSpinBox* dayTimeSpin;
    QLabel* dayTimeLabel;
    DoubleSpinBox* hourTimeSpin;
    QLabel* hourTimeLabel;
    DoubleSpinBox* minuteTimeSpin;
    QLabel* minuteTimeLabel;
    DoubleSpinBox* timeSpin;
    QLabel* secondTimeLabel;
    Slider* timeSlider;
    DoubleSpinBox* minTimeSpin;
    DoubleSpinBox* maxTimeSpin;
    QLabel* maxTimeUnitLabel;
    QLabel* timeRangeDelimiterLabel;
    int enabledTimeUnits;
    int decimals;
    double minTime;
    double maxTime;
    double playbackFrameRate;
    double playbackSpeedRatio;
    double animationTimeOffset;
    double ongoingTime;
    bool isIdleEventDrivenMode;
    bool isRepeatMode;
    bool isAutoExpansionMode;
    bool isDoingPlayback;
    bool isPlaybackProcessing;
    bool hasOngoingTime;
    bool isOngoingTimeSyncEnabled;
    map<int, double> ongoingTimeMap;
    int timerId;
    QElapsedTimer elapsedTimer;

    Signal<bool(double time)> sigPlaybackInitialized;
    Signal<void(double time)> sigPlaybackStarted;
    Signal<bool(double time)> sigTimeChanged;
    vector<bool> playbackContinueFlags;
    Signal<void(double time, bool isStoppedManually)> sigPlaybackStopped;
    Signal<double(double time, bool isStoppedManually)> sigPlaybackStoppedEx;
    vector<double> lastValidTimesWhenStopped;

    class ConfigDialog : public Dialog
    {
    public:
        DoubleSpinBox frameRateSpin;
        DoubleSpinBox playbackFrameRateSpin;
        CheckBox idleEventDrivenModeCheck;
        DoubleSpinBox playbackSpeedRatioSpin;
        CheckBox ongoingTimeSyncCheck;
        CheckBox autoExpansionCheck;
        CheckBox dayCheck;
        CheckBox hourCheck;
        CheckBox minuteCheck;
        CheckBox secondCheck;

        ConfigDialog(TimeBar* timeBar);
    };

    ConfigDialog* configDialog;
};

}


static void onSigOptionsParsed(OptionManager* om)
{
    if(om->count("--start-playback")){
        TimeBar::instance()->startPlayback();
    }
}


void TimeBar::setNegativeTimeEnabled(bool on)
{
    isNegativeTimeEnabled = on;
}


void TimeBar::initialize(ExtensionManager* ext)
{
    static bool initialized = false;
    if(!initialized){
        ext->addToolBar(TimeBar::instance());

        auto om = OptionManager::instance();
        om->add_flag("--start-playback", "start playback automatically");
        om->sigOptionsParsed(1).connect(onSigOptionsParsed);
            
        initialized = true;
    }
}


TimeBar* TimeBar::instance()
{
    static TimeBar* timeBar = new TimeBar;
    return timeBar;
}


TimeBar::TimeBar()
    : ToolBar(N_("TimeBar"))
{
    impl = new Impl(this);

    //! \todo Make the creation of the config dialog on-demand
    impl->configDialog = new Impl::ConfigDialog(this);

    impl->updateTimeSpinSlider();
}


TimeBar::Impl::Impl(TimeBar* self)
    : self(self)
{
    self->setVisibleByDefault(true);
    self->setStretchable(true);
    
    self->time_ = 0.0;
    self->frameRate_ = DEFAULT_FRAME_RATE;
    decimals = 2;
    minTime = 0.0;
    maxTime = 0.0;
    playbackFrameRate = DEFAULT_PLAYBACK_FRAMERATE;
    playbackSpeedRatio = 1.0;
    ongoingTime = 0.0;
    isIdleEventDrivenMode = false;
    isRepeatMode = false;
    isAutoExpansionMode = true;
    isDoingPlayback = false;
    isPlaybackProcessing = false;
    hasOngoingTime = false;
    isOngoingTimeSyncEnabled = true;
    enabledTimeUnits = UnitSecond;
    timerId = 0;

    auto playButton = self->addButton(":/Base/icon/play.svg", PlayButton);
    playButton->setToolTip(_("Start playback"));
    playButton->sigClicked().connect([this](){ onPlayActivated(); });

    resumeIcon = QtSvgUtil::createIconFromSvgFile(":/Base/icon/resume.svg");
    stopIcon = QtSvgUtil::createIconFromSvgFile(":/Base/icon/stop.svg");

    resumeButton = self->addButton(resumeIcon, ResumeButton);
    resumeButton->setToolTip(_("Resume playback"));
    resumeButton->sigClicked().connect([this](){ onResumeActivated(); });

    auto refreshButton = self->addButton(":/Base/icon/refresh.svg", RefreshButton);
    refreshButton->setToolTip(_("Refresh state at the current time"));
    refreshButton->sigClicked().connect([this](){ this->self->refresh(); });

    dayTimeSpin = new DoubleSpinBox;
    dayTimeSpin->setAlignment(Qt::AlignCenter);
    dayTimeSpin->setDecimals(0);
    dayTimeSpin->setRange(0, 99999);
    dayTimeSpin->sigValueChanged().connect([this](double){ onTimeUnitSpinValueChanged(dayTimeSpin); });
    self->addWidget(dayTimeSpin, DayTimeSpin);
    dayTimeLabel = self->addLabel(_("D "), DayTimeLabel);

    hourTimeSpin = new DoubleSpinBox;
    hourTimeSpin->setAlignment(Qt::AlignCenter);
    hourTimeSpin->setDecimals(0);
    hourTimeSpin->setRange(0, 99999);
    hourTimeSpin->sigValueChanged().connect([this](double){ onTimeUnitSpinValueChanged(hourTimeSpin); });
    self->addWidget(hourTimeSpin, HourTimeSpin);
    hourTimeLabel = self->addLabel(_("H "), HourTimeLabel);

    minuteTimeSpin = new DoubleSpinBox;
    minuteTimeSpin->setAlignment(Qt::AlignCenter);
    minuteTimeSpin->setDecimals(0);
    minuteTimeSpin->setRange(0, 99999);
    minuteTimeSpin->sigValueChanged().connect([this](double){ onTimeUnitSpinValueChanged(minuteTimeSpin); });
    self->addWidget(minuteTimeSpin, MinuteTimeSpin);
    minuteTimeLabel = self->addLabel(_("M "), MinuteTimeLabel);

    timeSpin = new DoubleSpinBox;
    timeSpin->setAlignment(Qt::AlignCenter);
    timeSpin->sigValueChanged().connect(
        [this](double){ onTimeUnitSpinValueChanged(timeSpin); });
    self->addWidget(timeSpin, TimeSpin);
    secondTimeLabel = self->addLabel(_("S"), SecondTimeLabel);

    timeSlider = new Slider(Qt::Horizontal);
    timeSlider->sigValueChanged().connect(
        [this](int value){ setTime(value / pow(10.0, decimals), Truncate, false, timeSlider); });
    timeSlider->setMinimumWidth(timeSlider->sizeHint().width());
    self->addWidget(timeSlider, TimeSlider);

    minTimeSpin = new DoubleSpinBox;
    minTimeSpin->setAlignment(Qt::AlignCenter);
    minTimeSpin->setRange(isNegativeTimeEnabled ? -9999.0 : 0.0, 9999.0);
    minTimeSpin->sigValueChanged().connect([this](double time){ onMinTimeSpinValueChanged(time); });
    self->addWidget(minTimeSpin, TimeRangeMinSpin);

    timeRangeDelimiterLabel = self->addLabel(" : ");

    maxTimeSpin = new DoubleSpinBox;
    maxTimeSpin->setAlignment(Qt::AlignCenter);
    maxTimeSpin->setRange(isNegativeTimeEnabled ? -9999.0 : 0.0, 9999.0);
    maxTimeSpin->sigValueChanged().connect([this](double time){ onMaxTimeSpinValueChanged(time); });
    self->addWidget(maxTimeSpin, TimeRangeMaxSpin);
    maxTimeUnitLabel = self->addLabel("", MaxTimeUnitLabel);

    auto configButton = self->addButton(":/Base/icon/setup.svg", ConfigButton);
    configButton->setToolTip(_("Show the config dialog"));
    configButton->sigClicked().connect([this](){ configDialog->show(); });

    // Initially only seconds are enabled; hide day/hour/minute widgets
    dayTimeSpin->hide();
    dayTimeLabel->hide();
    hourTimeSpin->hide();
    hourTimeLabel->hide();
    minuteTimeSpin->hide();
    minuteTimeLabel->hide();
    secondTimeLabel->hide();
    maxTimeUnitLabel->hide();

    setTimeRange(0.0, 30.0, true);
}


TimeBar::Impl::ConfigDialog::ConfigDialog(TimeBar* timeBar)
    : Dialog(timeBar)
{
    auto impl = timeBar->impl;

    setWindowTitle(_("Time Bar Config"));

    auto vbox = new QVBoxLayout;
    setLayout(vbox);

    // Grid layout for frame rate / speed ratio spins
    auto grid = new QGridLayout;

    grid->addWidget(new QLabel(_("Internal frame rate:")), 0, 0);
    frameRateSpin.setAlignment(Qt::AlignCenter);
    frameRateSpin.setRange(1.0, 10000.0);
    frameRateSpin.setDecimals(0);
    frameRateSpin.setValue(timeBar->frameRate_);
    frameRateSpin.sigValueChanged().connect(
        [impl](double value){ impl->setFrameRate(value, true, true); });
    grid->addWidget(&frameRateSpin, 0, 1);

    grid->addWidget(new QLabel(_("Playback frame rate:")), 1, 0);
    playbackFrameRateSpin.setAlignment(Qt::AlignCenter);
    playbackFrameRateSpin.setRange(0.0, 1000.0);
    playbackFrameRateSpin.setDecimals(0);
    playbackFrameRateSpin.setValue(impl->playbackFrameRate);
    playbackFrameRateSpin.sigValueChanged().connect(
        [impl](double value){ impl->setPlaybackFrameRate(value, true); });
    grid->addWidget(&playbackFrameRateSpin, 1, 1);

    idleEventDrivenModeCheck.setText(_("Idle event driven mode"));
    idleEventDrivenModeCheck.setChecked(impl->isIdleEventDrivenMode);
    idleEventDrivenModeCheck.sigToggled().connect(
        [impl](bool on){ impl->setIdleEventDrivenMode(on, true); });
    grid->addWidget(&idleEventDrivenModeCheck, 2, 0, 1, 2, Qt::AlignRight);

    grid->addWidget(new QLabel(_("Playback speed ratio:")), 3, 0);
    playbackSpeedRatioSpin.setAlignment(Qt::AlignCenter);
    playbackSpeedRatioSpin.setDecimals(1);
    playbackSpeedRatioSpin.setRange(0.1, 999999.9);
    playbackSpeedRatioSpin.setSingleStep(0.1);
    playbackSpeedRatioSpin.setValue(impl->playbackSpeedRatio);
    playbackSpeedRatioSpin.sigValueChanged().connect(
        [impl](double value){ impl->setPlaybackSpeedRatio(value, true); });
    grid->addWidget(&playbackSpeedRatioSpin, 3, 1);

    auto gridHBox = new QHBoxLayout;
    gridHBox->addLayout(grid);
    gridHBox->addStretch();
    vbox->addLayout(gridHBox);

    vbox->addWidget(new QLabel(_("Time display units:")));

    auto hbox = new QHBoxLayout;
    dayCheck.setText(_("Day"));
    dayCheck.setChecked(impl->enabledTimeUnits & UnitDay);
    dayCheck.sigToggled().connect([impl](bool){ impl->updateTimeUnitChecks(); });
    hbox->addWidget(&dayCheck);
    hourCheck.setText(_("Hour"));
    hourCheck.setChecked(impl->enabledTimeUnits & UnitHour);
    hourCheck.sigToggled().connect([impl](bool){ impl->updateTimeUnitChecks(); });
    hbox->addWidget(&hourCheck);
    minuteCheck.setText(_("Min"));
    minuteCheck.setChecked(impl->enabledTimeUnits & UnitMinute);
    minuteCheck.sigToggled().connect([impl](bool){ impl->updateTimeUnitChecks(); });
    hbox->addWidget(&minuteCheck);
    secondCheck.setText(_("Sec"));
    secondCheck.setChecked(impl->enabledTimeUnits & UnitSecond);
    secondCheck.sigToggled().connect([impl](bool){ impl->updateTimeUnitChecks(); });
    hbox->addWidget(&secondCheck);
    hbox->addStretch();
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout;
    ongoingTimeSyncCheck.setText(_("Sync with ongoing updates"));
    ongoingTimeSyncCheck.setChecked(impl->isOngoingTimeSyncEnabled);
    ongoingTimeSyncCheck.sigToggled().connect(
        [impl](bool on){ impl->setOngoingTimeSyncEnabled(on, true); });
    hbox->addWidget(&ongoingTimeSyncCheck);
    hbox->addStretch();
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout;
    autoExpansionCheck.setText(_("Automatically expand the time range"));
    autoExpansionCheck.setChecked(impl->isAutoExpansionMode);
    autoExpansionCheck.sigToggled().connect(
        [impl](bool on){ impl->setAutoExpansionMode(on, true); });
    hbox->addWidget(&autoExpansionCheck);
    hbox->addStretch();
    vbox->addLayout(hbox);

    vbox->addStretch();
    vbox->addWidget(new HSeparator);

    auto okButton = new PushButton(_("&OK"));
    okButton->setDefault(true);
    auto buttonBox = new QDialogButtonBox(this);
    buttonBox->addButton(okButton, QDialogButtonBox::AcceptRole);
    connect(buttonBox,SIGNAL(accepted()), this, SLOT(accept()));
    vbox->addWidget(buttonBox);
}


TimeBar::~TimeBar()
{
    delete impl;
}


TimeBar::Impl::~Impl()
{

}


void TimeBar::onActiveElementUpdated()
{
    if(!impl->minTimeSpin->isHidden() && !impl->maxTimeSpin->isHidden()){
        int pos = elementPosition(TimeRangeMinSpin);
        if(pos >= 0){
            pos += 1;
        }
        setInsertionPosition(pos);
        addWidget(impl->timeRangeDelimiterLabel);
        impl->timeRangeDelimiterLabel->show();
    }
}


SignalProxy<bool(double time)> TimeBar::sigPlaybackInitialized()
{
    return impl->sigPlaybackInitialized;
}


SignalProxy<void(double time)> TimeBar::sigPlaybackStarted()
{
    return impl->sigPlaybackStarted;
}


SignalProxy<bool(double time)> TimeBar::sigTimeChanged()
{
    return impl->sigTimeChanged;
}


SignalProxy<void(double time, bool isStoppedManually)> TimeBar::sigPlaybackStopped()
{
    return impl->sigPlaybackStopped;
}


SignalProxy<double(double time, bool isStoppedManually)> TimeBar::sigPlaybackStoppedEx()
{
    return impl->sigPlaybackStoppedEx;
}


double TimeBar::minTime() const
{
    return impl->minTime;
}


double TimeBar::maxTime() const
{
    return impl->maxTime;
}


void TimeBar::setTimeRange(double minTime, double maxTime)
{
    impl->setTimeRange(minTime, maxTime, true);
}


bool TimeBar::Impl::setTimeRange(double newMinTime, double newMaxTime, bool doUpdateTimeSpinSlider)
{
    bool updated = false;

    if(!isNegativeTimeEnabled){
        if(newMinTime < 0.0){
            newMinTime = 0.0;
        }
        if(newMaxTime < 0.0){
            newMaxTime = 0.0;
        }
    }

    if((newMinTime != minTime || newMaxTime != maxTime) && newMinTime <= newMaxTime){
        minTime = newMinTime;
        maxTime = newMaxTime;

        // Convert to display unit for min/max spins
        double minDisplay = minTime;
        double maxDisplay = maxTime;
        if(enabledTimeUnits != UnitSecond){
            // Find the highest enabled unit
            double divisor = 1.0;
            if(enabledTimeUnits & UnitDay) divisor = 86400.0;
            else if(enabledTimeUnits & UnitHour) divisor = 3600.0;
            else if(enabledTimeUnits & UnitMinute) divisor = 60.0;
            minDisplay = minTime / divisor;
            maxDisplay = maxTime / divisor;
        }

        minTimeSpin->blockSignals(true);
        minTimeSpin->setValue(minDisplay);
        minTimeSpin->blockSignals(false);

        maxTimeSpin->blockSignals(true);
        maxTimeSpin->setValue(maxDisplay);
        maxTimeSpin->blockSignals(false);

        updated = true;
        if(doUpdateTimeSpinSlider){
            updateTimeSpinSlider();
        }
    }

    return updated;
}


void TimeBar::Impl::onMinTimeSpinValueChanged(double value)
{
    // Convert from display unit to seconds
    double divisor = 1.0;
    if(enabledTimeUnits != UnitSecond){
        if(enabledTimeUnits & UnitDay) divisor = 86400.0;
        else if(enabledTimeUnits & UnitHour) divisor = 3600.0;
        else if(enabledTimeUnits & UnitMinute) divisor = 60.0;
    }
    double newMinTime = value * divisor;
    double newMaxTime = maxTime;
    if(newMinTime > newMaxTime){
        newMaxTime = newMinTime;
    }
    setTimeRange(newMinTime, newMaxTime, true);
}


void TimeBar::Impl::onMaxTimeSpinValueChanged(double value)
{
    // Convert from display unit to seconds
    double divisor = 1.0;
    if(enabledTimeUnits != UnitSecond){
        if(enabledTimeUnits & UnitDay) divisor = 86400.0;
        else if(enabledTimeUnits & UnitHour) divisor = 3600.0;
        else if(enabledTimeUnits & UnitMinute) divisor = 60.0;
    }
    double newMaxTime = value * divisor;
    double newMinTime = minTime;
    if(newMaxTime < newMinTime){
        newMinTime = newMaxTime;
    }
    setTimeRange(newMinTime, newMaxTime, true);
}


void TimeBar::setFrameRate(double rate)
{
    impl->setFrameRate(rate, false, true);
}


bool TimeBar::Impl::setFrameRate(double rate, bool isSpinBoxSignal, bool doUpdateTimeSpinSlider)
{
    bool updated = false;
    
    if((rate != self->frameRate_) && rate > 0.0){
        self->frameRate_ = rate;
        if(!isSpinBoxSignal){
            auto& spin = configDialog->frameRateSpin;
            spin.blockSignals(true);
            spin.setValue(rate);
            spin.blockSignals(false);
        }
        if(doUpdateTimeSpinSlider){
            updateTimeSpinSlider();
        }
        updated = true;
    }

    return updated;
}


void TimeBar::Impl::updateTimeSpinSlider()
{
    const double timeStep = 1.0 / self->frameRate_;
    decimals = static_cast<int>(ceil(log10(self->frameRate_)));
    const double r = pow(10.0, decimals);

    // Determine the lowest enabled unit for decimals
    int lowestUnit = UnitSecond;
    if(enabledTimeUnits & UnitSecond) lowestUnit = UnitSecond;
    else if(enabledTimeUnits & UnitMinute) lowestUnit = UnitMinute;
    else if(enabledTimeUnits & UnitHour) lowestUnit = UnitHour;
    else lowestUnit = UnitDay;

    // Update day spin
    if(enabledTimeUnits & UnitDay){
        dayTimeSpin->blockSignals(true);
        dayTimeSpin->setDecimals(lowestUnit == UnitDay ? decimals : 0);
        dayTimeSpin->setSingleStep(lowestUnit == UnitDay ? timeStep / 86400.0 : 1.0);
        dayTimeSpin->setRange(0, 99999);
        dayTimeSpin->blockSignals(false);
    }
    // Update hour spin
    if(enabledTimeUnits & UnitHour){
        hourTimeSpin->blockSignals(true);
        if(lowestUnit == UnitHour){
            hourTimeSpin->setDecimals(decimals);
            hourTimeSpin->setSingleStep(timeStep / 3600.0);
        } else {
            hourTimeSpin->setDecimals(0);
            hourTimeSpin->setSingleStep(1.0);
        }
        hourTimeSpin->setRange(0, (enabledTimeUnits & UnitDay) ? 23 : 99999);
        hourTimeSpin->blockSignals(false);
    }
    // Update minute spin
    if(enabledTimeUnits & UnitMinute){
        minuteTimeSpin->blockSignals(true);
        if(lowestUnit == UnitMinute){
            minuteTimeSpin->setDecimals(decimals);
            minuteTimeSpin->setSingleStep(timeStep / 60.0);
        } else {
            minuteTimeSpin->setDecimals(0);
            minuteTimeSpin->setSingleStep(1.0);
        }
        minuteTimeSpin->setRange(0, (enabledTimeUnits & (UnitHour | UnitDay)) ? 59 : 99999);
        minuteTimeSpin->blockSignals(false);
    }
    // Update second spin (timeSpin)
    if(enabledTimeUnits & UnitSecond){
        timeSpin->blockSignals(true);
        if(lowestUnit == UnitSecond){
            timeSpin->setDecimals(decimals);
            timeSpin->setSingleStep(timeStep);
        } else {
            timeSpin->setDecimals(0);
            timeSpin->setSingleStep(1.0);
        }
        timeSpin->setRange(0, (enabledTimeUnits & (UnitMinute | UnitHour | UnitDay)) ? 59.999999 : maxTime);
        if(enabledTimeUnits == UnitSecond){
            // Seconds-only mode: use the full range
            timeSpin->setRange(minTime, maxTime);
        }
        timeSpin->blockSignals(false);
    }

    // Update range spin decimals
    if(enabledTimeUnits == UnitSecond){
        minTimeSpin->setDecimals(decimals);
        maxTimeSpin->setDecimals(decimals);
    } else {
        // For non-second highest units, use integer display
        // except when the highest unit is second (only mode)
        int highestUnit = 0;
        if(enabledTimeUnits & UnitDay) highestUnit = UnitDay;
        else if(enabledTimeUnits & UnitHour) highestUnit = UnitHour;
        else if(enabledTimeUnits & UnitMinute) highestUnit = UnitMinute;
        else highestUnit = UnitSecond;

        if(highestUnit == UnitSecond){
            minTimeSpin->setDecimals(decimals);
            maxTimeSpin->setDecimals(decimals);
        } else {
            minTimeSpin->setDecimals(0);
            maxTimeSpin->setDecimals(0);
        }
    }

    timeSlider->blockSignals(true);
    timeSlider->setRange((int)nearbyint(minTime * r), (int)nearbyint(maxTime * r));
    timeSlider->setSingleStep(timeStep * r);
    timeSlider->blockSignals(false);

    if(self->time_ < minTime){
        self->time_ = minTime;
    } else if(self->time_ > maxTime){
        self->time_ = maxTime;
    }
    setTime(self->time_);
}

    
double TimeBar::playbackFrameRate() const
{
    return impl->playbackFrameRate;
}


void TimeBar::setPlaybackFrameRate(double rate)
{
    impl->setPlaybackFrameRate(rate, false);
}


void TimeBar::Impl::setPlaybackFrameRate(double rate, bool isSpinBoxSignal)
{
    if(rate != playbackFrameRate){
        playbackFrameRate = rate;
        if(!isSpinBoxSignal){
            auto& spin = configDialog->playbackFrameRateSpin;
            spin.blockSignals(true);
            spin.setValue(rate);
            spin.blockSignals(false);
        }
        if(isDoingPlayback){
            startPlayback();
        }
    }
}


bool TimeBar::isIdleEventDrivenMode() const
{
    return impl->isIdleEventDrivenMode;
}


void TimeBar::setIdleEventDrivenMode(bool on)
{
    impl->setIdleEventDrivenMode(on, false);
}


void TimeBar::Impl::setIdleEventDrivenMode(bool on, bool isCheckBoxSignal)
{
    if(on != isIdleEventDrivenMode){
        isIdleEventDrivenMode = on;
        if(isCheckBoxSignal){
            auto& check = configDialog->idleEventDrivenModeCheck;
            check.blockSignals(true);
            check.setChecked(on);
            check.blockSignals(false);
        }
        configDialog->playbackFrameRateSpin.setEnabled(!on);
        if(isDoingPlayback){
            startPlayback();
        }
    }
}
    

double TimeBar::playbackSpeedRatio() const
{
    return impl->playbackSpeedRatio;
}


void TimeBar::setPlaybackSpeedRatio(double ratio)
{
    impl->setPlaybackSpeedRatio(ratio, false);
}


void TimeBar::Impl::setPlaybackSpeedRatio(double ratio, bool isSpinBoxSignal)
{
    if(ratio != playbackSpeedRatio){
       playbackSpeedRatio = ratio;
       if(!isSpinBoxSignal){
           auto& spin = configDialog->playbackSpeedRatioSpin;
           spin.blockSignals(true);
           spin.setValue(ratio);
           spin.blockSignals(false);
       }
       if(isDoingPlayback){
           startPlayback();
       }
    }
}


void TimeBar::setRepeatMode(bool on)
{
    impl->isRepeatMode = on;
}


void TimeBar::Impl::setAutoExpansionMode(bool on, bool isCheckBoxSignal)
{
    if(on != isAutoExpansionMode){
        isAutoExpansionMode = on;
        if(!isCheckBoxSignal){
            auto& check = configDialog->autoExpansionCheck;
            check.blockSignals(true);
            check.setChecked(on);
            check.blockSignals(false);
        }
    }
}


void TimeBar::Impl::onPlayActivated()
{
    stopPlayback(true);
    startPlayback(minTime);
}


void TimeBar::Impl::onResumeActivated()
{
    if(isDoingPlayback){
        stopPlayback(true);
    } else {
        stopPlayback(true);
        startPlayback();
    }
}


void TimeBar::startPlayback()
{
    impl->startPlayback(time_);
}


void TimeBar::startPlayback(double time)
{
    impl->startPlayback(time);
}


void TimeBar::Impl::startPlayback()
{
    startPlayback(self->time_);
}


void TimeBar::Impl::startPlayback(double time)
{
    stopPlayback(false);

    isPlaybackProcessing = true;

    bool isOngoingTimeValid = hasOngoingTime && isOngoingTimeSyncEnabled;
    if(isOngoingTimeValid){
        time = ongoingTime;
    }

    self->time_ = quantizedTime(time);
    animationTimeOffset = self->time_;

    bool doStartPlayback = true;
    sigPlaybackInitialized.emitAndGetAllResults(self->time_, playbackContinueFlags);
    for(auto flag : playbackContinueFlags){
        if(!flag){
            doStartPlayback = false;
            break;
        }
    }
    
    if(doStartPlayback){
        sigPlaybackStarted(self->time_);
        
        if(!setTime(self->time_) && !isOngoingTimeValid){
            sigPlaybackStopped(self->time_, false);
            sigPlaybackStoppedEx(self->time_, false);

        } else {
            isDoingPlayback = true;
            dayTimeSpin->setUserInputEnabled(false);
            hourTimeSpin->setUserInputEnabled(false);
            minuteTimeSpin->setUserInputEnabled(false);
            timeSpin->setUserInputEnabled(false);
            timeSlider->setUserInputEnabled(false);

            const static QString tip(_("Stop animation"));
            resumeButton->setIcon(stopIcon);
            resumeButton->setToolTip(tip);
            int interval;
            if(isIdleEventDrivenMode){
                interval = 0;
            } else {
                interval = nearbyint(1000.0 / playbackFrameRate);
            }
            timerId = startTimer(interval, Qt::PreciseTimer);
            elapsedTimer.start();
        }
    }

    if(!isDoingPlayback){
        isPlaybackProcessing = false;
    }
}


void TimeBar::stopPlayback(bool isStoppedManually)
{
    impl->stopPlayback(isStoppedManually);
}


double TimeBar::Impl::stopPlayback(bool isStoppedManually)
{
    double lastValidTime;
    
    if(!isDoingPlayback){
        lastValidTime = self->time_;

    } else {
        killTimer(timerId);
        isDoingPlayback = false;
        dayTimeSpin->setUserInputEnabled(true);
        hourTimeSpin->setUserInputEnabled(true);
        minuteTimeSpin->setUserInputEnabled(true);
        timeSpin->setUserInputEnabled(true);
        timeSlider->setUserInputEnabled(true);

        if(hasOngoingTime && ongoingTime > self->time_ && isOngoingTimeSyncEnabled){
            setTime(ongoingTime);
        }

        sigPlaybackStopped(self->time_, isStoppedManually);

        sigPlaybackStoppedEx.emitAndGetAllResults(self->time_, isStoppedManually, lastValidTimesWhenStopped);
        lastValidTime = 0.0;
        for(auto& time : lastValidTimesWhenStopped){
            if(time > lastValidTime){
                lastValidTime = time;
            }
        }
        
        const static QString tip(_("Resume animation"));
        resumeButton->setIcon(resumeIcon);
        resumeButton->setToolTip(tip);

        if(ongoingTimeMap.empty()){
            hasOngoingTime = false;
        }
    }

    isPlaybackProcessing = false;

    return lastValidTime;
}


bool TimeBar::isDoingPlayback() const
{
    return impl->isDoingPlayback;
}


bool TimeBar::isPlaybackProcessing() const
{
    return impl->isPlaybackProcessing;
}


void TimeBar::Impl::timerEvent(QTimerEvent*)
{
    double time = animationTimeOffset + playbackSpeedRatio * (elapsedTimer.elapsed() / 1000.0);

    bool doAutoExpansion = isAutoExpansionMode;
    bool doStopAtLastOngoingTime = false;
    if(hasOngoingTime){
        if(isOngoingTimeSyncEnabled || (time > ongoingTime)){
            animationTimeOffset += (ongoingTime - time);
            time = ongoingTime;
            if(ongoingTimeMap.empty()){
                doStopAtLastOngoingTime = true;
            }
            if(isOngoingTimeSyncEnabled){
                doAutoExpansion = true;
            }
        }
    }

    int options = doAutoExpansion ? (Truncate | Expand) : Truncate;
    if(!setTime(time, options, true) || doStopAtLastOngoingTime){
        double lastValidTime = stopPlayback(false);
        
        if(!doStopAtLastOngoingTime && isRepeatMode){
            startPlayback(minTime);
        } else {
            if(lastValidTime < self->time_){
                setTimeBarTime(lastValidTime);
            }
        }
    }
}


double TimeBar::Impl::quantizedTime(double time) const
{
    return floor(time * self->frameRate_) / self->frameRate_;
}


bool TimeBar::setTime(double time, int options)
{
    return impl->setTime(time, options);
}


/**
   @todo check whether block() and unblock() of sigc::connection
   decrease the performance or not.
*/
bool TimeBar::Impl::setTime
(double time, int options, bool calledFromPlaybackLoop, QWidget* callerWidget)
{
    if(TRACE_FUNCTIONS){
        cout << "TimeBar::Impl::setTime(" << time << ", " << calledFromPlaybackLoop << ")" << endl;
    }
    
    if(!calledFromPlaybackLoop && isDoingPlayback){
        return false;
    }

    double newTime;
    if(options & Round){
        double ftime1 = time * self->frameRate_;
        double ftime2 = floor(ftime1);
        ftime2 += std::round(ftime1 - ftime2);
        newTime = ftime2 / self->frameRate_;
    } else {
        newTime = quantizedTime(time);
    }

    // Avoid redundant update
    if(calledFromPlaybackLoop || callerWidget){
        // When the compiler optimization is enabled,
        // the result of (newTime == self->time_) sometimes becomes false,
        // so here the following judgement is used.
        if(fabs(newTime - self->time_) < 1.0e-14){
            return calledFromPlaybackLoop;
        }
    }

    bool isValid = false;

    bool doExpansion = options & Expand;
    bool isWithinTimeRange = setTimeBarTime(newTime, doExpansion, calledFromPlaybackLoop, callerWidget);
    if(isWithinTimeRange || calledFromPlaybackLoop){
        sigTimeChanged.emitAndGetAllResults(self->time_, playbackContinueFlags);
        for(auto flag : playbackContinueFlags){
            if(flag){
                isValid = true;
                break;
            }
        }
    }
    
    return isValid;
}


bool TimeBar::Impl::setTimeBarTime
(double time, bool doAutoExpansion, bool calledFromPlaybackLoop, QWidget* callerWidget)
{
    bool isWithinTimeRange = true;
    bool doExpand = false;

    if(time < minTime){
        if(doAutoExpansion && (time >= 0.0 || isNegativeTimeEnabled)){
            minTime = time;
            double minDisplay = minTime;
            if(enabledTimeUnits != UnitSecond){
                double divisor = 1.0;
                if(enabledTimeUnits & UnitDay) divisor = 86400.0;
                else if(enabledTimeUnits & UnitHour) divisor = 3600.0;
                else if(enabledTimeUnits & UnitMinute) divisor = 60.0;
                minDisplay = minTime / divisor;
            }
            minTimeSpin->blockSignals(true);
            minTimeSpin->setValue(minDisplay);
            minTimeSpin->blockSignals(false);
            doExpand = true;
        } else {
            isWithinTimeRange = false;
            if(calledFromPlaybackLoop){
                time = minTime;
            }
        }
    }
    if(time > maxTime){
        if(doAutoExpansion){
            maxTime = time;
            double maxDisplay = maxTime;
            if(enabledTimeUnits != UnitSecond){
                double divisor = 1.0;
                if(enabledTimeUnits & UnitDay) divisor = 86400.0;
                else if(enabledTimeUnits & UnitHour) divisor = 3600.0;
                else if(enabledTimeUnits & UnitMinute) divisor = 60.0;
                maxDisplay = maxTime / divisor;
            }
            maxTimeSpin->blockSignals(true);
            maxTimeSpin->setValue(maxDisplay);
            maxTimeSpin->blockSignals(false);
            doExpand = true;
        } else {
            isWithinTimeRange = false;
            if(calledFromPlaybackLoop){
                time = maxTime;
            }
        }
    }
    if(doExpand){
        if(enabledTimeUnits == UnitSecond){
            timeSpin->blockSignals(true);
            timeSpin->setRange(minTime, maxTime);
            timeSpin->blockSignals(false);
        }
        timeSlider->blockSignals(true);
        const double r = pow(10.0, decimals);
        timeSlider->setRange((int)nearbyint(minTime * r), (int)nearbyint(maxTime * r));
        timeSlider->blockSignals(false);
    }

    if(isWithinTimeRange || calledFromPlaybackLoop){
        self->time_ = time;

        // Decompose time and update all unit spins
        double days, hours, minutes, seconds;
        decomposeTime(self->time_, days, hours, minutes, seconds);

        if(callerWidget != dayTimeSpin && (enabledTimeUnits & UnitDay)){
            dayTimeSpin->blockSignals(true);
            dayTimeSpin->setValue(days);
            dayTimeSpin->blockSignals(false);
        }
        if(callerWidget != hourTimeSpin && (enabledTimeUnits & UnitHour)){
            hourTimeSpin->blockSignals(true);
            hourTimeSpin->setValue(hours);
            hourTimeSpin->blockSignals(false);
        }
        if(callerWidget != minuteTimeSpin && (enabledTimeUnits & UnitMinute)){
            minuteTimeSpin->blockSignals(true);
            minuteTimeSpin->setValue(minutes);
            minuteTimeSpin->blockSignals(false);
        }
        if(callerWidget != timeSpin && (enabledTimeUnits & UnitSecond)){
            timeSpin->blockSignals(true);
            timeSpin->setValue(seconds);
            timeSpin->blockSignals(false);
        }
        if(callerWidget != timeSlider){
            timeSlider->blockSignals(true);
            timeSlider->setValue((int)nearbyint(self->time_ * pow(10.0, decimals)));
            timeSlider->blockSignals(false);
        }
    }

    return isWithinTimeRange;
}


void TimeBar::refresh()
{
    if(!impl->isDoingPlayback){
        impl->setTime(time_);
    }
}


bool TimeBar::isOngoingTimeSyncEnabled() const
{
    return impl->isOngoingTimeSyncEnabled;
}


void TimeBar::setOngoingTimeSyncEnabled(bool on)
{
    impl->setOngoingTimeSyncEnabled(on, false);
}


void TimeBar::Impl::setOngoingTimeSyncEnabled(bool on, bool isCheckBoxSignal)
{
    if(on != isOngoingTimeSyncEnabled){
        isOngoingTimeSyncEnabled = on;
        if(!isCheckBoxSignal){
            auto& check = configDialog->ongoingTimeSyncCheck;
            check.blockSignals(true);
            check.setChecked(on);
            check.blockSignals(false);
        }
    }
}


int TimeBar::startOngoingTimeUpdate(double time)
{
    return impl->startOngoingTimeUpdate(time);
}


int TimeBar::Impl::startOngoingTimeUpdate(double time)
{
    int id = 0;

    if(ongoingTimeMap.empty()){
        hasOngoingTime = true;
    }
    while(true){
        auto inserted = ongoingTimeMap.insert(make_pair(id, time)).second;
        if(inserted){
            updateMinOngoingTime();
            break;
        }
        ++id; // try to find an unused id
    }

    return id;
}


void TimeBar::updateOngoingTime(int id, double time)
{
    impl->updateOngoingTime(id, time);
}


void TimeBar::Impl::updateOngoingTime(int id, double time)
{
    auto it = ongoingTimeMap.find(id);
    if(it != ongoingTimeMap.end()){
        it->second = time;
        updateMinOngoingTime();
    }
}


void TimeBar::Impl::updateMinOngoingTime()
{
    if(!ongoingTimeMap.empty()){
        double minOngoingTime = std::numeric_limits<double>::max();
        for(auto& kv : ongoingTimeMap){
            minOngoingTime = std::min(kv.second, minOngoingTime);
        }
        ongoingTime = minOngoingTime;
    }
}    


void TimeBar::stopOngoingTimeUpdate(int id)
{
    impl->stopOngoingTimeUpdate(id);
}


void TimeBar::Impl::stopOngoingTimeUpdate(int id)
{
    ongoingTimeMap.erase(id);

    if(!ongoingTimeMap.empty()){
        updateMinOngoingTime();
    } else {
        if(!isDoingPlayback){
            hasOngoingTime = false;
        }
    }
}


double TimeBar::realPlaybackTime() const
{
    if(impl->isDoingPlayback){
        return impl->animationTimeOffset + impl->playbackSpeedRatio * (impl->elapsedTimer.elapsed() / 1000.0);
    } else {
        return time_;
    }
}


int TimeBar::stretchableDefaultWidth() const
{
    return sizeHint().width() + impl->timeSlider->sizeHint().width() * 5;
}
    

bool TimeBar::storeState(Archive& archive)
{
    ToolBar::storeState(archive);
    return impl->storeState(archive);
}


bool TimeBar::Impl::storeState(Archive& archive)
{
    archive.write("current_time", self->time_);
    archive.write("min_time", minTime);
    archive.write("max_time", maxTime);
    archive.write("frame_rate", self->frameRate_);
    archive.write("playback_frame_rate", playbackFrameRate);
    archive.write("idle_loop_driven_mode", isIdleEventDrivenMode);
    archive.write("playback_speed_ratio", playbackSpeedRatio);
    archive.write("sync_to_ongoing_updates", isOngoingTimeSyncEnabled);
    archive.write("auto_expansion", isAutoExpansionMode);

    if(enabledTimeUnits != UnitSecond){
        vector<string> units;
        if(enabledTimeUnits & UnitDay) units.push_back("day");
        if(enabledTimeUnits & UnitHour) units.push_back("hour");
        if(enabledTimeUnits & UnitMinute) units.push_back("minute");
        if(enabledTimeUnits & UnitSecond) units.push_back("second");
        if(units.size() == 1){
            archive.write("time_unit", units[0]);
        } else {
            auto& seq = *archive.createFlowStyleListing("time_unit");
            for(auto& u : units) seq.append(u);
        }
    }

    return true;
}


bool TimeBar::restoreState(const Archive& archive)
{
    if(ToolBar::restoreState(archive)){
        return impl->restoreState(archive);
    }
    return false;
}


bool TimeBar::Impl::restoreState(const Archive& archive)
{
    bool doUpdateTimeSpinSlider = false;

    // Restore time_unit before time range so display is correct
    {
        int newUnits = UnitSecond; // default
        string unitStr;
        if(archive.read("time_unit", unitStr)){
            // Single unit string
            newUnits = 0;
            if(unitStr == "day") newUnits = UnitDay;
            else if(unitStr == "hour") newUnits = UnitHour;
            else if(unitStr == "minute") newUnits = UnitMinute;
            else if(unitStr == "second") newUnits = UnitSecond;
            if(newUnits == 0) newUnits = UnitSecond;
        } else {
            auto listing = archive.findListing("time_unit");
            if(listing && listing->isValid()){
                newUnits = 0;
                for(int i = 0; i < listing->size(); ++i){
                    string u = listing->at(i)->toString();
                    if(u == "day") newUnits |= UnitDay;
                    else if(u == "hour") newUnits |= UnitHour;
                    else if(u == "minute") newUnits |= UnitMinute;
                    else if(u == "second") newUnits |= UnitSecond;
                }
                if(newUnits == 0) newUnits = UnitSecond;
            }
        }
        if(newUnits != enabledTimeUnits){
            enabledTimeUnits = newUnits;
            doUpdateTimeSpinSlider = true;
        }
    }

    bool doUpdateTimeRange = false;
    double newMinTime, newMaxTime;
    doUpdateTimeRange |= archive.read({ "min_time", "minTime" }, newMinTime);
    doUpdateTimeRange |= archive.read({ "max_time", "maxTime" }, newMaxTime);
    if(doUpdateTimeRange){
        doUpdateTimeSpinSlider |= setTimeRange(newMinTime, newMaxTime, false);
    }
    double newFrameRate;
    if(archive.read({ "frame_rate", "frameRate" }, newFrameRate)){
        doUpdateTimeSpinSlider |= setFrameRate(newFrameRate, false, false);
    }

    double newTime;
    if(archive.read({ "current_time", "currentTime" }, newTime)){
        if(newTime != self->time_){
            self->time_ = newTime;
            doUpdateTimeSpinSlider = true;
        }
    }

    setPlaybackFrameRate(
        archive.get({ "playback_frame_rate", "playbackFrameRate" }, playbackFrameRate), false);
    setIdleEventDrivenMode(
        archive.get("idle_loop_driven_mode", isIdleEventDrivenMode), false);
    setPlaybackSpeedRatio(
        archive.get("playback_speed_ratio", playbackSpeedRatio), false);
    setOngoingTimeSyncEnabled(
        archive.get("sync_to_ongoing_updates", isOngoingTimeSyncEnabled), false);
    setAutoExpansionMode(
        archive.get("auto_expansion", isAutoExpansionMode), false);

    if(doUpdateTimeSpinSlider){
        updateTimeUnitDisplay();
        updateTimeSpinSlider();
    }

    return true;
}


void TimeBar::Impl::decomposeTime(double totalSeconds, double& days, double& hours, double& minutes, double& seconds)
{
    days = hours = minutes = 0.0;
    double rem = totalSeconds;

    if(enabledTimeUnits & UnitDay){
        days = floor(rem / 86400.0);
        rem -= days * 86400.0;
    }
    if(enabledTimeUnits & UnitHour){
        hours = floor(rem / 3600.0);
        rem -= hours * 3600.0;
    }
    if(enabledTimeUnits & UnitMinute){
        minutes = floor(rem / 60.0);
        rem -= minutes * 60.0;
    }
    seconds = rem;
}


double TimeBar::Impl::composeTimeFromSpins()
{
    double total = 0;
    if(enabledTimeUnits & UnitDay) total += dayTimeSpin->value() * 86400.0;
    if(enabledTimeUnits & UnitHour) total += hourTimeSpin->value() * 3600.0;
    if(enabledTimeUnits & UnitMinute) total += minuteTimeSpin->value() * 60.0;
    if(enabledTimeUnits & UnitSecond) total += timeSpin->value();
    return total;
}


void TimeBar::Impl::onTimeUnitSpinValueChanged(QWidget* callerSpin)
{
    double time = composeTimeFromSpins();
    setTime(time, Truncate, false, callerSpin);
}


void TimeBar::Impl::updateTimeUnitDisplay()
{
    bool multiUnit = (enabledTimeUnits != UnitSecond);

    // Day
    bool dayEnabled = enabledTimeUnits & UnitDay;
    dayTimeSpin->setVisible(dayEnabled);
    dayTimeLabel->setVisible(dayEnabled && multiUnit);

    // Hour
    bool hourEnabled = enabledTimeUnits & UnitHour;
    hourTimeSpin->setVisible(hourEnabled);
    hourTimeLabel->setVisible(hourEnabled && multiUnit);

    // Minute
    bool minuteEnabled = enabledTimeUnits & UnitMinute;
    minuteTimeSpin->setVisible(minuteEnabled);
    minuteTimeLabel->setVisible(minuteEnabled && multiUnit);

    // Second
    bool secondEnabled = enabledTimeUnits & UnitSecond;
    timeSpin->setVisible(secondEnabled);
    secondTimeLabel->setVisible(secondEnabled && multiUnit);

    // Min/Max time unit labels
    if(multiUnit){
        const char* unitLabel = _("S");
        if(enabledTimeUnits & UnitDay) unitLabel = _("D");
        else if(enabledTimeUnits & UnitHour) unitLabel = _("H");
        else if(enabledTimeUnits & UnitMinute) unitLabel = _("M");
        maxTimeUnitLabel->setText(unitLabel);
        maxTimeUnitLabel->show();
    } else {
        maxTimeUnitLabel->hide();
    }

    // Update config dialog checkboxes
    if(configDialog){
        configDialog->dayCheck.blockSignals(true);
        configDialog->dayCheck.setChecked(dayEnabled);
        configDialog->dayCheck.blockSignals(false);

        configDialog->hourCheck.blockSignals(true);
        configDialog->hourCheck.setChecked(hourEnabled);
        configDialog->hourCheck.blockSignals(false);

        configDialog->minuteCheck.blockSignals(true);
        configDialog->minuteCheck.setChecked(minuteEnabled);
        configDialog->minuteCheck.blockSignals(false);

        configDialog->secondCheck.blockSignals(true);
        configDialog->secondCheck.setChecked(secondEnabled);
        configDialog->secondCheck.blockSignals(false);
    }
}


void TimeBar::Impl::updateTimeUnitChecks()
{
    int units = 0;
    if(configDialog->dayCheck.isChecked()) units |= UnitDay;
    if(configDialog->hourCheck.isChecked()) units |= UnitHour;
    if(configDialog->minuteCheck.isChecked()) units |= UnitMinute;
    if(configDialog->secondCheck.isChecked()) units |= UnitSecond;

    // At least one unit must be checked
    if(units == 0){
        // Revert to previous state
        updateTimeUnitDisplay();
        return;
    }

    // Enforce continuity: fill gaps between highest and lowest set bits
    // Find the highest and lowest set bits
    int highest = 0, lowest = 0;
    for(int b = UnitDay; b >= UnitSecond; b >>= 1){
        if(units & b){
            if(highest == 0) highest = b;
            lowest = b;
        }
    }
    // Fill all bits between highest and lowest
    int filled = 0;
    bool filling = false;
    for(int b = UnitDay; b >= UnitSecond; b >>= 1){
        if(b == highest) filling = true;
        if(filling) filled |= b;
        if(b == lowest) break;
    }
    units = filled;

    if(units != enabledTimeUnits){
        enabledTimeUnits = units;
        updateTimeUnitDisplay();

        // Re-apply time range display
        setTimeRange(minTime, maxTime, false);

        updateTimeSpinSlider();
    }
}
