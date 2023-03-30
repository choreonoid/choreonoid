#include "TimeBar.h"
#include "ExtensionManager.h"
#include "Archive.h"
#include "OptionManager.h"
#include "SpinBox.h"
#include "Slider.h"
#include "Buttons.h"
#include "CheckBox.h"
#include "Dialog.h"
#include <QDialogButtonBox>
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
    ConfigButton = 7
};

}

namespace cnoid {

class TimeBar::Impl : public QObject
{
public:
    Impl(TimeBar* self);
    ~Impl();

    double quantizedTime(double time) const;
    bool setTime(double time, bool doAutoExpansion = false, bool calledFromPlaybackLoop = false, QWidget* callerWidget = nullptr);
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

    TimeBar* self;

    ToolButton* resumeButton;
    ToolButton* frameModeToggle;
    QIcon resumeIcon;
    QIcon stopIcon;
    DoubleSpinBox* timeSpin;
    Slider* timeSlider;
    DoubleSpinBox* minTimeSpin;
    DoubleSpinBox* maxTimeSpin;
    QLabel* timeRangeDelimiterLabel;
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
        
        ConfigDialog(TimeBar* timeBar);
    };

    ConfigDialog* configDialog;
};

}


static void onSigOptionsParsed(boost::program_options::variables_map& v)
{
    if(v.count("start-playback")){
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

        ext->optionManager()
            .addOption("start-playback", "start playback automatically")
            .sigOptionsParsed(1).connect(onSigOptionsParsed);
            
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
    : self(self),
      resumeIcon(QIcon(":/Base/icon/resume.svg")),
      stopIcon(QIcon(":/Base/icon/stop.svg"))
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
    hasOngoingTime = false;
    isOngoingTimeSyncEnabled = true;
    timerId = 0;

    auto playButton = self->addButton(QIcon(":/Base/icon/play.svg"), PlayButton);
    playButton->setToolTip(_("Start playback"));
    playButton->sigClicked().connect([this](){ onPlayActivated(); });

    resumeButton = self->addButton(resumeIcon, ResumeButton);
    resumeButton->setToolTip(_("Resume playback"));
    resumeButton->sigClicked().connect([this](){ onResumeActivated(); });

    auto refreshButton = self->addButton(QIcon(":/Base/icon/refresh.svg"), RefreshButton);
    refreshButton->setToolTip(_("Refresh state at the current time"));
    refreshButton->sigClicked().connect([this](){ this->self->refresh(); });
    
    timeSpin = new DoubleSpinBox;
    timeSpin->setAlignment(Qt::AlignCenter);
    timeSpin->sigValueChanged().connect(
        [this](double time){ setTime(time, false, false, timeSpin); });
    self->addWidget(timeSpin, TimeSpin);

    timeSlider = new Slider(Qt::Horizontal);
    timeSlider->sigValueChanged().connect(
        [this](int value){ setTime(value / pow(10.0, decimals), false, false, timeSlider); });
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

    auto configButton = self->addButton(QIcon(":/Base/icon/setup.svg"));
    configButton->setToolTip(_("Show the config dialog"));
    configButton->sigClicked().connect([this](){ configDialog->show(); });

    setTimeRange(0.0, 30.0, true);
}


TimeBar::Impl::ConfigDialog::ConfigDialog(TimeBar* timeBar)
    : Dialog(timeBar)
{
    auto impl = timeBar->impl;
    
    setWindowTitle(_("Time Bar Config"));

    auto vbox = new QVBoxLayout;
    setLayout(vbox);

    auto hbox = new QHBoxLayout;
    hbox->addWidget(new QLabel(_("Internal frame rate")));
    frameRateSpin.setAlignment(Qt::AlignCenter);
    frameRateSpin.setRange(1.0, 10000.0);
    frameRateSpin.setDecimals(0);
    frameRateSpin.setValue(timeBar->frameRate_);
    frameRateSpin.sigValueChanged().connect(
        [impl](double value){ impl->setFrameRate(value, true, true); });

    hbox->addWidget(&frameRateSpin);
    hbox->addStretch();
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout;
    hbox->addWidget(new QLabel(_("Playback frame rate")));
    playbackFrameRateSpin.setAlignment(Qt::AlignCenter);
    playbackFrameRateSpin.setRange(0.0, 1000.0);
    playbackFrameRateSpin.setDecimals(0);
    playbackFrameRateSpin.setValue(impl->playbackFrameRate);
    playbackFrameRateSpin.sigValueChanged().connect(
        [impl](double value){ impl->setPlaybackFrameRate(value, true); });
    hbox->addWidget(&playbackFrameRateSpin);
    hbox->addStretch();
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout;
    idleEventDrivenModeCheck.setText(_("Idle event driven mode"));
    idleEventDrivenModeCheck.setChecked(impl->isIdleEventDrivenMode);
    idleEventDrivenModeCheck.sigToggled().connect(
        [impl](bool on){ impl->setIdleEventDrivenMode(on, true); });
    hbox->addWidget(&idleEventDrivenModeCheck);
    hbox->addStretch();
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout;
    hbox->addWidget(new QLabel(_("Playback speed ratio")));
    playbackSpeedRatioSpin.setAlignment(Qt::AlignCenter);
    playbackSpeedRatioSpin.setDecimals(1);
    playbackSpeedRatioSpin.setRange(0.1, 99.9);
    playbackSpeedRatioSpin.setSingleStep(0.1);
    playbackSpeedRatioSpin.setValue(impl->playbackSpeedRatio);
    playbackSpeedRatioSpin.sigValueChanged().connect(
        [impl](double value){ impl->setPlaybackSpeedRatio(value, true); });
    hbox->addWidget(&playbackSpeedRatioSpin);
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
        setInsertionPosition(elementPosition(TimeRangeMinSpin));
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

        minTimeSpin->blockSignals(true);
        minTimeSpin->setValue(minTime);
        minTimeSpin->blockSignals(false);

        maxTimeSpin->blockSignals(true);
        maxTimeSpin->setValue(maxTime);
        maxTimeSpin->blockSignals(false);

        updated = true;
        if(doUpdateTimeSpinSlider){
            updateTimeSpinSlider();
        }
    }
    
    return updated;
}


void TimeBar::Impl::onMinTimeSpinValueChanged(double minTime)
{
    double maxTime = maxTimeSpin->value();
    if(minTime > maxTime){
        maxTime = minTime;
    }
    setTimeRange(minTime, maxTime, true);
}


void TimeBar::Impl::onMaxTimeSpinValueChanged(double maxTime)
{
    double minTime = minTimeSpin->value();
    if(maxTime < minTime){
        minTime = maxTime;
    }
    setTimeRange(minTime, maxTime, true);
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

    timeSpin->blockSignals(true);
    timeSpin->setRange(minTime, maxTime);
    timeSpin->setDecimals(decimals);
    timeSpin->setSingleStep(timeStep);
    timeSpin->blockSignals(false);

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

    return lastValidTime;
}


bool TimeBar::isDoingPlayback()
{
    return impl->isDoingPlayback;
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

    if(!setTime(time, doAutoExpansion, true) || doStopAtLastOngoingTime){
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


bool TimeBar::setTime(double time)
{
    return impl->setTime(time);
}


/**
   @todo check whether block() and unblock() of sigc::connection
   decrease the performance or not.
*/
bool TimeBar::Impl::setTime
(double time, bool doAutoExpansion, bool calledFromPlaybackLoop, QWidget* callerWidget)
{
    if(TRACE_FUNCTIONS){
        cout << "TimeBar::Impl::setTime(" << time << ", " << calledFromPlaybackLoop << ")" << endl;
    }
    
    if(!calledFromPlaybackLoop && isDoingPlayback){
        return false;
    }

    const double newTime = quantizedTime(time);

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
    
    bool isWithinTimeRange = setTimeBarTime(newTime, doAutoExpansion, calledFromPlaybackLoop, callerWidget);
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
            minTimeSpin->blockSignals(true);
            minTimeSpin->setValue(maxTime);
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
            maxTimeSpin->blockSignals(true);
            maxTimeSpin->setValue(maxTime);
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
        timeSpin->blockSignals(true);
        timeSlider->blockSignals(true);
        timeSpin->setRange(minTime, maxTime);
        const double r = pow(10.0, decimals);
        timeSlider->setRange((int)nearbyint(minTime * r), (int)nearbyint(maxTime * r));
        timeSlider->blockSignals(false);
        timeSpin->blockSignals(false);
    }

    if(isWithinTimeRange || calledFromPlaybackLoop){
        self->time_ = time;

        if(callerWidget != timeSpin){
            timeSpin->blockSignals(true);
            timeSpin->setValue(self->time_);
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
        updateTimeSpinSlider();
    }
    
    return true;
}
