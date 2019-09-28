/**
   @author Shin'ichiro Nakaoka
*/

#include "TimeBar.h"
#include "ExtensionManager.h"
#include "Archive.h"
#include "MessageView.h"
#include "MainWindow.h"
#include "OptionManager.h"
#include "LazyCaller.h"
#include "SpinBox.h"
#include "Slider.h"
#include "Buttons.h"
#include "CheckBox.h"
#include "Dialog.h"
#include <QDialogButtonBox>
#include <cmath>
#include <limits>
#include <iostream>
#include <QElapsedTimer>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

const bool TRACE_FUNCTIONS = false;

const double DEFAULT_FRAME_RATE = 1000.0;

// The following value shoud be same as the display refresh rate to make the animation smooth
const double DEFAULT_PLAYBACK_FRAMERATE = 60.0;
    
class ConfigDialog : public Dialog
{
public:
    SpinBox frameRateSpin;
    SpinBox playbackFrameRateSpin;
    CheckBox idleLoopDrivenCheck;
    DoubleSpinBox playbackSpeedScaleSpin;
    CheckBox fillLevelSyncCheck;
    CheckBox autoExpandCheck;
    QCheckBox beatModeCheck;
    DoubleSpinBox tempoSpin;
    SpinBox beatcSpin;
    SpinBox beatmSpin;
    DoubleSpinBox beatOffsetSpin;

    ConfigDialog() {
        setWindowTitle(_("Time Bar Config"));

        QVBoxLayout* vbox = new QVBoxLayout();
        setLayout(vbox);
        
        QHBoxLayout* hbox = new QHBoxLayout();
        hbox->addWidget(new QLabel(_("Internal frame rate")));
        frameRateSpin.setAlignment(Qt::AlignCenter);
        frameRateSpin.setRange(1, 10000);
        hbox->addWidget(&frameRateSpin);
        hbox->addStretch();
        vbox->addLayout(hbox);

        hbox = new QHBoxLayout();
        hbox->addWidget(new QLabel(_("Playback frame rate")));
        playbackFrameRateSpin.setAlignment(Qt::AlignCenter);
        playbackFrameRateSpin.setRange(0, 1000);
        playbackFrameRateSpin.setValue(DEFAULT_PLAYBACK_FRAMERATE);
        hbox->addWidget(&playbackFrameRateSpin);
        hbox->addStretch();
        vbox->addLayout(hbox);

        hbox = new QHBoxLayout();
        idleLoopDrivenCheck.setText(_("Idle loop driven mode"));
        hbox->addWidget(&idleLoopDrivenCheck);
        hbox->addStretch();
        vbox->addLayout(hbox);
            
        hbox = new QHBoxLayout();
        hbox->addWidget(new QLabel(_("Playback speed scale")));
        playbackSpeedScaleSpin.setAlignment(Qt::AlignCenter);
        playbackSpeedScaleSpin.setDecimals(1);
        playbackSpeedScaleSpin.setRange(0.1, 99.9);
        playbackSpeedScaleSpin.setSingleStep(0.1);
        playbackSpeedScaleSpin.setValue(1.0);
        hbox->addWidget(&playbackSpeedScaleSpin);
        hbox->addStretch();
        vbox->addLayout(hbox);

        hbox = new QHBoxLayout();
        fillLevelSyncCheck.setText(_("Sync with ongoing updates"));
        fillLevelSyncCheck.setChecked(true);
        hbox->addWidget(&fillLevelSyncCheck);
        hbox->addStretch();
        vbox->addLayout(hbox);

        hbox = new QHBoxLayout();
        autoExpandCheck.setText(_("Automatically expand the time range"));
        autoExpandCheck.setChecked(true);
        hbox->addWidget(&autoExpandCheck);
        hbox->addStretch();
        vbox->addLayout(hbox);
            
        /*
          hbox = new QHBoxLayout();
          vbox->addLayout(hbox);
            
          beatModeCheck = new QCheckBox(_("Beat mode"));
          hbox->addWidget(beatModeCheck);

          beatcSpin = new SpinBox();
          beatcSpin->setRange(1, 99);
          hbox->addWidget(beatcSpin);

          hbox->addWidget(new QLabel("/"));

          beatmSpin = new SpinBox();
          beatmSpin->setRange(1, 99);
          hbox->addWidget(beatmSpin);

          hbox->addStretch();
          hbox = new QHBoxLayout();
          vbox->addLayout(hbox);

          hbox->addWidget(new QLabel(_("Tempo")));
          tempoSpin = new DoubleSpinBox();
          tempoSpin->setRange(1.0, 999.99);
          tempoSpin->setDecimals(2);
          hbox->addWidget(tempoSpin);

          hbox->addWidget(new QLabel(_("Offset")));
          beatOffsetSpin = new DoubleSpinBox();
          beatOffsetSpin->setRange(-9.99, 9.99);
          beatOffsetSpin->setDecimals(2);
          beatOffsetSpin->setSingleStep(0.1);
          hbox->addWidget(beatOffsetSpin);
          hbox->addWidget(new QLabel(_("[s]")));

          hbox->addStretch();
        */

        vbox->addStretch();

        PushButton* okButton = new PushButton(_("&OK"));
        okButton->setDefault(true);
        QDialogButtonBox* buttonBox = new QDialogButtonBox(this);
        buttonBox->addButton(okButton, QDialogButtonBox::AcceptRole);
        connect(buttonBox,SIGNAL(accepted()), this, SLOT(accept()));
        vbox->addWidget(okButton);
    }
};

}

namespace cnoid {

class TimeBarImpl : public QObject
{
public:
    TimeBarImpl(TimeBar* self);
    ~TimeBarImpl();

    bool setTime(double time, bool calledFromPlaybackLoop, QWidget* callerWidget = 0);
    void onTimeSpinChanged(double value);
    bool onTimeSliderValueChanged(int value);

    void setTimeRange(double minTime, double maxTime);
    void setFrameRate(double rate);
    void updateTimeProperties(bool forceUpdate);
    void onPlaybackSpeedScaleChanged(double value);
    void onPlaybackFrameRateChanged(int value);
    void onPlayActivated();
    void onResumeActivated();
    void startPlayback();
    void stopPlayback(bool isStoppedManually);
    int startFillLevelUpdate();
    void updateFillLevel(int id, double time);
    void updateMinFillLevel();
    void stopFillLevelUpdate(int id);

    void onTimeRangeSpinsChanged();
    void onFrameRateSpinChanged(int value);

    virtual void timerEvent(QTimerEvent* event);
        
    void onRefreshButtonClicked();

    bool storeState(Archive& archive);
    bool restoreState(const Archive& archive);

    TimeBar* self;
    ostream& os;
    ConfigDialog config;

    ToolButton* stopResumeButton;
    ToolButton* frameModeToggle;
    QIcon resumeIcon;
    QIcon stopIcon;
        
    DoubleSpinBox* timeSpin;
    Slider* timeSlider;
    DoubleSpinBox* minTimeSpin;
    DoubleSpinBox* maxTimeSpin;
    int decimals;
    double minTime;
    double maxTime;
    double playbackSpeedScale;
    double playbackFrameRate;
    double animationTimeOffset;
    int timerId;
    QElapsedTimer elapsedTimer;
    bool repeatMode;
    bool isDoingPlayback;
    map<int, double> fillLevelMap;
    double fillLevel;
    bool isFillLevelActive;

    Signal<bool(double time), LogicalProduct> sigPlaybackInitialized;
    Signal<void(double time)> sigPlaybackStarted;
    Signal<bool(double time), LogicalSum> sigTimeChanged;
    Signal<void(double time, bool isStoppedManually)> sigPlaybackStopped;
};
}


static void onSigOptionsParsed(boost::program_options::variables_map& v)
{
    if(v.count("start-playback")){
        callLater([](){ TimeBar::instance()->startPlayback(); });
    }
}


void TimeBar::initialize(ExtensionManager* ext)
{
    static bool initialized = false;
    if(!initialized){
        ext->addToolBar(TimeBar::instance());

        ext->optionManager()
            .addOption("start-playback", "start playback automatically")
            .sigOptionsParsed().connect(onSigOptionsParsed);
            
        initialized = true;
    }
}


TimeBar* TimeBar::instance()
{
    static TimeBar* timeBar = new TimeBar();
    return timeBar;
}


TimeBar::TimeBar()
    : ToolBar(N_("TimeBar"))
{
    impl = new TimeBarImpl(this);
}


TimeBarImpl::TimeBarImpl(TimeBar* self)
    : self(self),
      os(MessageView::mainInstance()->cout()),
      resumeIcon(QIcon(":/Base/icons/resume.png")),
      stopIcon(QIcon(":/Base/icons/stop.png"))
{
    self->setVisibleByDefault(true);
    self->setStretchable(true);
    
    self->time_ = 0.0;
    self->frameRate_ = DEFAULT_FRAME_RATE;
    decimals = 2;
    minTime = 0.0;
    maxTime = 30.0;
    repeatMode = false;
    timerId = 0;
    isDoingPlayback = false;
    fillLevel = 0;
    isFillLevelActive = false;

    self->addButton(QIcon(":/Base/icons/play.png"), _("Start animation"))
        ->sigClicked().connect([&](){ onPlayActivated(); });

    stopResumeButton = self->addButton(resumeIcon, _("Resume animation"));
    stopResumeButton->setIcon(resumeIcon);
    stopResumeButton->sigClicked().connect([&](){ onResumeActivated(); });

    self->addButton(QIcon(":/Base/icons/refresh.png"), _("Refresh state at the current time"))
        ->sigClicked().connect([&](){ onRefreshButtonClicked(); });
    
    timeSpin = new DoubleSpinBox();
    timeSpin->setAlignment(Qt::AlignCenter);
    timeSpin->sigValueChanged().connect([&](double value){ onTimeSpinChanged(value); });
    self->addWidget(timeSpin);

    timeSlider = new Slider(Qt::Horizontal);
    timeSlider->sigValueChanged().connect([&](int value){ onTimeSliderValueChanged(value); });
    timeSlider->setMinimumWidth(timeSlider->sizeHint().width());
    self->addWidget(timeSlider);

    minTimeSpin = new DoubleSpinBox();
    minTimeSpin->setAlignment(Qt::AlignCenter);
    minTimeSpin->setRange(-9999.0, 9999.0);
    minTimeSpin->sigValueChanged().connect([&](double){ onTimeRangeSpinsChanged(); });
    self->addWidget(minTimeSpin);

    self->addLabel(" : ");

    maxTimeSpin = new DoubleSpinBox();
    maxTimeSpin->setAlignment(Qt::AlignCenter);
    maxTimeSpin->setRange(-9999.0, 9999.0);
    maxTimeSpin->sigValueChanged().connect([&](double){ onTimeRangeSpinsChanged(); });
    self->addWidget(maxTimeSpin);

    self->addButton(QIcon(":/Base/icons/setup.png"), _("Show the config dialog"))
        ->sigClicked().connect([&](){ config.show(); });

    config.frameRateSpin.sigValueChanged().connect([&](int value){ onFrameRateSpinChanged(value); });
    config.playbackFrameRateSpin.sigValueChanged().connect([&](int value){ onPlaybackFrameRateChanged(value); });
    config.playbackSpeedScaleSpin.sigValueChanged().connect([&](double value){ onPlaybackSpeedScaleChanged(value); });

    playbackSpeedScale = config.playbackSpeedScaleSpin.value();
    playbackFrameRate = config.playbackFrameRateSpin.value();

    updateTimeProperties(true);
}


TimeBar::~TimeBar()
{
    delete impl;
}


TimeBarImpl::~TimeBarImpl()
{

}


SignalProxy<bool(double time), LogicalProduct> TimeBar::sigPlaybackInitialized()
{
    return impl->sigPlaybackInitialized;
}


SignalProxy<void(double time)> TimeBar::sigPlaybackStarted()
{
    return impl->sigPlaybackStarted;
}


/**
   Signal emitted when the TimeBar's time changes.
   
   In the function connected to this signal, please return true if the time is valid for it,
   and return false if the time is not valid. The example of the latter case is that
   the time is over the length of the data processed in the function.
*/
SignalProxy<bool(double time), LogicalSum> TimeBar::sigTimeChanged()
{
    return impl->sigTimeChanged;
}


SignalProxy<void(double time, bool isStoppedManually)> TimeBar::sigPlaybackStopped()
{
    return impl->sigPlaybackStopped;
}


bool TimeBar::setTime(double time)
{
    return impl->setTime(time, false);
}


/**
   @todo check whether block() and unblock() of sigc::connection
   decrease the performance or not.
*/
bool TimeBarImpl::setTime(double time, bool calledFromPlaybackLoop, QWidget* callerWidget)
{
    if(TRACE_FUNCTIONS){
        cout << "TimeBarImpl::setTime(" << time << ", " << calledFromPlaybackLoop << ")" << endl;
    }
    
    if(!calledFromPlaybackLoop && isDoingPlayback){
        return false;
    }

    /*
    double newTime; 
    if(isFillLevelActive && calledFromPlaybackLoop){
        newTime = floor(time * self->frameRate_) / self->frameRate_;
    } else {
        newTime = nearbyint(time * self->frameRate_) / self->frameRate_;
    }
    */
    const double newTime = floor(time * self->frameRate_) / self->frameRate_;

    // When the optimization is enabled,
    // the result of (newTime == self->time_) sometimes becomes false,
    // so here the following judgement is used.
    if(fabs(newTime - self->time_) < 1.0e-14){
        if(calledFromPlaybackLoop){
            return true;
        }
        if(callerWidget){
            return false;
        }
    }

    if(newTime > maxTime && config.autoExpandCheck.isChecked()){
        maxTime = newTime;
        timeSpin->blockSignals(true);
        timeSlider->blockSignals(true);
        maxTimeSpin->blockSignals(true);
        timeSpin->setRange(minTime, maxTime);
        const double r = pow(10.0, decimals);
        timeSlider->setRange((int)nearbyint(minTime * r), (int)nearbyint(maxTime * r));
        maxTimeSpin->setValue(maxTime);
        maxTimeSpin->blockSignals(false);
        timeSlider->blockSignals(false);
        timeSpin->blockSignals(false);
    }
        
    self->time_ = newTime;

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

    return sigTimeChanged(self->time_);
}


void TimeBarImpl::onTimeSpinChanged(double value)
{
    if(TRACE_FUNCTIONS){
        cout << "TimeBarImpl::onTimeSpinChanged()" << endl;
    }
    if(isDoingPlayback){
        stopPlayback(true);
    }
    setTime(value, false, timeSpin);
}


bool TimeBarImpl::onTimeSliderValueChanged(int value)
{
    if(TRACE_FUNCTIONS){
        cout << "TimeBarImpl::onTimeSliderChanged(): value = " << value << endl;
    }
    if(isDoingPlayback){
        stopPlayback(true);
    }
    setTime(value / pow(10.0, decimals), false, timeSlider);
    return true;
}


void TimeBar::setFrameRate(double rate)
{
    impl->setFrameRate(rate);
}


void TimeBarImpl::setFrameRate(double rate)
{
    if(rate > 0.0){
        if(self->frameRate_ != rate){
            self->frameRate_ = rate;
            updateTimeProperties(true);
        }
    }
}


double TimeBar::minTime() const
{
    return impl->minTime;
}


double TimeBar::maxTime() const
{
    return impl->maxTime;
}


void TimeBar::setTimeRange(double min, double max)
{
    impl->setTimeRange(min, max);
}


void TimeBarImpl::setTimeRange(double minTime, double maxTime)
{
    this->minTime = minTime;
    this->maxTime = maxTime;
    updateTimeProperties(false);
}


void TimeBarImpl::updateTimeProperties(bool forceUpdate)
{
    timeSpin->blockSignals(true);
    timeSlider->blockSignals(true);
    minTimeSpin->blockSignals(true);
    maxTimeSpin->blockSignals(true);
    config.frameRateSpin.blockSignals(true);
    
    const double timeStep = 1.0 / self->frameRate_;
    decimals = static_cast<int>(ceil(log10(self->frameRate_)));
    const double r = pow(10.0, decimals);

    if(forceUpdate ||
       (minTime != timeSpin->minimum() || maxTime != timeSpin->maximum())){
        timeSpin->setRange(minTime, maxTime);
        timeSlider->setRange((int)nearbyint(minTime * r), (int)nearbyint(maxTime * r));
    }

    timeSpin->setDecimals(decimals);
    timeSpin->setSingleStep(timeStep);
    timeSlider->setSingleStep(timeStep * r);
    minTimeSpin->setValue(minTime);
    maxTimeSpin->setValue(maxTime);
    config.frameRateSpin.setValue(self->frameRate_);

    config.frameRateSpin.blockSignals(false);
    maxTimeSpin->blockSignals(false);
    minTimeSpin->blockSignals(false);
    timeSlider->blockSignals(false);
    timeSpin->blockSignals(false);

    setTime(self->time_, false);
}

    
void TimeBarImpl::onPlaybackSpeedScaleChanged(double value)
{
    playbackSpeedScale = value;
    
    if(isDoingPlayback){
        startPlayback();
    }
}


double TimeBar::playbackSpeedScale() const
{
    return impl->config.playbackSpeedScaleSpin.value();
}


void TimeBar::setPlaybackSpeedScale(double scale)
{
    impl->config.playbackSpeedScaleSpin.setValue(scale);
}


void TimeBarImpl::onPlaybackFrameRateChanged(int value)
{
    playbackFrameRate = value;

    if(isDoingPlayback){
        startPlayback();
    }
}


double TimeBar::playbackFrameRate() const
{
    return impl->config.playbackFrameRateSpin.value();
}


void TimeBar::setPlaybackFrameRate(double rate)
{
    impl->config.playbackFrameRateSpin.setValue(rate);
}


void TimeBar::setRepeatMode(bool on)
{
    impl->repeatMode = on;
}


void TimeBarImpl::onPlayActivated()
{
    stopPlayback(true);
    setTime(minTime, false);
    startPlayback();
}


void TimeBarImpl::onResumeActivated()
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
    impl->startPlayback();
}


void TimeBarImpl::startPlayback()
{
    stopPlayback(false);
    
    animationTimeOffset = self->time_;

    if(sigPlaybackInitialized(self->time_)){

        sigPlaybackStarted(self->time_);

        if(!setTime(self->time_, true)){
            sigPlaybackStopped(self->time_, false);

        } else {
            isDoingPlayback = true;

            const static QString tip(_("Stop animation"));
            stopResumeButton->setIcon(stopIcon);
            stopResumeButton->setToolTip(tip);
            int interval;
            if(config.idleLoopDrivenCheck.isChecked()){
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


void TimeBarImpl::stopPlayback(bool isStoppedManually)
{
    if(isDoingPlayback){
        killTimer(timerId);
        isDoingPlayback = false;
        sigPlaybackStopped(self->time_, isStoppedManually);

        const static QString tip(_("Resume animation"));
        stopResumeButton->setIcon(resumeIcon);
        stopResumeButton->setToolTip(tip);

        if(fillLevelMap.empty()){
            isFillLevelActive = false;
        }
    }
}


bool TimeBar::isDoingPlayback()
{
    return impl->isDoingPlayback;
}


int TimeBar::startFillLevelUpdate()
{
    return impl->startFillLevelUpdate();    
}


int TimeBarImpl::startFillLevelUpdate()
{
    int id=0;
    if(fillLevelMap.empty()){
        isFillLevelActive = true;
    } else {
        while(fillLevelMap.find(id) == fillLevelMap.end()){
            ++id;
        }
    }
    updateFillLevel(id, 0.0);
    return id;
}



void TimeBar::updateFillLevel(int id, double time)
{
    impl->updateFillLevel(id, time);
}


void TimeBarImpl::updateFillLevel(int id, double time)
{
    fillLevelMap[id] = time;
    updateMinFillLevel();
}


void TimeBarImpl::updateMinFillLevel()
{
    double minFillLevel = std::numeric_limits<double>::max();
    map<int,double>::iterator p;
    for(p = fillLevelMap.begin(); p != fillLevelMap.end(); ++p){
        minFillLevel = std::min(p->second, minFillLevel);
    }
    fillLevel = minFillLevel;
}    


void TimeBar::stopFillLevelUpdate(int id)
{
    impl->stopFillLevelUpdate(id);
}


void TimeBarImpl::stopFillLevelUpdate(int id)
{
    fillLevelMap.erase(id);

    if(!fillLevelMap.empty()){
        updateMinFillLevel();
    } else {
        if(!isDoingPlayback){
            isFillLevelActive = false;
        }
    }
}


void TimeBar::setFillLevelSync(bool on)
{
    impl->config.fillLevelSyncCheck.setChecked(on);
}


void TimeBar::startPlaybackFromFillLevel()
{
    if(isDoingPlayback()){
        stopPlayback();
    }
    setTime(impl->fillLevel);
    startPlayback();
}


double TimeBar::realPlaybackTime() const
{
    if(impl->isDoingPlayback){
        return impl->animationTimeOffset + impl->playbackSpeedScale * (impl->elapsedTimer.elapsed() / 1000.0);
    } else {
        return time_;
    }
}


void TimeBarImpl::timerEvent(QTimerEvent*)
{
    double time = animationTimeOffset + playbackSpeedScale * (elapsedTimer.elapsed() / 1000.0);

    bool doStopAtLastFillLevel = false;
    if(isFillLevelActive){
        if(config.fillLevelSyncCheck.isChecked() || (time > fillLevel)){
            animationTimeOffset += (fillLevel - time);
            time = fillLevel;
            if(fillLevelMap.empty()){
                doStopAtLastFillLevel = true;
            }
        }
    }

    if(!setTime(time, true) || doStopAtLastFillLevel){
        stopPlayback(false);
        
        if(!doStopAtLastFillLevel && repeatMode){
            setTime(minTime, true);
            startPlayback();
        }
    }
}


void TimeBarImpl::onTimeRangeSpinsChanged()
{
    setTimeRange(minTimeSpin->value(), maxTimeSpin->value());
}


void TimeBarImpl::onFrameRateSpinChanged(int value)
{
    setFrameRate(config.frameRateSpin.value());
}


void TimeBarImpl::onRefreshButtonClicked()
{
    if(!isDoingPlayback){
        sigTimeChanged(self->time_);
    }
}


int TimeBar::stretchableDefaultWidth() const
{
    return sizeHint().width() + impl->timeSlider->sizeHint().width() * 5;
}
    

bool TimeBar::storeState(Archive& archive)
{
    return impl->storeState(archive);
}


bool TimeBarImpl::storeState(Archive& archive)
{
    archive.write("minTime", minTime);
    archive.write("maxTime", maxTime);
    archive.write("frameRate", self->frameRate_);
    archive.write("playbackFrameRate", playbackFrameRate);
    archive.write("idleLoopDrivenMode", config.idleLoopDrivenCheck.isChecked());
    archive.write("currentTime", self->time_);
    archive.write("speedScale", playbackSpeedScale);
    archive.write("syncToOngoingUpdates", config.fillLevelSyncCheck.isChecked());
    archive.write("autoExpansion", config.autoExpandCheck.isChecked());
    return true;
}


bool TimeBar::restoreState(const Archive& archive)
{
    return impl->restoreState(archive);
}


bool TimeBarImpl::restoreState(const Archive& archive)
{
    archive.read("minTime", minTime);
    archive.read("maxTime", maxTime);
    archive.read("currentTime", self->time_);

    config.playbackFrameRateSpin.setValue(archive.get("playbackFrameRate", playbackFrameRate));
    config.idleLoopDrivenCheck.setChecked(archive.get("idleLoopDrivenMode", config.idleLoopDrivenCheck.isChecked()));
    config.playbackSpeedScaleSpin.setValue(archive.get("speedScale", playbackSpeedScale));
    config.fillLevelSyncCheck.setChecked(archive.get("syncToOngoingUpdates", config.fillLevelSyncCheck.isChecked()));
    config.autoExpandCheck.setChecked(archive.get("autoExpansion", config.autoExpandCheck.isChecked()));

    double prevFrameRate = self->frameRate_;
    archive.read("frameRate", self->frameRate_);

    updateTimeProperties(self->frameRate_ != prevFrameRate);
    
    return true;
}
