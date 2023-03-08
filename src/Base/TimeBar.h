#ifndef CNOID_BASE_TIME_BAR_H
#define CNOID_BASE_TIME_BAR_H

#include <cnoid/ToolBar>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT TimeBar : public ToolBar
{
public:
    // The following can be called before the initializeClass function is called to customize TimeBar.
    static void setNegativeTimeEnabled(bool on);
        
    static void initialize(ExtensionManager* ext);
    static TimeBar* instance();

    ~TimeBar();

    /**
       The return value from each slot indicates whether the playback is successfully initialized or not.
       If any slot returns false, starting the playback will be canceled.
    */
    SignalProxy<bool(double time)> sigPlaybackInitialized();

    SignalProxy<void(double time)> sigPlaybackStarted();

    //! The return value from each slot indicates whether a valid data exists for the specified time or not.
    SignalProxy<bool(double time)> sigTimeChanged();
    
    SignalProxy<void(double time, bool isStoppedManually)> sigPlaybackStopped();

    //! The return value from each slot is the time of the last valid frame when the playback is stopped.
    SignalProxy<double(double time, bool isStoppedManually)> sigPlaybackStoppedEx();
    
    inline double time() const { return time_; }
        
    bool setTime(double time);
    void refresh();

    double realPlaybackTime() const;

    double minTime() const;
    double maxTime() const;
    void setTimeRange(double minTime, double maxTime);
        
    inline double frameRate() const { return frameRate_; }
    void setFrameRate(double rate);

    inline double timeStep() const { return 1.0 / frameRate_; }

    double playbackFrameRate() const;
    void setPlaybackFrameRate(double rate);

    bool isIdleEventDrivenMode() const;
    void setIdleEventDrivenMode(bool on);

    double playbackSpeedRatio() const;
    void setPlaybackSpeedRatio(double ratio);

    void setRepeatMode(bool on);
	
    void startPlayback();
    void startPlayback(double time);
    void stopPlayback(bool isStoppedManually = false);
    bool isDoingPlayback();

    bool isOngoingTimeSyncEnabled() const;
    void setOngoingTimeSyncEnabled(bool on);
    int startOngoingTimeUpdate(double time = 0.0);
    void updateOngoingTime(int id, double time);
    void stopOngoingTimeUpdate(int id);

    [[deprecated("Use playbackSpeedRatio")]]
    double playbackSpeedScale() const { return playbackSpeedRatio(); }
    [[deprecated("Use setPlaybackSpeedRatio")]]
    void setPlaybackSpeedScale(double scale) { setPlaybackSpeedRatio(scale); }
        
    [[deprecated]]
    void startPlaybackFromFillLevel() { startPlayback(); }
    [[deprecated]]
    int startFillLevelUpdate(double time = 0.0) { return startOngoingTimeUpdate(time); }
    [[deprecated]]
    void updateFillLevel(int id, double time) { updateOngoingTime(id, time); }
    [[deprecated]]
    void stopFillLevelUpdate(int id) { stopOngoingTimeUpdate(id); }
    [[deprecated]]
    void setFillLevelSync(bool on) { setOngoingTimeSyncEnabled(on); }

    virtual int stretchableDefaultWidth() const override;

    class Impl;

protected:
    virtual void onActiveElementUpdated() override;
    virtual bool storeState(Archive& archive) override;
    virtual bool restoreState(const Archive& archive) override;
        
private:
    TimeBar();

    Impl* impl;
    double time_;
    double frameRate_;
};

}

#endif
