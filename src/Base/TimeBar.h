/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_GUIBASE_TIME_BAR_H_INCLUDED
#define CNOID_GUIBASE_TIME_BAR_H_INCLUDED

#include <cnoid/ToolBar>
#include "exportdecl.h"

namespace cnoid {

class ExtensionManager;
class TimeBarImpl;

class CNOID_EXPORT TimeBar : public ToolBar
{
    class LogicalSum
    {
    public:
        typedef bool result_type;
        template<typename InputIterator>
        bool operator()(InputIterator first, InputIterator last) const {
            bool result = false;
            while(first != last){
                result |= *first++;
            }
            return result;
        }
    };

    class LogicalProduct
    {
    public:
        typedef bool result_type;
        template<typename InputIterator>
        bool operator()(InputIterator first, InputIterator last) const {
            bool result = true;
            while(first != last){
                result &= *first++;
            }
            return result;
        }
    };
        
public:

    static void initialize(ExtensionManager* ext);
    static TimeBar* instance();

    /**
       \note If any connected slot returns false, the playback is canceled.
    */
    SignalProxy< boost::signal<bool(double time), LogicalProduct> > sigPlaybackInitialized();
        
    SignalProxy< boost::signal<void(double time)> > sigPlaybackStarted();
    SignalProxy< boost::signal<bool(double time), LogicalSum> > sigTimeChanged();
    SignalProxy< boost::signal<void(double time, bool isStoppedManually)> > sigPlaybackStopped();

    inline double time() const { return time_; }
        
    bool setTime(double time);

    double realPlaybackTime() const;

    double minTime() const;
    double maxTime() const;
        
    void setTimeRange(double min, double max);
        
    inline double frameRate() const { return frameRate_; }
    void setFrameRate(double rate);

    inline double timeStep() const { return 1.0 / frameRate_; }

    inline bool isBeatMode() const { return isBeatMode_; }
    inline double beatOffset() const { return beatOffset_; }
    inline double tempo() const { return tempo_; }
    double timeOfBeatLocation(double beatLocation) const;
    double beatLocationOfTime(double time) const;
    inline int beatNumerator() const { return beatNumerator_; }
    inline int beatDenominator() const { return beatDenominator_; }

    double playbackSpeedScale() const;
    void setPlaybackSpeedScale(double scale);
        
    double playbackFrameRate() const;
    void setPlaybackFrameRate(double rate);

    void setRepeatMode(bool on);
	
    void startPlayback();
    void startPlaybackFromFillLevel();
    void stopPlayback(bool isStoppedManually = false);
    bool isDoingPlayback();

    int startFillLevelUpdate();
    void updateFillLevel(int id, double time);
    void stopFillLevelUpdate(int id);
    void setFillLevelSync(bool on);

    virtual QSize minimumSizeHint () const;
    virtual QSize sizeHint () const;

protected:

    virtual bool storeState(Archive& archive);
    virtual bool restoreState(const Archive& archive);
        
private:
    TimeBar();
    virtual ~TimeBar();
        
    TimeBarImpl* impl;
    double time_;
    double frameRate_;
    bool isBeatMode_;
    double beatOffset_;
    double tempo_;
    int beatNumerator_;
    int beatDenominator_;

    friend class TimeBarImpl;
};
}

#endif
