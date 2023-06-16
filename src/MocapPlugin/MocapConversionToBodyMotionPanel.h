#ifndef CNOID_MOCAP_PLUGIN_MOCAP_CONVERSION_TO_BODY_MOTION_PANEL_H
#define CNOID_MOCAP_PLUGIN_MOCAP_CONVERSION_TO_BODY_MOTION_PANEL_H

#include <QWidget>
#include <functional>
#include "exportdecl.h"

namespace cnoid {

class MarkerMotion;
class SkeletonMotion;
class Body;
class BodyMotion;
class Mapping;

/**
   \note The object using a panel must check the destroyed signal of the panel and
   invalidate the panel when the signal is emitted.
*/
class CNOID_EXPORT MocapConversionToBodyMotionPanel : public QWidget
{
public:
    template<class PanelType>
    static void registerConversionPanel(const char* name, const char* caption)
    {
        registerConversionPanel_(name, caption, [](){ return new PanelType; });
    }
    static void unregisterConversionPanel(const char* name);
    static int numConversionPanels();
    static std::string getConversionPanelName(int index);
    static std::string getConversionPanelCaption(int index);
    static MocapConversionToBodyMotionPanel* getConversionPanel(int index);
    static MocapConversionToBodyMotionPanel* getOrCreateConversionPanel(int index);

    virtual bool convert(MarkerMotion& markerMotion, Body* body, BodyMotion& bodyMotion);
    virtual bool convert(SkeletonMotion& skeletonMotion, Body* body, BodyMotion& bodyMotion);
    virtual void storeConfigurations(Mapping* conf);
    virtual void restoreConfigurations(const Mapping* conf);
    
private:
    static void registerConversionPanel_(
        const char* name, const char* caption,
        std::function<MocapConversionToBodyMotionPanel*()> factory);
};

}

#endif
