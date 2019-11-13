/*! @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODYPLUGIN_POSITION_VIEW_H
#define CNOID_BODYPLUGIN_POSITION_VIEW_H

#include <cnoid/View>
#include <functional>
#include <string>
#include <tuple>
#include "exportdecl.h"

namespace cnoid {

class LinkKinematicsKit;

class CNOID_EXPORT PositionView : public View
{
public:
    static void initializeClass(ExtensionManager* ext);
    static PositionView* instance();

    PositionView();
    virtual ~PositionView();

    void setCoordinateModeLabels(
        const char* worldModeLabel, const char* modelModeLabel, const char* localModeLabel);

    void setCoordinateOffsetLabels(const char* baseOffsetLabel, const char* endOffsetLabel);

    void customizeDefaultCoordinateFrameNames(
        std::function<std::tuple<std::string,std::string,std::string>(LinkKinematicsKit*)> getNames);

protected:
    virtual void onActivated() override;
    virtual void onDeactivated() override;
    virtual bool storeState(Archive& archive) override;
    virtual bool restoreState(const Archive& archive) override;

private:
    class Impl;
    Impl* impl;
};

}

#endif
