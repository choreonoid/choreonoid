#ifndef CNOID_BODY_PLUGIN_LINK_POSITION_WIDGET_H
#define CNOID_BODY_PLUGIN_LINK_POSITION_WIDGET_H

#include <QWidget>
#include <functional>
#include <string>
#include <utility>
#include "exportdecl.h"

namespace cnoid {

class BodyItem;
class Link;
class LinkKinematicsKit;
class CoordinateFrame;
class MenuManager;
class Archive;

class CNOID_EXPORT LinkPositionWidget : public QWidget
{
public:
    LinkPositionWidget(QWidget* parent);
    ~LinkPositionWidget();

    void customizeCoordinateModeLabels(
        const char* worldModeLabel, const char* modelModeLabel, const char* localModeLabel);

    typedef std::function<std::string(LinkKinematicsKit* kit, CoordinateFrame* frame, bool isDefaultFrame)>
        FrameLabelFunction;
    void customizeBaseFrameLabels(const char* caption, FrameLabelFunction labelFunction);
    void customizeOffsetFrameLabels(const char* caption, FrameLabelFunction labelFunction);

    enum TargetLinkType { AnyLink, RootOrIkLink, IkLink, NumTargetLinkTypes };
    void setTargetLinkType(int type);
    int targetLinkType() const;

    void setTargetBodyAndLink(BodyItem* bodyItem, Link* link);
    BodyItem* targetBodyItem();
    Link* targetLink();

    void setCustomIkEnabled(bool on);
    bool isCustomIkEnabled() const;

    void setOptionMenuTo(MenuManager& menuManager);

    bool storeState(Archive& archive);
    bool restoreState(const Archive& archive);

    class Impl;

private:
    Impl* impl;
};

}

#endif
