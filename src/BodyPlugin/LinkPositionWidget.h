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
class AbstractPositionEditTarget;
class MenuManager;
class Archive;

class CNOID_EXPORT LinkPositionWidget : public QWidget
{
public:
    LinkPositionWidget(QWidget* parent);
    ~LinkPositionWidget();

    void setCoordinateModeLabels(
        const char* worldModeLabel, const char* modelModeLabel, const char* localModeLabel);

    void setCoordinateLabels(const char* baseCoordinateLabel, const char* offsetCoordinateLabel);

    void customizeDefaultCoordinateFrameNames(
        std::function<std::pair<std::string,std::string>(LinkKinematicsKit*)> getNames);

    enum TargetLinkType { AnyLink, RootOrIkLink, IkLink, NumTargetLinkTypes };
    void setTargetLinkType(int type);
    int targetLinkType() const;

    void setTargetBodyAndLink(BodyItem* bodyItem, Link* link);
    BodyItem* targetBodyItem();
    Link* targetLink();
    bool setPositionEditTarget(AbstractPositionEditTarget* target);

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
