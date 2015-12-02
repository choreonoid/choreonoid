/**
   @file
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_GROBOT_CONTROLLER_ITEM_H_INCLUDED
#define CNOID_GROBOT_CONTROLLER_ITEM_H_INCLUDED

#include <cnoid/Item>
#include <cnoid/ConnectionSet>

class GRobotController;

namespace cnoid {

class MessageView;
class BodyItem;

class GRobotControllerItem : public Item
{
public:
    GRobotControllerItem();
    GRobotControllerItem(const GRobotControllerItem& org);
    ~GRobotControllerItem();
            
protected:
    virtual Item* doDuplicate() const;
    virtual void onConnectedToRoot();
    virtual void onDisconnectedFromRoot();
    virtual void onPositionChanged();
    virtual void doPutProperties(PutPropertyFunction& putProperty);
    virtual bool store(Archive& archive);
    virtual bool restore(const Archive& archive);

private:
    GRobotController* controller;
    BodyItem* bodyItem;
    ConnectionSet connections;
    Connection checkToggledConnection;
    Connection kinematicStateChangeConnection;
    Connection playbackInitilizeConnection;
    ConnectionSet playbackConnections;
    MessageView* mv;

    void onCheckToggled(bool on);
    void setSyncMode(bool on);
    void requestToSendPose(double transitionTime);
    bool onPlaybackInitialized(double time);
    void onPlaybackStarted(double time);
    bool onPortPropertyChanged(const std::string& port);
    void onPlaybackStopped();
};

typedef ref_ptr<GRobotControllerItem> GRobotControllerItemPtr;
}

#endif
