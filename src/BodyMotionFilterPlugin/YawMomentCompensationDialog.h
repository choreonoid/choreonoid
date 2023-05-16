#ifndef CNOID_BODY_MOTION_FILTER_PLUGIN_YAW_MOMENT_COMPENSATION_DIALOG_H
#define CNOID_BODY_MOTION_FILTER_PLUGIN_YAW_MOMENT_COMPENSATION_DIALOG_H

#include <cnoid/Dialog>

namespace cnoid {

class Plugin;
class BodyItem;
class BodyMotionItem;
class Archive;

class YawMomentCompensationDialog : public Dialog
{
public:
    static void initialize(Plugin* plugin);
    static YawMomentCompensationDialog* instance();

    YawMomentCompensationDialog();
    ~YawMomentCompensationDialog();

    bool applyFilter(BodyItem* bodyItem, BodyMotionItem* motionItem);

protected:
    virtual void onAccepted() override;

private:
    class Impl;
    Impl* impl;
};

}

#endif
