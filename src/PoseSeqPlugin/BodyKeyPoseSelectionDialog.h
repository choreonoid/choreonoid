#ifndef CNOID_POSE_SEQ_PLUGIN_BODY_KEY_POSE_SELECTION_DIALOG_H
#define CNOID_POSE_SEQ_PLUGIN_BODY_KEY_POSE_SELECTION_DIALOG_H

#include "PoseSeqItem.h"
#include <cnoid/Dialog>
#include <cnoid/Buttons>
#include <cnoid/DoubleSpinBox>
#include <functional>

namespace cnoid {

class LinkDeviceTreeWidget;

class BodyKeyPoseSelectionDialog : public Dialog
{
public:
    BodyKeyPoseSelectionDialog(
        LinkDeviceTreeWidget* linkTreeWidget,
        std::function<PoseSeqItem*()> targetPoseSeqItemFunc);

    void storeState(Mapping* archive);
    void restoreState(const Mapping* archive);

private:
    LinkDeviceTreeWidget* linkTreeWidget;
    std::function<PoseSeqItem*()> targetPoseSeqItemFunc;
    
    DoubleSpinBox startTimeSpin;
    DoubleSpinBox endTimeSpin;
    RadioButton allKeyPosesRadio;
    RadioButton includeSelectedPartsRadio;
    RadioButton matchSelectedPartsRadio;

    void clearSelection();
    void doSelection();
};

}

#endif
