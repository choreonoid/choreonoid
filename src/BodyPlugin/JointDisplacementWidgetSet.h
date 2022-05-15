#ifndef CNOID_BODY_PLUGIN_JOINT_DISPLACEMENT_WIDGET_H
#define CNOID_BODY_PLUGIN_JOINT_DISPLACEMENT_WIDGET_H

#include <QWidget>
#include <QGridLayout>
#include <cnoid/Signal>
#include "exportdecl.h"

namespace cnoid {

class BodyItem;
class MenuManager;
class Archive;

class CNOID_EXPORT JointDisplacementWidgetSet
{
public:
    JointDisplacementWidgetSet(
        QWidget* baseWidget, QGridLayout* sharedGrid = nullptr, int* sharedRowCounter = nullptr);
    ~JointDisplacementWidgetSet();

    void setBodyItem(BodyItem* bodyItem);
    BodyItem* bodyItem();

    void setTargetBodyLabelEnabled(bool on);
    void setVisible(bool on);
    void setOptionMenuTo(MenuManager& menu);

    bool storeState(Archive* archive);
    bool restoreState(const Archive* archive);

    SignalProxy<void(QWidget* widget)> sigJointWidgetFocused();

    class Impl;

private:
    Impl* impl;
};

}

#endif
