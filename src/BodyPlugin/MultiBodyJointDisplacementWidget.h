#ifndef CNOID_BODY_PLUGIN_MULTI_BODY_JOINT_DISPLACEMENT_WIDGET_H
#define CNOID_BODY_PLUGIN_MULTI_BODY_JOINT_DISPLACEMENT_WIDGET_H

#include <QWidget>
#include "exportdecl.h"

namespace cnoid {

class KinematicBodyItemSet;

class CNOID_EXPORT MultiBodyJointDisplacementWidget : public QWidget
{
public:
    MultiBodyJointDisplacementWidget(QWidget* parent);
    ~MultiBodyJointDisplacementWidget();

    void setKinematicBodyItemSet(KinematicBodyItemSet* bodyItemSet);
    KinematicBodyItemSet* kinematicBodyItemSet();

private:
    class Impl;
    Impl* impl;
};

}

#endif
