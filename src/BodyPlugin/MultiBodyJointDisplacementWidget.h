#ifndef CNOID_BODY_PLUGIN_MULTI_BODY_JOINT_DISPLACEMENT_WIDGET_H
#define CNOID_BODY_PLUGIN_MULTI_BODY_JOINT_DISPLACEMENT_WIDGET_H

#include <QWidget>
#include "exportdecl.h"

namespace cnoid {

class BodyItem;
class KinematicBodyItemSet;

class CNOID_EXPORT MultiBodyJointDisplacementWidget : public QWidget
{
public:
    MultiBodyJointDisplacementWidget(QWidget* parent);
    ~MultiBodyJointDisplacementWidget();

    enum LabelOption { PlainLabel = 0, BoldLabel = 1, BracketedLabel = 2 };
    void setTargetBodyLabelOptions(int options);

    void setPersistentBodyItem(BodyItem* bodyItem);
    void setKinematicBodyItemSet(KinematicBodyItemSet* bodyItemSet);
    KinematicBodyItemSet* kinematicBodyItemSet();

private:
    class Impl;
    Impl* impl;
};

}

#endif
