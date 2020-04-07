#ifndef CNOID_BODY_PLUGIN_JOINT_DISPLACEMENT_WIDGET_H
#define CNOID_BODY_PLUGIN_JOINT_DISPLACEMENT_WIDGET_H

#include <QWidget>
#include <cnoid/Signal>
#include "exportdecl.h"

namespace cnoid {

class BodyItem;
class MenuManager;
class Archive;

class CNOID_EXPORT JointDisplacementWidget : public QWidget
{
public:
    JointDisplacementWidget(QWidget* parent);
    ~JointDisplacementWidget();

    void setBodyItem(BodyItem* bodyItem);
    BodyItem* bodyItem();
    
    void setOptionMenuTo(MenuManager& menu);
    bool storeState(Archive& archive);
    bool restoreState(const Archive& archive);

    SignalProxy<void(QWidget* widget)> sigJointWidgetFocused();

    class Impl;

protected:
    virtual bool eventFilter(QObject* object, QEvent* event) override;
    
private:
    Impl* impl;
};

}

#endif
