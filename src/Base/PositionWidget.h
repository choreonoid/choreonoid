#ifndef CNOID_BASE_POSITION_WIDGET_H
#define CNOID_BASE_POSITION_WIDGET_H

#include <cnoid/EigenTypes>
#include <QWidget>
#include <functional>
#include "exportdecl.h"

namespace cnoid {

class MenuManager;
class Archive;

class CNOID_EXPORT PositionWidget : public QWidget
{
public:
    PositionWidget(QWidget* parent);
    ~PositionWidget();

    void setOptionMenuTo(MenuManager& menuManager);
    void setEditable(bool on);
    void setUserInputValuePriorityMode(bool on);
    void setCallbacks(
        std::function<bool(const Isometry3& T)> callbackOnPositionInput,
        std::function<void()> callbackOnPositionInputFinished);
    [[deprecated("Use setCallbacks")]]
    void setPositionCallback(std::function<bool(const Isometry3& T)> callback);
    void clearPosition();
    void refreshPosition();
    void applyPositionInput();
    Vector3 getRpyInput() const;
    void setReferenceRpy(const Vector3& rpy);
    void setPosition(const Isometry3& T);
    void setErrorHighlight(bool on);
    void storeState(Archive& archive);
    void restoreState(const Archive& archive);

private:
    class Impl;
    Impl* impl;
};

}

#endif
