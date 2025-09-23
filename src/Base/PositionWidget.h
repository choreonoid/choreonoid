#ifndef CNOID_BASE_POSITION_WIDGET_H
#define CNOID_BASE_POSITION_WIDGET_H

#include <cnoid/Signal>
#include <cnoid/EigenTypes>
#include <QWidget>
#include <functional>
#include "exportdecl.h"

namespace cnoid {

class DisplayValueFormat;
class MenuManager;
class Archive;

class CNOID_EXPORT PositionWidget : public QWidget
{
public:
    PositionWidget(QWidget* parent = nullptr);
    PositionWidget(DisplayValueFormat* format, QWidget* parent = nullptr);
    ~PositionWidget();

    void setDisplayValueFormat(DisplayValueFormat* format);
    void setOptionMenuTo(MenuManager& menuManager);
    void setRpyVisible(bool on);
    void setEditable(bool on);
    void setUserInputValuePriorityMode(bool on);
    void setAdditionalPrecisionInterfaceEnabled(bool on);
    int additionalPrecision() const;
    SignalProxy<void(int precision)> sigAdditionalPrecisionChanged();
    void setCallbacks(
        std::function<bool(const Isometry3& T)> callbackOnPositionInput,
        std::function<void()> callbackOnPositionInputFinished = nullptr);
    [[deprecated("Use setCallbacks")]]
    void setPositionCallback(std::function<bool(const Isometry3& T)> callback);
    const Isometry3& currentPosition() const;
    void clearPosition();
    void refreshPosition();
    void applyPositionInput();
    Vector3 getRpyInput() const;
    void setReferenceRpy(const Vector3& rpy);
    void setPosition(const Isometry3& T);
    void setTranslation(const Vector3& p);
    void setRpy(const Vector3& rpy);
    void setErrorHighlight(bool on);
    void storeState(Archive* archive);
    void restoreState(const Archive* archive);

private:
    class Impl;
    Impl* impl;
};

}

#endif
