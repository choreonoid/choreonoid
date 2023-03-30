#ifndef CNOID_BASE_TOOL_BUTTON_H
#define CNOID_BASE_TOOL_BUTTON_H

#include <cnoid/Signal>
#include <QToolButton>
#include <cnoid/stdx/optional>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT ToolButton : public QToolButton
{
public:
    ToolButton(QWidget* parent = nullptr);
    ToolButton(const QString& text, QWidget* parent = nullptr);
    
    void setUserInputEnabled(bool on) { isUserInputEnabled_ = on; }
    bool isUserInputEnabled() const { return isUserInputEnabled_; }

    SignalProxy<void()> sigClicked();
    SignalProxy<void(bool)> sigToggled();
    SignalProxy<void()> sigPressed();
    SignalProxy<void()> sigReleased();

protected:
    virtual void keyPressEvent(QKeyEvent* event) override;
    virtual void mousePressEvent(QMouseEvent* event) override;

private:
    stdx::optional<Signal<void()>> sigClicked_;
    stdx::optional<Signal<void(bool)>> sigToggled_;
    stdx::optional<Signal<void()>> sigPressed_;
    stdx::optional<Signal<void()>> sigReleased_;
    bool isUserInputEnabled_;
};

}

#endif
