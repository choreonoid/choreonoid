#ifndef CNOID_BASE_PUSH_BUTTON_H
#define CNOID_BASE_PUSH_BUTTON_H

#include <cnoid/Signal>
#include <QPushButton>
#include <cnoid/stdx/optional>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT PushButton : public QPushButton
{
public:
    PushButton(QWidget* parent = nullptr);
    PushButton(const QString& text, QWidget* parent = nullptr);
    PushButton(const QIcon& icon, const QString& text, QWidget* parent = nullptr);

    void setUserInputEnabled(bool on) { isUserInputEnabled_ = on; }
    bool isUserInputEnabled() const { return isUserInputEnabled_; }

    SignalProxy<void()> sigClicked();
    SignalProxy<void(bool)> sigToggled();

protected:
    virtual void keyPressEvent(QKeyEvent* event) override;
    virtual void mousePressEvent(QMouseEvent* event) override;

private:
    stdx::optional<Signal<void()>> sigClicked_;
    stdx::optional<Signal<void(bool)>> sigToggled_;
    bool isUserInputEnabled_;
};

}

#endif
