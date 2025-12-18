#ifndef CNOID_BASE_CHECK_BOX_H
#define CNOID_BASE_CHECK_BOX_H

#include <cnoid/Signal>
#include <QCheckBox>
#include <optional>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT CheckBox : public QCheckBox
{
public:
    CheckBox(QWidget* parent = nullptr);
    CheckBox(const QString& text, QWidget* parent = nullptr);
                               
    void setUserInputEnabled(bool on) { isUserInputEnabled_ = on; }
    bool isUserInputEnabled() const { return isUserInputEnabled_; }

    SignalProxy<void(int)> sigStateChanged();
    SignalProxy<void(bool)> sigToggled();

protected:
    virtual void keyPressEvent(QKeyEvent* event) override;
    virtual void mousePressEvent(QMouseEvent* event) override;

private:
    std::optional<Signal<void(int)>> sigStateChanged_;
    std::optional<Signal<void(bool)>> sigToggled_;
    bool isUserInputEnabled_;
};

}

#endif
