#ifndef CNOID_BASE_RADIO_BUTTON_H
#define CNOID_BASE_RADIO_BUTTON_H

#include <cnoid/Signal>
#include <QRadioButton>
#include <cnoid/stdx/optional>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT RadioButton : public QRadioButton
{
public:
    RadioButton(QWidget* parent = nullptr);
    RadioButton(const QString & text, QWidget* parent = nullptr);

    void setUserInputEnabled(bool on) { isUserInputEnabled_ = on; }
    bool isUserInputEnabled() const { return isUserInputEnabled_; }

    SignalProxy<void(bool checked)> sigToggled();

protected:
    virtual void keyPressEvent(QKeyEvent* event) override;
    virtual void mousePressEvent(QMouseEvent* event) override;

private:
    stdx::optional<Signal<void(bool)>> sigToggled_;
    bool isUserInputEnabled_;
};

}

#endif
