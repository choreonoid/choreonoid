#ifndef CNOID_BASE_COMBO_BOX_H
#define CNOID_BASE_COMBO_BOX_H

#include <cnoid/Signal>
#include <QComboBox>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT ComboBox : public QComboBox
{
public:
    ComboBox(QWidget* parent = nullptr);
    ~ComboBox();

    void enableI18n(const char* domainname);
    void addI18nItem(const char* text);
    void addI18nItem(const QIcon & icon, const char* text);

    void setUserInputEnabled(bool on) { isUserInputEnabled_ = on; }
    bool isUserInputEnabled() const { return isUserInputEnabled_; }

    QString currentOrgText() const;
    int findOrgText(const std::string& text, bool setFoundItemCurrent = false);
    virtual void showPopup() override;

    SignalProxy<void(int)> sigActivated();
    SignalProxy<void(int)> sigCurrentIndexChanged();
    SignalProxy<void(const QString&)> sigEditTextChanged();
    SignalProxy<void(int)> sigHighlighted();
    SignalProxy<void()> sigAboutToShowPopup();

protected:
    virtual void keyPressEvent(QKeyEvent* event) override;
    virtual void mousePressEvent(QMouseEvent* event) override;
    virtual void wheelEvent(QWheelEvent* event) override;

private:
    class Impl;
    Impl* impl;
    bool isUserInputEnabled_;
};

}

#endif
