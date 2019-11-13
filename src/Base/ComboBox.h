/**
   @author Shin'ichiro NAKAOKA
*/

#ifndef CNOID_BASE_COMBO_BOX_H
#define CNOID_BASE_COMBO_BOX_H

#include <cnoid/Signal>
#include <QComboBox>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT ComboBox : public QComboBox
{
    Q_OBJECT

public:
    ComboBox(QWidget* parent = nullptr);
    ~ComboBox();

    void enableI18n(const char* domainname);
    void addI18nItem(const char* text);
    void addI18nItem(const QIcon & icon, const char* text);
    QString currentOrgText() const;
    int findOrgText(const std::string& text, bool setFoundItemCurrent = false);
    virtual void showPopup() override;

    SignalProxy<void(int)> sigActivated();
    SignalProxy<void(int)> sigCurrentIndexChanged();
    SignalProxy<void(const QString&)> sigEditTextChanged();
    SignalProxy<void(int)> sigHighlighted();
    SignalProxy<void()> sigAboutToShowPopup();

private Q_SLOTS:
    void onActivated(int index);
    void onCurrentIndexChanged(int index);
    void onEditTextChanged(const QString& text);
    void onHighlighted(int index);

private:
    class Impl;
    Impl* impl;
};

}

#endif
