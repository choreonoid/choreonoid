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
    ComboBox(QWidget* parent = 0);

    void enableI18n(const char* domainname);

    void addI18nItem(const char* text);
    void addI18nItem(const QIcon & icon, const char* text);

    QString currentOrgText() const;
    int findOrgText(const std::string& text, bool setFoundItemCurrent = false);

    SignalProxy<void(int)> sigActivated() {
        return sigActivated_;
    }
            
    SignalProxy<void(int)> sigCurrentIndexChanged() {
        return sigCurrentIndexChanged_;
    }

    SignalProxy<void(const QString&)> sigEditTextChanged() {
        return sigEditTextChanged_;
    }
        
    SignalProxy<void(int)> sigHighlighted() {
        return sigHighlighted_;
    }

private Q_SLOTS:
    void onActivated(int index);
    void onCurrentIndexChanged(int index);
    void onEditTextChanged(const QString& text);
    void onHighlighted(int index);

private:
    bool isI18nEnabled;
    std::string domainName;
        
    Signal<void(int)> sigActivated_;
    Signal<void(int)> sigCurrentIndexChanged_;
    Signal<void(const QString&)> sigEditTextChanged_;
    Signal<void(int)> sigHighlighted_;
};

}

#endif
