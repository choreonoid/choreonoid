/**
   @author Shin'ichiro NAKAOKA
*/

#ifndef CNOID_GUIBASE_COMBO_BOX_H_INCLUDED
#define CNOID_GUIBASE_COMBO_BOX_H_INCLUDED

#include <cnoid/SignalProxy>
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

    inline SignalProxy< boost::signal<void(int)> > sigActivated() {
        return sigActivated_;
    }
            
    inline SignalProxy< boost::signal<void(int)> > sigCurrentIndexChanged() {
        return sigCurrentIndexChanged_;
    }

    inline SignalProxy< boost::signal<void(const QString&)> > sigEditTextChanged() {
        return sigEditTextChanged_;
    }
        
    inline SignalProxy< boost::signal<void(int)> > sigHighlighted() {
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
        
    boost::signal<void(int)> sigActivated_;
    boost::signal<void(int)> sigCurrentIndexChanged_;
    boost::signal<void(const QString&)> sigEditTextChanged_;
    boost::signal<void(int)> sigHighlighted_;
};
}

#endif
