/**
   @author Shin'ichiro Nakaoka
*/

#include "ComboBox.h"
#include <cnoid/stdx/optional>
#include "gettext.h"

using namespace cnoid;

namespace cnoid {

class ComboBox::Impl
{
public:
    bool isI18nEnabled;
    std::string domainName;
        
    stdx::optional<Signal<void(int)>> sigActivated;
    stdx::optional<Signal<void(int)>> sigCurrentIndexChanged;
    stdx::optional<Signal<void(const QString&)>> sigEditTextChanged;
    stdx::optional<Signal<void(int)>> sigHighlighted;
    Signal<void()> sigAboutToShowPopup;

    Impl();
};

}
    
ComboBox::ComboBox(QWidget* parent)
    : QComboBox(parent)
{
    impl = new Impl;
}


ComboBox::Impl::Impl()
{
    isI18nEnabled = false;
}


ComboBox::~ComboBox()
{
    delete impl;
}


void ComboBox::enableI18n(const char* domainName)
{
    impl->isI18nEnabled = true;
    impl->domainName = domainName;
}


void ComboBox::addI18nItem(const char* text)
{
    if(impl->isI18nEnabled){
        QComboBox::addItem(dgettext(impl->domainName.c_str(), text), text);
    } else {
        QComboBox::addItem(text);
    }
}


void ComboBox::addI18nItem(const QIcon & icon, const char* text)
{
    if(impl->isI18nEnabled){
        QComboBox::addItem(icon, dgettext(impl->domainName.c_str(), text), text);
    } else {
        QComboBox::addItem(icon, text);
    }
}


QString ComboBox::currentOrgText() const
{
    if(impl->isI18nEnabled){
        return itemData(currentIndex()).toString();
    } else {
        return currentText();
    }
}


int ComboBox::findOrgText(const std::string& orgText, bool setFoundItemCurrent)
{
    QString translatedText;
    if(impl->isI18nEnabled){
        translatedText = dgettext(impl->domainName.c_str(), orgText.c_str());
    } else {
        translatedText = orgText.c_str();
    }
    int index = findText(translatedText);
    if(index >= 0){
        if(setFoundItemCurrent){
            setCurrentIndex(index);
        }
    }
    return index;
}


void ComboBox::showPopup()
{
    impl->sigAboutToShowPopup();
    
    QComboBox::showPopup();
}


SignalProxy<void(int)> ComboBox::sigActivated()
{
    if(!impl->sigActivated){
        stdx::emplace(impl->sigActivated);
        connect(this, SIGNAL(activated(int)), this, SLOT(onActivated(int)));
    }
    return *impl->sigActivated;
}


SignalProxy<void(int)> ComboBox::sigCurrentIndexChanged()
{
    if(!impl->sigCurrentIndexChanged){
        stdx::emplace(impl->sigCurrentIndexChanged);
        connect(this, SIGNAL(currentIndexChanged(int)), this, SLOT(onCurrentIndexChanged(int)));

    }
    return *impl->sigCurrentIndexChanged;
}
    

SignalProxy<void(const QString&)> ComboBox::sigEditTextChanged()
{
    if(!impl->sigEditTextChanged){
        stdx::emplace(impl->sigEditTextChanged);
        connect(this, SIGNAL(editTextChanged(const QString&)), this, SLOT(onEditTextChanged(const QString&)));
    }
    return *impl->sigEditTextChanged;
}


SignalProxy<void(int)> ComboBox::sigHighlighted()
{
    if(!impl->sigHighlighted){
        stdx::emplace(impl->sigHighlighted);
        connect(this, SIGNAL(highlighted(int)), this, SLOT(onHighlighted(int)));
    }
    return *impl->sigHighlighted;
}


SignalProxy<void()> ComboBox::sigAboutToShowPopup()
{
    return impl->sigAboutToShowPopup;
}


void ComboBox::onActivated(int index)
{
    (*impl->sigActivated)(index);
}


void ComboBox::onCurrentIndexChanged(int index)
{
    (*impl->sigCurrentIndexChanged)(index);
}


void ComboBox::onEditTextChanged(const QString& text)
{
    (*impl->sigEditTextChanged)(text);
}


void ComboBox::onHighlighted(int index)
{
    (*impl->sigHighlighted)(index);
}
