/**
   @author Shin'ichiro Nakaoka
*/

#include "ComboBox.h"
#include <optional>
#include "gettext.h"

using namespace cnoid;

namespace cnoid {

class ComboBox::Impl
{
public:
    bool isI18nEnabled;
    std::string domainName;
        
    std::optional<Signal<void(int)>> sigActivated;
    std::optional<Signal<void(int)>> sigCurrentIndexChanged;
    std::optional<Signal<void(const QString&)>> sigEditTextChanged;
    std::optional<Signal<void(int)>> sigHighlighted;
    Signal<void()> sigAboutToShowPopup;

    Impl();
};

}
    
ComboBox::ComboBox(QWidget* parent)
    : QComboBox(parent)
{
    impl = new Impl;
    isUserInputEnabled_ = true;
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
        impl->sigActivated.emplace();
        connect(this, (void(QComboBox::*)(int)) &QComboBox::activated,
                [this](int index){ (*impl->sigActivated)(index); });
    }
    return *impl->sigActivated;
}


SignalProxy<void(int)> ComboBox::sigCurrentIndexChanged()
{
    if(!impl->sigCurrentIndexChanged){
        impl->sigCurrentIndexChanged.emplace();
        connect(this, (void(QComboBox::*)(int)) &QComboBox::currentIndexChanged,
                [this](int index){ (*impl->sigCurrentIndexChanged)(index); });
    }
    return *impl->sigCurrentIndexChanged;
}
    

SignalProxy<void(const QString&)> ComboBox::sigEditTextChanged()
{
    if(!impl->sigEditTextChanged){
        impl->sigEditTextChanged.emplace();
        connect(this, (void(QComboBox::*)(const QString&)) &QComboBox::editTextChanged,
                [this](const QString& text){ (*impl->sigEditTextChanged)(text); });
    }
    return *impl->sigEditTextChanged;
}


SignalProxy<void(int)> ComboBox::sigHighlighted()
{
    if(!impl->sigHighlighted){
        impl->sigHighlighted.emplace();
        connect(this, (void(QComboBox::*)(int)) &QComboBox::highlighted,
                [this](int index){ (*impl->sigHighlighted)(index); });
    }
    return *impl->sigHighlighted;
}


SignalProxy<void()> ComboBox::sigAboutToShowPopup()
{
    return impl->sigAboutToShowPopup;
}


void ComboBox::keyPressEvent(QKeyEvent* event)
{
    if(isUserInputEnabled_){
        QComboBox::keyPressEvent(event);
    }
}


void ComboBox::mousePressEvent(QMouseEvent* event)
{
    if(isUserInputEnabled_){
        QComboBox::mousePressEvent(event);
    }
}


void ComboBox::wheelEvent(QWheelEvent* event)
{
    if(isUserInputEnabled_){
        QComboBox::wheelEvent(event);
    }
}
