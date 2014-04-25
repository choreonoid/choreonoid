/**
   @author Shin'ichiro NAKAOKA
*/

#include "ComboBox.h"
#include "gettext.h"

using namespace cnoid;

ComboBox::ComboBox(QWidget* parent)
    : QComboBox(parent)
{
    isI18nEnabled = false;
    
    connect(this, SIGNAL(activated(int)), this, SLOT(onActivated(int)));
    connect(this, SIGNAL(currentIndexChanged(int)), this, SLOT(onCurrentIndexChanged(int)));
    connect(this, SIGNAL(editTextChanged(const QString&)), this, SLOT(onEditTextChanged(const QString&)));
    connect(this, SIGNAL(highlighted(int)), this, SLOT(onHighlighted(int)));
}


void ComboBox::enableI18n(const char* domainName)
{
    isI18nEnabled = true;
    this->domainName = domainName;
}


void ComboBox::addI18nItem(const char* text)
{
    if(isI18nEnabled){
        QComboBox::addItem(dgettext(domainName.c_str(), text), text);
    } else {
        QComboBox::addItem(text);
    }
}


void ComboBox::addI18nItem(const QIcon & icon, const char* text)
{
    if(isI18nEnabled){
        QComboBox::addItem(icon, dgettext(domainName.c_str(), text), text);
    } else {
        QComboBox::addItem(icon, text);
    }
}


QString ComboBox::currentOrgText() const
{
    if(isI18nEnabled){
        return itemData(currentIndex()).toString();
    } else {
        return currentText();
    }
}


int ComboBox::findOrgText(const std::string& orgText, bool setFoundItemCurrent)
{
    QString translatedText;
    if(isI18nEnabled){
        translatedText = dgettext(domainName.c_str(), orgText.c_str());
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


void ComboBox::onActivated(int index)
{
    sigActivated_(index);
}


void ComboBox::onCurrentIndexChanged(int index)
{
    sigCurrentIndexChanged_(index);
}


void ComboBox::onEditTextChanged(const QString& text)
{
    sigEditTextChanged_(text);
}


void ComboBox::onHighlighted(int index)
{
    sigHighlighted_(index);
}
