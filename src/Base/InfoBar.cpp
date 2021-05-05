/**
   @author Shin'ichiro Nakaoka
*/

#include "InfoBar.h"
#include "View.h"
#include <QApplication>
#include <QLabel>
#include <regex>
#include "gettext.h"

using namespace std;
using namespace cnoid;


InfoBar* InfoBar::instance()
{
    static InfoBar* infoBar = new InfoBar();
    return infoBar;
}


InfoBar::InfoBar()
{
    /**
       The following label is inserted to make the height of the info bar constant.
       The font height varies depending on the character set actually used in the text.
       For example, in Japanese environment, the size of the label consisting of ascii characters
       and that consisting of kanji characters are often different, and the sizeHint or
       minimumSizeHint function of QLabel returns different height depending on the actual text.
       This changes the layout of the main window, but the unintentional layout change should
       be avoided.
       By inserting the label with the space character which is translated into the space defined
       in the local character set, the sizeHint always returns a constant size because a local
       character is usually slightly bigger than an ascii character.
       Note that in order for this method to work, you have to specify the translation of the space
       character in the gettext po files.
    */
    addPermanentWidget(new QLabel(_(" ")));
    
    indicatorBase = new QWidget(this);
    indicatorLayout = new QHBoxLayout();
    indicatorLayout->setContentsMargins(0, 0, 0, 0);
    indicatorLayout->setSpacing(0);
    indicatorBase->setLayout(indicatorLayout);
    addPermanentWidget(indicatorBase);
    
    currentIndicator = nullptr;

    connect(qApp, &QApplication::focusChanged,
            [this](QWidget* old, QWidget* now){ onFocusChanged(old, now); });
}


InfoBar::~InfoBar()
{
    removeCurrentIndicator();
}


void InfoBar::notify(const std::string& message)
{
    static regex escseqPattern("\\x1b\\[([0-9;]*)([A-z])");
    plainTextMessage.clear();
    regex_replace(back_inserter(plainTextMessage), message.begin(), message.end(), escseqPattern, "");
    showMessage(QString(plainTextMessage.c_str()), 5000);
}


void InfoBar::notify(const char* message)
{
    notify(string(message));
}


void InfoBar::notify(const QString& message)
{
    notify(message.toStdString());
}


void InfoBar::setIndicator(QWidget* indicator)
{
    if(indicator != currentIndicator){
        removeCurrentIndicator();
        if(indicator){
            indicatorLayout->addWidget(indicator);
            indicator->show();
            indicatorConnection =
                connect(indicator, &QObject::destroyed,
                        [this](QObject* obj){ onIndicatorDestroyed(obj); });
            currentIndicator = indicator;
        }
    }
}


void InfoBar::removeCurrentIndicator()
{
    if(currentIndicator){
        indicatorLayout->removeWidget(currentIndicator);
        currentIndicator->hide();
        currentIndicator->setParent(nullptr);
        currentIndicator = nullptr;
        disconnect(indicatorConnection);
    }
}    


void InfoBar::onIndicatorDestroyed(QObject* obj)
{
    if(obj == currentIndicator){
        currentIndicator = nullptr;
    }
}


void InfoBar::onFocusChanged(QWidget* /* old */, QWidget* now)
{
    QWidget* widget = now;
    while(widget){
        View* view = dynamic_cast<View*>(widget);
        if(view){
            QWidget* indicator = view->indicatorOnInfoBar();
            if(indicator){
                setIndicator(indicator);
                break;
            }
        }
        widget = widget->parentWidget();
    }
}
