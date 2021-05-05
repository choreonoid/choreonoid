/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_INFO_BAR_H
#define CNOID_BASE_INFO_BAR_H

#include <QStatusBar>
#include <QHBoxLayout>
#include <string>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT InfoBar : public QStatusBar
{
public:
    static InfoBar* instance();
        
    void notify(const std::string& message);
    void notify(const char* message);
    void notify(const QString& message);

private:
    QWidget* indicatorBase;
    QHBoxLayout* indicatorLayout;
    QWidget* currentIndicator;
    QMetaObject::Connection indicatorConnection;
    std::string plainTextMessage;
        
    InfoBar();
    ~InfoBar();
    void setIndicator(QWidget* indicator);
    void removeCurrentIndicator();
    void onFocusChanged(QWidget* old, QWidget* now);
    void onIndicatorDestroyed(QObject* obj);
};

}

#endif
