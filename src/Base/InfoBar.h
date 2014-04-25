/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_GUIBASE_INFO_BAR_H_INCLUDED
#define CNOID_GUIBASE_INFO_BAR_H_INCLUDED

#include <string>
#include <QStatusBar>
#include <QHBoxLayout>
#include "exportdecl.h"

namespace cnoid {

class InfoBarImpl;

class CNOID_EXPORT InfoBar : public QStatusBar
{
    Q_OBJECT
        public:
    static InfoBar* instance();
        
    void notify(const char* message);
    void notify(const std::string& message);
    void notify(const QString& message);

private:
    InfoBar();
    ~InfoBar();

    QWidget* indicatorBase;
    QHBoxLayout* indicatorLayout;
    QWidget* currentIndicator;
        
    void setIndicator(QWidget* indicator);
    void removeCurrentIndicator();

private Q_SLOTS:
    void onFocusChanged(QWidget* old, QWidget* now);
    void onIndicatorDestroyed(QObject* obj);
};
}

#endif
