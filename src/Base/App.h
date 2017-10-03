/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_APP_H
#define CNOID_BASE_APP_H

#include <string>
#include <QWidget>
#include "exportdecl.h"

namespace cnoid {

class ExtensionManager;
class AppImpl;

class CNOID_EXPORT App : public QObject
{
    Q_OBJECT
        
public:
    App(int& argc, char**& argv);
    ~App();
        
    void initialize(const char* appName, const char* vendorName, const QIcon& icon, const char* pluginPathList);

    int exec();

    static void clearFocusView();

    virtual bool eventFilter(QObject* watched, QEvent* event);

private:
    AppImpl* impl;

private Q_SLOTS:
    void onFocusChanged(QWidget* old, QWidget* now);

};

}

#endif
