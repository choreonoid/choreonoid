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
    /**
       @if jp
       @param appName アプリケーション名
       @param vendorName ベンダ名
       @endif
    */
    App(int& argc, char**& argv);
    ~App();
        
    void initialize(const char* appName, const char* vendorName, const QIcon& icon, const char* pluginPathList);

    int exec();

    virtual bool eventFilter(QObject* watched, QEvent* event);

private:
    AppImpl* impl;

private Q_SLOTS:
    void onFocusChanged(QWidget* old, QWidget* now);

};

}

#endif
