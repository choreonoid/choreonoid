/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_APP_H
#define CNOID_BASE_APP_H

#include <cnoid/Signal>

#ifdef _WIN32
#include <windows.h>
#endif

#include "exportdecl.h"

namespace cnoid {

class ExtensionManager;

class CNOID_EXPORT App
{
        
public:
    App(int& argc, char** argv);

#ifdef _WIN32
    App(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nCmdShow);
#endif

    ~App();

    // Optional setting
    void setIcon(const char* filename);

    void initialize(const char* appName, const char* vendorName, const char* pluginPathList = nullptr);
    int exec();

    static bool isDoingInitialization();
    static ExtensionManager* baseModule();
    static void updateGui();
    static void exit(int returnCode = 0);
    static void checkErrorAndExitIfTestMode();
    static SignalProxy<void()> sigExecutionStarted();
    static SignalProxy<void()> sigAboutToQuit();
        
private:
    class Impl;
    Impl* impl;
};

}

#endif
