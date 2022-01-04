/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_APP_H
#define CNOID_BASE_APP_H

#include <cnoid/Signal>
#include <string>

#ifdef _WIN32
#include <windows.h>
#endif

#include "exportdecl.h"

namespace cnoid {

class ExtensionManager;

class CNOID_EXPORT App
{
        
public:
    App(int& argc, char** argv, const std::string& appName, const std::string& organization);

#ifdef _WIN32
    App(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nCmdShow,
        const std::string& appName, const std::string& organization);
#endif

    ~App();

    void addPluginPath(const std::string& path);
    bool requirePluginToCustomizeApplication(const std::string& pluginName);

    // Optional setting
    void setIcon(const std::string& filename);

    void setBuiltinProject(const std::string& projectFile);

    /**
       This function is only used to execute some initialization code after the basic App initialization has been finished.
       Even if this function is not explicitly called, the function is called inside the exec function.
    */
    void initialize();

    /**
       This function must be called to execute the application.
    */
    int exec();

    enum ErrorCode { NoError, PluginNotFound, CustomizationFailed };
    ErrorCode error() const;
    const std::string& errorMessage() const;

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
