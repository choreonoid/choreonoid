/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_APP_H
#define CNOID_BASE_APP_H

#ifdef _WIN32
#include <windows.h>
#endif

#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT App
{
        
public:
    App(int& argc, char** argv);

#ifdef _WIN32
    App(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nCmdShow);
#endif

    ~App();
        
    void initialize(const char* appName, const char* vendorName, const char* pluginPathList = nullptr);

    int exec();

private:
    class Impl;
    Impl* impl;
};

}

#endif
