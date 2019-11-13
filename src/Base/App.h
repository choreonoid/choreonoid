/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_APP_H
#define CNOID_BASE_APP_H

#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT App
{
        
public:
    App(int& argc, char**& argv);
    ~App();
        
    void initialize(const char* appName, const char* vendorName, const char* pluginPathList);

    int exec();

    static void clearFocusView();

private:
    class Impl;
    Impl* impl;
};

}

#endif
