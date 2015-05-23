/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_PYTHON_PLUGIN_PYTHON_CONSOLE_VIEW_H
#define CNOID_PYTHON_PLUGIN_PYTHON_CONSOLE_VIEW_H

#include <cnoid/View>
#include "exportdecl.h"

namespace cnoid {

class PythonConsoleViewImpl;

class CNOID_EXPORT PythonConsoleView : public View
{
public:
    static void initializeClass(ExtensionManager* ext);
        
    PythonConsoleView();
    virtual ~PythonConsoleView();
        
    void put(const std::string& message);
    void putln(const std::string& message);
    void flush();
    void clear();
    
private:
    PythonConsoleViewImpl* impl;
};

}

#endif
