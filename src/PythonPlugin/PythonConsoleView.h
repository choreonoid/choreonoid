/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_PYTHON_PLUGIN_PYTHON_CONSOLE_VIEW_H_INCLUDED
#define CNOID_PYTHON_PLUGIN_PYTHON_CONSOLE_VIEW_H_INCLUDED

#include <cnoid/View>

namespace cnoid {

class PythonConsoleViewImpl;

class PythonConsoleView : public View
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
