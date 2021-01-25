/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_PYTHON_PLUGIN_PYTHON_CONSOLE_VIEW_H
#define CNOID_PYTHON_PLUGIN_PYTHON_CONSOLE_VIEW_H

#include <cnoid/View>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT PythonConsoleView : public View
{
public:
    static void initializeClass(ExtensionManager* ext);
    static PythonConsoleView* instance();
        
    PythonConsoleView();
    virtual ~PythonConsoleView();
        
    void inputCommand(const std::string& command);
    SignalProxy<void(const std::string& output)> sigOutput();

    class Impl;
    
protected:
    virtual void onActivated() override;
    
private:
    Impl* impl;
};

}

#endif
