/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_PYTHON3_PLUGIN_PYTHON3_CONSOLE_VIEW_H
#define CNOID_PYTHON3_PLUGIN_PYTHON3_CONSOLE_VIEW_H

#include <cnoid/View>
#include "exportdecl.h"

namespace cnoid {

class Python3ConsoleViewImpl;

class CNOID_EXPORT Python3ConsoleView : public View
{
public:
    static void initializeClass(ExtensionManager* ext);
        
    Python3ConsoleView();
    virtual ~Python3ConsoleView();
        
    void inputCommand(const std::string& command);
    SignalProxy<void(const std::string& output)> sigOutput();
    
private:
    Python3ConsoleViewImpl* impl;
};

}

#endif
