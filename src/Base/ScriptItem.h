/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_SCRIPT_ITEM_H
#define CNOID_BASE_SCRIPT_ITEM_H

#include "AbstractTextItem.h"
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT ScriptItem : public AbstractTextItem
{
public:
    static void initializeClass(ExtensionManager* ext);
    
    ScriptItem();
    ScriptItem(const ScriptItem& org);

    virtual const std::string& textFilename() const;
    virtual const std::string& scriptFilename() const = 0;

    virtual std::string identityName() const;

    virtual void setBackgroundMode(bool on) = 0;
    virtual bool isBackgroundMode() const;
    virtual bool isRunning() const;
        
    virtual bool execute() = 0;

    /**
       This function executes the code in the same namespace as that of the script exection.
       @note Implementing this function is optional.
    */
    virtual bool executeCode(const char* code);

    /**
       This function waits for the script to finish.
       @return True if the script is actually finished, or false if timeout happens.
       @note Implementing this function is optional.
       The function returns false if the function is not implemented.
    */
    virtual bool waitToFinish(double timeout = 0.0);
        
    virtual std::string resultString() const;

    virtual SignalProxy<void()> sigScriptFinished() = 0;
        
    virtual bool terminate() = 0;

protected:
    virtual ~ScriptItem();
};

typedef ref_ptr<ScriptItem> ScriptItemPtr;
}

#endif
