/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_PYTHON_PLUGIN_PYTHON_EXECUTOR_H
#define CNOID_PYTHON_PLUGIN_PYTHON_EXECUTOR_H

#include <cnoid/PyUtil>
#include <cnoid/Signal>
#include "exportdecl.h"

namespace cnoid {

/**
   \note GIL must be locked to access to the objects of this class.
   Even the destructor requires GIL to be locked when it is executed.
*/
class CNOID_EXPORT PythonExecutor
{
public:
    static void setModuleRefreshEnabled(bool on);
        
    PythonExecutor();
    PythonExecutor(const PythonExecutor& org);
    ~PythonExecutor();

    void setBackgroundMode(bool on);
    bool isBackgroundMode() const;

    enum State { NOT_RUNNING, RUNNING_FOREGROUND, RUNNING_BACKGROUND };

    State state() const;

    bool eval(const std::string& code);

    //! \note This is only valid when the "eval' function is executed.
    python::object returnValue();

    //! \deprecated. Use the returnValue function.
    python::object resultObject() { return returnValue(); }

    bool execCode(const std::string& code);
    bool execFile(const std::string& filename);
    bool waitToFinish(double timeout);

    SignalProxy<void()> sigFinished();

    bool hasException() const;
    const std::string exceptionTypeName() const;
    const std::string exceptionText() const;
    python::object exceptionType() const;
    python::object exceptionValue() const;

    //! \deprecated. Use the 'exceptionText' function.
    const std::string resultString() const { return exceptionText(); }

    bool isTerminated() const;
        
    bool terminate();

private:
    class Impl;
    Impl* impl;
};

}

#endif
