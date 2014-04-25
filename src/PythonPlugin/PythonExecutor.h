/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_PYTHON_PLUGIN_PYTHON_EXECUTOR_H_INCLUDED
#define CNOID_PYTHON_PLUGIN_PYTHON_EXECUTOR_H_INCLUDED

#include "PythonUtil.h"
#include <cnoid/SignalProxy>
#include "exportdecl.h"

namespace cnoid {

class PythonExecutorImpl;

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
        
    bool execCode(const std::string& code);
    bool execFile(const std::string& filename);
    bool waitToFinish(double timeout);
    boost::python::object resultObject();
    const std::string resultString() const;
    SignalProxy< boost::signal<void()> > sigFinished();

    bool hasException() const;
    const std::string exceptionTypeName() const;
    const std::string exceptionText() const;
    boost::python::object exceptionType() const;
    boost::python::object exceptionValue() const;
        
    bool terminate();

private:
    PythonExecutorImpl* impl;
};
}

#endif
