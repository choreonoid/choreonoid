/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_PYTHON_PLUGIN_PYTHON_EXECUTOR_H
#define CNOID_PYTHON_PLUGIN_PYTHON_EXECUTOR_H

#include "PythonPlugin.h"
#include <cnoid/Signal>
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
    pybind11::object resultObject();
    const std::string resultString() const;
    SignalProxy<void()> sigFinished();

    bool hasException() const;
    const std::string exceptionTypeName() const;
    const std::string exceptionText() const;
    pybind11::object exceptionType() const;
    pybind11::object exceptionValue() const;

    bool isTerminated() const;
        
    bool terminate();

private:
    PythonExecutorImpl* impl;
};

}

#endif
