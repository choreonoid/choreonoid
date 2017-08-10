/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_PYTHON3_PLUGIN_PYTHON3_EXECUTOR_H
#define CNOID_PYTHON3_PLUGIN_PYTHON3_EXECUTOR_H

#include "Python3Plugin.h"
#include <cnoid/Signal>
#include "exportdecl.h"

namespace cnoid {

class Python3ExecutorImpl;

class CNOID_EXPORT Python3Executor
{
public:
    static void setModuleRefreshEnabled(bool on);
        
    Python3Executor();
    Python3Executor(const Python3Executor& org);
    ~Python3Executor();

    void setBackgroundMode(bool on);
    bool isBackgroundMode() const;

    enum State { NOT_RUNNING, RUNNING_FOREGROUND, RUNNING_BACKGROUND };

    State state() const;
        
    bool execCode(const std::string& code);
    bool execFile(const std::string& filename);
    bool waitToFinish(double timeout);
    const std::string resultString() const;
    SignalProxy<void()> sigFinished();

    bool hasException() const;
    const std::string exceptionTypeName() const;
    const std::string exceptionText() const;
    pybind11::object resultObject();
    pybind11::object exceptionType() const;
    pybind11::object exceptionValue() const;

    bool isTerminated() const;
        
    bool terminate();

private:
    Python3ExecutorImpl* impl;
};

}

#endif
