#ifndef CNOID_PYTHON_PLUGIN_PYTHON_EXECUTOR_H
#define CNOID_PYTHON_PLUGIN_PYTHON_EXECUTOR_H

#include <cnoid/Signal>
#include <string>
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

    enum NamespaceMode { SharedNamespace, IndividualNamespace };

    void setNamespaceMode(NamespaceMode mode);
    NamespaceMode namespaceMode() const;
    void setNamespaceClearedOnExecution(bool on);
    bool isNamespaceClearedOnExecution() const;

    enum State { NOT_RUNNING, RUNNING_FOREGROUND, RUNNING_BACKGROUND };

    State state() const;

    bool eval(const std::string& code);
    bool execCode(const std::string& code);
    bool execFile(const std::string& filename);
    bool waitToFinish(double timeout);

    SignalProxy<void()> sigFinished();

    bool hasException() const;
    const std::string exceptionTypeName() const;
    const std::string exceptionText() const;

    bool isTerminated() const;

    bool terminate();

private:
    class Impl;
    Impl* impl;
};

}

#endif
