/**
   @author Shin'ichiro Nakaoka
*/

#include "PythonExecutor.h"
#include <cnoid/PyUtil>
#include <cnoid/FileUtil>
#include <cnoid/LazyCaller>
#include <QThread>
#include <QMutex>
#include <QWaitCondition>
#include <pybind11/eval.h>
#include <pybind11/stl.h>
#include <map>

using namespace std;
using namespace cnoid;
namespace filesystem = cnoid::stdx::filesystem;

namespace {

bool isDefaultModuleRefreshEnabled = false;

typedef map<string, int> PathRefMap;
PathRefMap additionalPythonPathRefMap;

}

namespace cnoid {

// defined in PythonPlugin.cpp
python::object getGlobalNamespace();
python::module getSysModule();
python::object getExitException();
python::module getRollbackImporterModule();


class PythonExecutorImpl : public QThread
{
public:
    bool isBackgroundMode;
    bool isRunningForeground;
    bool isModuleRefreshEnabled;
    std::function<python::object()> functionToExecScript;
    Qt::HANDLE threadId;
    mutable QMutex stateMutex;
    QWaitCondition stateCondition;
    python::object resultObject;
    string resultString;
    Signal<void()> sigFinished;

    string scriptDirectory;
    PathRefMap::iterator pathRefIter;

    bool hasException;
    string exceptionTypeName;
    string exceptionText;
    python::object exceptionType;
    python::object exceptionValue;

    python::object lastResultObject;
    string lastResultString;
    python::object lastExceptionType;
    python::object lastExceptionValue;
    string lastExceptionTypeName;
    string lastExceptionText;
    bool isTerminated;

    PythonExecutorImpl();
    PythonExecutorImpl(const PythonExecutorImpl& org);
    void resetLastResultObjects();
    ~PythonExecutorImpl();
    PythonExecutor::State state() const;
    bool exec(std::function<python::object()> execScript, const string& filename);
    bool execMain(std::function<python::object()> execScript);
    virtual void run();
    bool waitToFinish(double timeout);
    void onBackgroundExecutionFinished();
    void releasePythonPathRef();
    bool terminateScript();
};

}


void PythonExecutor::setModuleRefreshEnabled(bool on)
{
    isDefaultModuleRefreshEnabled = on;
}


PythonExecutor::PythonExecutor()
{
    impl = new PythonExecutorImpl();
}


PythonExecutorImpl::PythonExecutorImpl()
{
    isBackgroundMode = false;
    isRunningForeground = false;
    isModuleRefreshEnabled = isDefaultModuleRefreshEnabled;
    hasException = false;
    isTerminated = false;

    resetLastResultObjects();
}


PythonExecutor::PythonExecutor(const PythonExecutor& org)
{
    impl = new PythonExecutorImpl(*org.impl);
}


PythonExecutorImpl::PythonExecutorImpl(const PythonExecutorImpl& org)
{
    isBackgroundMode = org.isBackgroundMode;
    isRunningForeground = false;
    isModuleRefreshEnabled = isDefaultModuleRefreshEnabled;
    hasException = false;
    isTerminated = false;

    resetLastResultObjects();
}


void PythonExecutorImpl::resetLastResultObjects()
{
    lastResultObject = python::object(); // null
    lastExceptionType = python::object(); // null
    lastExceptionValue = python::object(); // null
}


PythonExecutor::~PythonExecutor()
{
    delete impl;
}


PythonExecutorImpl::~PythonExecutorImpl()
{
    if(state() == PythonExecutor::RUNNING_BACKGROUND){
        if(!terminateScript()){
            QThread::terminate();
            wait();
        }
    }
}


void PythonExecutor::setBackgroundMode(bool on)
{
    impl->isBackgroundMode = on;
}


bool PythonExecutor::isBackgroundMode() const
{
    return impl->isBackgroundMode;
}


PythonExecutor::State PythonExecutorImpl::state() const
{
    PythonExecutor::State state;
    if(QThread::isRunning()){
        state = PythonExecutor::RUNNING_BACKGROUND;
    } else {
        stateMutex.lock();
        if(isRunningForeground){
            state = PythonExecutor::RUNNING_FOREGROUND;
        } else {
            state = PythonExecutor::NOT_RUNNING;
        }
        stateMutex.unlock();
    }
    return state;
}


PythonExecutor::State PythonExecutor::state() const
{
    return impl->state();
}


static python::object execPythonFileSub(const std::string& filename)
{
    return pybind11::eval_file(filename.c_str(), getGlobalNamespace());
}


bool PythonExecutor::execCode(const std::string& code)
{
    return impl->exec(
        [=](){ return pybind11::eval<pybind11::eval_statements>(code.c_str(), getGlobalNamespace()); },
        "");
}


bool PythonExecutor::execFile(const std::string& filename)
{
    return impl->exec([=](){ return execPythonFileSub(filename); }, filename);
}


bool PythonExecutorImpl::exec(std::function<python::object()> execScript, const string& filename)
{
    if(state() != PythonExecutor::NOT_RUNNING){
        return false;
    }

    bool doAddPythonPath = false;
    pathRefIter = additionalPythonPathRefMap.end();

    filesystem::path filepath;

    if(filename.empty()){
        scriptDirectory.clear();
    } else {
        filepath = getAbsolutePath(filesystem::path(filename));
        scriptDirectory = getPathString(filepath.parent_path());
        if(!scriptDirectory.empty()){
            pathRefIter = additionalPythonPathRefMap.find(scriptDirectory);
            if(pathRefIter == additionalPythonPathRefMap.end()){
                pathRefIter = additionalPythonPathRefMap.insert(PathRefMap::value_type(scriptDirectory, 1)).first;
                doAddPythonPath = true;
            } else {
                pathRefIter->second += 1;
            }
        }
    }

    bool result = true;
    {
        python::gil_scoped_acquire lock;

        // clear exception variables
        hasException = false;
        exceptionTypeName.clear();
        exceptionText.clear();
        exceptionType = python::object();
        exceptionValue = python::object();

        isTerminated = false;

        functionToExecScript = execScript;

        if(doAddPythonPath){
            getSysModule().attr("path").attr("insert")(0, scriptDirectory);
        }

        if(isModuleRefreshEnabled){
            getRollbackImporterModule().attr("refresh")(scriptDirectory);
        }

        if(!filename.empty()){
            filesystem::path relative;
            if(findRelativePath(filesystem::current_path(), filepath, relative)){
                getGlobalNamespace()["__file__"] = getPathString(relative);
            }
        }

        if(!isBackgroundMode){
            stateMutex.lock();
            threadId = currentThreadId();
            isRunningForeground = true;
            stateMutex.unlock();
            
            result = execMain(execScript);
        }
    }
    if(isBackgroundMode){
        stateMutex.lock();
        isRunningForeground = false;
        start();
        // wait for the threadId variable to be set in the run() function.
        stateCondition.wait(&stateMutex);
        stateMutex.unlock();
    }

    return result;
}


bool PythonExecutorImpl::execMain(std::function<python::object()> execScript)
{
    bool completed = false;
    resultObject = python::object();
    resultString.clear();
    
    try {
        resultObject = execScript();
        resultString.clear();
        completed = true;
    }
    catch(const python::error_already_set& ex) {
        exceptionText = ex.what();
        resultString = exceptionText;
        hasException = true;
        if(ex.matches(getExitException())){
            isTerminated = true;
        }
    }

    releasePythonPathRef();

    stateMutex.lock();
    isRunningForeground = false;
    lastResultObject = resultObject;
    lastResultString = resultString;
    lastExceptionType = exceptionType;
    lastExceptionValue = exceptionValue;
    lastExceptionTypeName = exceptionTypeName;
    lastExceptionText = exceptionText;
    stateCondition.wakeAll();
    stateMutex.unlock();
    
    if(QThread::isRunning()){
        callLater([&](){ onBackgroundExecutionFinished(); });
    } else {
        sigFinished();
    }

    return completed;
}


void PythonExecutorImpl::run()
{
    stateMutex.lock();
    threadId = currentThreadId();
    stateCondition.wakeAll();
    stateMutex.unlock();
    python::gil_scoped_acquire lock;
    execMain(functionToExecScript);
}


bool PythonExecutor::waitToFinish(double timeout)
{
    return impl->waitToFinish(timeout);
}


bool PythonExecutorImpl::waitToFinish(double timeout)
{
    unsigned long time = (timeout == 0.0) ? ULONG_MAX : timeout * 1000.0;
    
    if(QThread::isRunning()){
        return wait(time);
    } else if(isRunningForeground){
        stateMutex.lock();
        const bool isDifferentThread = (threadId != QThread::currentThreadId());
        stateMutex.unlock();
        if(!isDifferentThread){
            return false;
        } else {
            bool isTimeout = false;
            while(true){
                bool finished = false;
                stateMutex.lock();
                if(!isRunningForeground){
                    finished = true;
                } else {
                    isTimeout = !stateCondition.wait(&stateMutex, time);
                    finished = !isRunningForeground;
                }
                stateMutex.unlock();
                if(finished || isTimeout){
                    break;
                }
            }
            return !isTimeout;
        }
    }
    return true;
}


/**
   \note GIL must be obtained when accessing this object.
*/
python::object PythonExecutor::resultObject()
{
    impl->stateMutex.lock();
    python::object object = impl->lastResultObject;
    impl->stateMutex.unlock();
    return object;
}


const std::string PythonExecutor::resultString() const
{
    impl->stateMutex.lock();
    string result = impl->lastResultString;
    impl->stateMutex.unlock();
    return result;
}


void PythonExecutorImpl::onBackgroundExecutionFinished()
{
    sigFinished();
}


void PythonExecutorImpl::releasePythonPathRef()
{
    /**
       When a number of Python scripts is proccessed, releasing the path corresponding to a certain
       script may affect other scripts. To prevent it, set true to the following constant value.
    */
    static const bool DISABLE_RELEASE = true;

    if(DISABLE_RELEASE){
        return;
    }
    
    if(pathRefIter != additionalPythonPathRefMap.end()){
        if(--pathRefIter->second == 0){
            python::gil_scoped_acquire lock;
            getSysModule().attr("path").attr("remove")(scriptDirectory);
            additionalPythonPathRefMap.erase(pathRefIter);
        }
        pathRefIter = additionalPythonPathRefMap.end();
    }
}


SignalProxy<void()> PythonExecutor::sigFinished()
{
    return impl->sigFinished;
}


bool PythonExecutor::terminate()
{
    return impl->terminateScript();
}


bool PythonExecutorImpl::terminateScript()
{
    bool terminated = true;

    if(QThread::isRunning()){
        terminated = false;

        for(int i=0; i < 400; ++i){
            {
                python::gil_scoped_acquire lock;

                /**
                   Set the exception class itself instead of an instance of the exception class
                   because the following function only accepts a single parameter with regard to the
                   exception object in constrast to PyErr_SetObject that takes both the type and value
                   of the exeption, and if the instance is given to the following function, the
                   exception type will be unknown in the exception handler, which makes it impossible
                   for the handler to check if the termination is requested. By giving the class object,
                   the handler can detect the exception type even in this case.
                */
                PyThreadState_SetAsyncExc((long)threadId, getExitException().ptr());
            }
            if(wait(20)){
                terminated = true;
                break;
            }
        }
        releasePythonPathRef();
        
    } else if(isRunningForeground){
        python::gil_scoped_acquire lock;
        PyErr_SetObject(getExitException().ptr(), 0);
        releasePythonPathRef();
        
        if(PyErr_Occurred()) throw pybind11::error_already_set();
    }

    return terminated;
}


bool PythonExecutor::hasException() const
{
    return impl->hasException;
}


/**
   \note The name includes module components.
*/
const std::string PythonExecutor::exceptionTypeName() const
{
    impl->stateMutex.lock();
    string name = impl->lastExceptionTypeName;
    impl->stateMutex.unlock();
    return name;
}


const std::string PythonExecutor::exceptionText() const
{
    impl->stateMutex.lock();
    string text = impl->lastExceptionText;
    impl->stateMutex.unlock();
    return text;
}


/**
   \note GIL must be obtained when accessing this object.
*/
python::object PythonExecutor::exceptionType() const
{
    impl->stateMutex.lock();
    python::object exceptionType = impl->lastExceptionType;
    impl->stateMutex.unlock();
    return exceptionType;
}

        
/**
   \note GIL must be obtained when accessing this object.
*/
python::object PythonExecutor::exceptionValue() const
{
    impl->stateMutex.lock();
    python::object value = impl->lastExceptionValue;
    impl->stateMutex.unlock();
    return value;
}


bool PythonExecutor::isTerminated() const
{
    return impl->isTerminated;
}
