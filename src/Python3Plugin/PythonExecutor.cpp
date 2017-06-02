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
#include <boost/version.hpp>
#include <pybind11/eval.h>
#include <pybind11/stl.h>
#include <map>
#include <iostream>

using namespace std;
using namespace cnoid;
namespace filesystem = boost::filesystem;
namespace py = pybind11;


namespace {
    
bool isInitialized = false;

py::object exitExceptionType;
py::object sys;
py::object StringOutClass;

bool isDefaultModuleRefreshEnabled = false;
py::object rollBackImporter;

typedef map<string, int> PathRefMap;
PathRefMap additionalPythonPathRefMap;

class StringOut
{
    string buf;
public:
    void write(string const& text){
        buf += text;
    }
    const string& text() const {
        return buf; }
};
        
void initializeStaticObjects()
{
    if(!isInitialized){

        py::gil_scoped_acquire lock;

        exitExceptionType = py::module::import("cnoid.PythonPlugin").attr("ExitException");
        sys = py::module::import("sys");

        py::module m = pythonMainModule();
        StringOutClass =
                py::class_<StringOut>(m, "StringOut")
                .def(py::init<>())
                .def("write", &StringOut::write)
                .def("text", &StringOut::text, py::return_value_policy::copy);

        rollBackImporter = py::module::import("cnoid.rbimporter");

        isInitialized = true;
    }
}
}


namespace cnoid {

class PythonExecutorImpl : public QThread
{
public:
    bool isBackgroundMode;
    bool isRunningForeground;
    bool isModuleRefreshEnabled;
    std::function<py::object()> functionToExecScript;
    Qt::HANDLE threadId;
    mutable QMutex stateMutex;
    QWaitCondition stateCondition;
    py::object resultObject;
    string resultString;
    Signal<void()> sigFinished;

    string scriptDirectory;
    PathRefMap::iterator pathRefIter;
        
    bool hasException;
    string exceptionTypeName;
    string exceptionText;
    py::object exceptionType;
    py::object exceptionValue;

    py::object lastResultObject;
    string lastResultString;
    py::object lastExceptionType;
    py::object lastExceptionValue;
    string lastExceptionTypeName;
    string lastExceptionText;
    bool isTerminated;

    PythonExecutorImpl();
    PythonExecutorImpl(const PythonExecutorImpl& org);
    void resetLastResultObjects();
    ~PythonExecutorImpl();
    PythonExecutor::State state() const;
    bool exec(std::function<py::object()> execScript, const string& filename);
    bool execMain(std::function<py::object()> execScript);
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
    lastResultObject = py::object(); // null
    lastExceptionType = py::object(); // null
    lastExceptionValue = py::object(); // null
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


static py::object execPythonCodeSub(const std::string& code)
{
    return py::eval<py::eval_statements>(code.c_str(), cnoid::pythonMainNamespace());
}


static py::object execPythonFileSub(const std::string& filename)
{
    return py::eval_file(filename.c_str(), cnoid::pythonMainNamespace());
}


bool PythonExecutor::execCode(const std::string& code)
{
    return impl->exec(std::bind(execPythonCodeSub, code), "");
}


bool PythonExecutor::execFile(const std::string& filename)
{
    return impl->exec(std::bind(execPythonFileSub, filename), filename);
}


bool PythonExecutorImpl::exec(std::function<py::object()> execScript, const string& filename)
{
    if(state() != PythonExecutor::NOT_RUNNING){
        return false;
    }

    if(!isInitialized){
        initializeStaticObjects();
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
        py::gil_scoped_acquire lock;

        // clear exception variables
        hasException = false;
        exceptionTypeName.clear();
        exceptionText.clear();
        exceptionType = py::object();
        exceptionValue = py::object();

        isTerminated = false;

        functionToExecScript = execScript;

        if(doAddPythonPath){
            pythonSysModule().attr("path").attr("insert")(0, scriptDirectory);
        }

        if(isModuleRefreshEnabled){
            rollBackImporter.attr("refresh")(scriptDirectory);
        }

        if(!filename.empty()){
            filesystem::path relative;
            if(findRelativePath(filesystem::current_path(), filepath, relative)){
                pythonMainNamespace()["__file__"] = getPathString(relative);
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


bool PythonExecutorImpl::execMain(std::function<py::object()> execScript)
{
    bool completed = false;
    resultObject = py::object();
    resultString.clear();
    
    try {
        resultObject = execScript();
        completed = true;
    }
    catch(py::error_already_set const & ex) {
        py::object stdout_ = sys.attr("stdout");
        py::object strout = StringOutClass();
        sys.attr("stdout") = strout;
        py::print(ex.what());
        sys.attr("stdout") = stdout_;
        py::object st = strout.attr("text")();
        exceptionText = strout.attr("text")().cast<string>();
        resultString = exceptionText;

        hasException = true;
    }

    //releasePythonPathRef();

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
        callLater(std::bind(&PythonExecutorImpl::onBackgroundExecutionFinished, this));
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
    py::gil_scoped_acquire lock;
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
py::object PythonExecutor::resultObject()
{
    impl->stateMutex.lock();
    py::object object = impl->lastResultObject;
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
    if(pathRefIter != additionalPythonPathRefMap.end()){
        if(--pathRefIter->second == 0){
            py::gil_scoped_acquire lock;
            pythonSysModule().attr("path").attr("remove")(scriptDirectory);

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
                py::gil_scoped_acquire lock;
                PyThreadState_SetAsyncExc((long)threadId, exitExceptionType().ptr());
            }
            if(wait(20)){
                terminated = true;
                break;
            }
        }
        //releasePythonPathRef();
        
    } else if(isRunningForeground){
        py::gil_scoped_acquire lock;
        PyErr_SetObject(exitExceptionType().ptr(), 0);
        //releasePythonPathRef();
        if( PyErr_Occurred())
            throw py::error_already_set();
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
py::object PythonExecutor::exceptionType() const
{
    impl->stateMutex.lock();
    py::object exceptionType = impl->lastExceptionType;
    impl->stateMutex.unlock();
    return exceptionType;
}

        
/**
   \note GIL must be obtained when accessing this object.
*/
py::object PythonExecutor::exceptionValue() const
{
    impl->stateMutex.lock();
    py::object value = impl->lastExceptionValue;
    impl->stateMutex.unlock();
    return value;
}


bool PythonExecutor::isTerminated() const
{
    return impl->isTerminated;
}
