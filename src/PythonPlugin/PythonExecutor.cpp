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
#include <boost/bind.hpp>
#include <map>
#include <iostream>

using namespace std;
using namespace boost;
using namespace cnoid;

namespace {
    
bool isInitialized = false;

python::object exitExceptionType;
python::object sys;
python::object StringOutClass;

bool isDefaultModuleRefreshEnabled = false;
python::object rollBackImporter;

typedef map<string, int> PathRefMap;
PathRefMap additionalPythonPathRefMap;

class StringOut
{
    string buf;
public:
    void write(string const& text){
        buf += text;
    }
    const string& text() const { return buf; }
};
        
void initializeStaticObjects()
{
    if(!isInitialized){

        PyGILock lock;

        exitExceptionType = python::import("cnoid.PythonPlugin").attr("ExitException");
        sys = python::import("sys");
            
        StringOutClass =
            python::class_<StringOut>("StringOut", python::init<>())
            .def("write", &StringOut::write)
            .def("text", &StringOut::text, python::return_value_policy<python::copy_const_reference>());

        rollBackImporter = python::import("cnoid.rbimporter");

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
    boost::function<boost::python::object()> functionToExecScript;
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
    bool exec(boost::function<boost::python::object()> execScript, const string& filename);
    bool execMain(boost::function<boost::python::object()> execScript);
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
    lastResultObject = boost::python::object(); // null
    lastExceptionType = boost::python::object(); // null
    lastExceptionValue = boost::python::object(); // null
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


static boost::python::object execPythonCodeSub(const std::string& code)
{
    return python::exec(code.c_str(), cnoid::pythonMainNamespace());
}


static boost::python::object execPythonFileSub(const std::string& filename)
{
    return python::exec_file(filename.c_str(), cnoid::pythonMainNamespace());
}


bool PythonExecutor::execCode(const std::string& code)
{
    return impl->exec(boost::bind(execPythonCodeSub, code), "");
}


bool PythonExecutor::execFile(const std::string& filename)
{
    return impl->exec(boost::bind(execPythonFileSub, filename), filename);
}


bool PythonExecutorImpl::exec(boost::function<boost::python::object()> execScript, const string& filename)
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
        PyGILock lock;

        // clear exception variables
        hasException = false;
        exceptionTypeName.clear();
        exceptionText.clear();
        exceptionType = python::object();
        exceptionValue = python::object();

        isTerminated = false;

        functionToExecScript = execScript;

        if(doAddPythonPath){
            python::list syspath = python::extract<python::list>(pythonSysModule().attr("path"));
            syspath.insert(0, scriptDirectory);
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


bool PythonExecutorImpl::execMain(boost::function<boost::python::object()> execScript)
{
    bool completed = false;
    resultObject = boost::python::object();
    resultString.clear();
    
    try {
        resultObject = execScript();
        resultString =  python::extract<string>(python::str(resultObject));
        completed = true;
    }
    catch(python::error_already_set const & ex) {
        if(PyErr_Occurred()){
            if(PyErr_ExceptionMatches(exitExceptionType.ptr())){
                PyErr_Clear();
                isTerminated = true;
            } else {
                PyObject* ptype;
                PyObject* pvalue;
                PyObject* ptraceback;
                PyErr_Fetch(&ptype, &pvalue, &ptraceback);
                if(ptype){
                    exceptionType = python::object(python::handle<>(python::borrowed(ptype)));
                    exceptionTypeName = python::extract<string>(python::str(exceptionType));
                }
                if(pvalue){
                    exceptionValue = python::object(python::handle<>(python::borrowed(pvalue)));
                }
                
                // get an error message by redirecting the output of PyErr_Print()
                python::object stderr_ = sys.attr("stderr");
                python::object strout = StringOutClass();
                sys.attr("stderr") = strout;
                PyErr_Restore(ptype, pvalue, ptraceback);
                PyErr_Print();
                sys.attr("stderr") = stderr_;
                exceptionText = python::extract<string>(strout.attr("text")());

                resultObject = exceptionValue;
                resultString = exceptionText;

                hasException = true;
            }
        }
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
        callLater(boost::bind(&PythonExecutorImpl::onBackgroundExecutionFinished, this));
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
    PyGILock lock;
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
boost::python::object PythonExecutor::resultObject()
{
    impl->stateMutex.lock();
    boost::python::object object = impl->lastResultObject;
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
            PyGILock lock;
            python::list syspath = python::extract<python::list>(pythonSysModule().attr("path"));
            int n = python::len(syspath);
            for(int i=0; i < n; ++i){
                string path = python::extract<string>(syspath[i]);
                if(path == scriptDirectory){
                    syspath.pop(i);
                    break;
                }
            }
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
                PyGILock lock;
                PyThreadState_SetAsyncExc((long)threadId, exitExceptionType().ptr());
            }
            if(wait(20)){
                terminated = true;
                break;
            }
        }
        //releasePythonPathRef();
        
    } else if(isRunningForeground){
        PyGILock lock;
        PyErr_SetObject(exitExceptionType().ptr(), 0);
        //releasePythonPathRef();
        python::throw_error_already_set();
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
boost::python::object PythonExecutor::exceptionType() const
{
    impl->stateMutex.lock();
    python::object exceptionType = impl->lastExceptionType;
    impl->stateMutex.unlock();
    return exceptionType;
}

        
/**
   \note GIL must be obtained when accessing this object.
*/
boost::python::object PythonExecutor::exceptionValue() const
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
