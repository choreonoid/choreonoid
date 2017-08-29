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
#include <map>
#include <iostream>

#ifdef CNOID_USE_PYBIND11
#include <pybind11/eval.h>
#include <pybind11/stl.h>
#endif

// Boost 1.58
#if BOOST_VERSION / 100 % 1000 == 58
#include <fstream>
#endif

using namespace std;
using namespace cnoid;
namespace filesystem = boost::filesystem;

namespace {
    
bool isInitialized = false;

pybind11::object exitExceptionType;
pybind11::object sys;
pybind11::object StringOutClass;

bool isDefaultModuleRefreshEnabled = false;
pybind11::object rollBackImporter;

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

        pybind11::gil_scoped_acquire lock;

        exitExceptionType = pybind11::module::import("cnoid.PythonPlugin").attr("ExitException");
        sys = pybind11::module::import("sys");

#ifdef CNOID_USE_PYBIND11
        pybind11::module m = pythonMainModule();
        StringOutClass =
            pybind11::class_<StringOut>(m, "StringOut")
            .def(pybind11::init<>())
            .def("write", &StringOut::write)
            .def("text", &StringOut::text, pybind11::return_value_policy::copy);
#else
        StringOutClass =
            boost::python::class_<StringOut>("StringOut", boost::python::init<>())
            .def("write", &StringOut::write)
            .def("text", &StringOut::text, boost::python::return_value_policy<boost::python::copy_const_reference>());
#endif

        rollBackImporter = pybind11::module::import("cnoid.rbimporter");

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
    std::function<pybind11::object()> functionToExecScript;
    Qt::HANDLE threadId;
    mutable QMutex stateMutex;
    QWaitCondition stateCondition;
    pybind11::object resultObject;
    string resultString;
    Signal<void()> sigFinished;

    string scriptDirectory;
    PathRefMap::iterator pathRefIter;
        
    bool hasException;
    string exceptionTypeName;
    string exceptionText;
    pybind11::object exceptionType;
    pybind11::object exceptionValue;

    pybind11::object lastResultObject;
    string lastResultString;
    pybind11::object lastExceptionType;
    pybind11::object lastExceptionValue;
    string lastExceptionTypeName;
    string lastExceptionText;
    bool isTerminated;

    PythonExecutorImpl();
    PythonExecutorImpl(const PythonExecutorImpl& org);
    void resetLastResultObjects();
    ~PythonExecutorImpl();
    PythonExecutor::State state() const;
    bool exec(std::function<pybind11::object()> execScript, const string& filename);
    bool execMain(std::function<pybind11::object()> execScript);
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
    lastResultObject = pybind11::object(); // null
    lastExceptionType = pybind11::object(); // null
    lastExceptionValue = pybind11::object(); // null
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


static pybind11::object execPythonCodeSub(const std::string& code)
{
#ifdef CNOID_USE_PYBIND11
    return pybind11::eval<pybind11::eval_statements>(code.c_str(), cnoid::pythonMainNamespace());
#else    
    return boost::python::exec(code.c_str(), cnoid::pythonMainNamespace());
#endif
}


static pybind11::object execPythonFileSub(const std::string& filename)
{
#ifdef CNOID_USE_PYBIND11
    return pybind11::eval_file(filename.c_str(), cnoid::pythonMainNamespace());
#else
// Boost 1.58
#if BOOST_VERSION / 100 % 1000 == 58
    // Avoid a segv with exec_file
    // See: https://github.com/boostorg/python/pull/15
    std::ifstream t(filename.c_str());
    std::stringstream buffer;
    buffer << t.rdbuf();
    return execPythonCodeSub(buffer.str().c_str());
#else // default implementation
    return boost::python::exec_file(filename.c_str(), cnoid::pythonMainNamespace());
#endif
#endif
}


bool PythonExecutor::execCode(const std::string& code)
{
    return impl->exec(std::bind(execPythonCodeSub, code), "");
}


bool PythonExecutor::execFile(const std::string& filename)
{
    return impl->exec(std::bind(execPythonFileSub, filename), filename);
}


bool PythonExecutorImpl::exec(std::function<pybind11::object()> execScript, const string& filename)
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
        pybind11::gil_scoped_acquire lock;

        // clear exception variables
        hasException = false;
        exceptionTypeName.clear();
        exceptionText.clear();
        exceptionType = pybind11::object();
        exceptionValue = pybind11::object();

        isTerminated = false;

        functionToExecScript = execScript;

        if(doAddPythonPath){
#ifdef CNOID_USE_PYBIND11
            pythonSysModule().attr("path").attr("insert")(0, scriptDirectory);
#else
            boost::python::list syspath = boost::python::extract<pybind11::list>(pythonSysModule().attr("path"));
            syspath.insert(0, scriptDirectory);
#endif
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


bool PythonExecutorImpl::execMain(std::function<pybind11::object()> execScript)
{
    bool completed = false;
    resultObject = pybind11::object();
    resultString.clear();
    
    try {
        resultObject = execScript();
#ifdef CNOID_USE_BOOST_PYTHON
        resultString =  boost::python::extract<string>(boost::python::str(resultObject));
#endif
        completed = true;
    }
    catch(pybind11::error_already_set const & ex) {
#ifdef CNOID_USE_PYBIND11
        pybind11::object stdout_ = sys.attr("stdout");
        pybind11::object strout = StringOutClass();
        sys.attr("stdout") = strout;
        pybind11::print(ex.what());
        sys.attr("stdout") = stdout_;
        pybind11::object st = strout.attr("text")();
        exceptionText = strout.attr("text")().cast<string>();
        resultString = exceptionText;
        hasException = true;
#else        
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
                    exceptionType = boost::python::object(boost::python::handle<>(boost::python::borrowed(ptype)));
                    exceptionTypeName = boost::python::extract<string>(boost::python::str(exceptionType));
                }
                if(pvalue){
                    exceptionValue = boost::python::object(boost::python::handle<>(boost::python::borrowed(pvalue)));
                }
                
                // get an error message by redirecting the output of PyErr_Print()
                boost::python::object stderr_ = sys.attr("stderr");
                boost::python::object strout = StringOutClass();
                sys.attr("stderr") = strout;
                PyErr_Restore(ptype, pvalue, ptraceback);
                PyErr_Print();
                sys.attr("stderr") = stderr_;
                exceptionText = boost::python::extract<string>(strout.attr("text")());

                resultObject = exceptionValue;
                resultString = exceptionText;
                hasException = true;
            }
        }
#endif
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
    pybind11::gil_scoped_acquire lock;
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
pybind11::object PythonExecutor::resultObject()
{
    impl->stateMutex.lock();
    pybind11::object object = impl->lastResultObject;
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
            pybind11::gil_scoped_acquire lock;
#ifdef CNOID_USE_PYBIND11
            pythonSysModule().attr("path").attr("remove")(scriptDirectory);
#else
            boost::python::list syspath = boost::python::extract<boost::python::list>(pythonSysModule().attr("path"));
            int n = boost::python::len(syspath);
            for(int i=0; i < n; ++i){
                string path = boost::python::extract<string>(syspath[i]);
                if(path == scriptDirectory){
                    syspath.pop(i);
                    break;
                }
            }
#endif
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
                pybind11::gil_scoped_acquire lock;
                PyThreadState_SetAsyncExc((long)threadId, exitExceptionType().ptr());
            }
            if(wait(20)){
                terminated = true;
                break;
            }
        }
        //releasePythonPathRef();
        
    } else if(isRunningForeground){
        pybind11::gil_scoped_acquire lock;
        PyErr_SetObject(exitExceptionType().ptr(), 0);
        //releasePythonPathRef();
#ifdef CNOID_USE_PYBIND11
        if(PyErr_Occurred()) throw pybind11::error_already_set();
#else
        boost::python::throw_error_already_set();
#endif
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
pybind11::object PythonExecutor::exceptionType() const
{
    impl->stateMutex.lock();
    pybind11::object exceptionType = impl->lastExceptionType;
    impl->stateMutex.unlock();
    return exceptionType;
}

        
/**
   \note GIL must be obtained when accessing this object.
*/
pybind11::object PythonExecutor::exceptionValue() const
{
    impl->stateMutex.lock();
    pybind11::object value = impl->lastExceptionValue;
    impl->stateMutex.unlock();
    return value;
}


bool PythonExecutor::isTerminated() const
{
    return impl->isTerminated;
}
