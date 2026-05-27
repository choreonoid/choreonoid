#include "PythonExecutor.h"
#include "PythonPlugin.h"
#include "PyCApiUtil.h"
#include <cnoid/LazyCaller>
#include <cnoid/UTF8>
#include <cnoid/FileUtil>
#include <filesystem>
#include <QThread>
#include <QMutex>
#include <QWaitCondition>
#include <functional>
#include <map>
#include <climits>
#include <cstdio>

using namespace std;
using namespace cnoid;

namespace {

bool isDefaultModuleRefreshEnabled = false;

typedef map<string, int> PathRefMap;
PathRefMap additionalPythonPathRefMap;

}

namespace cnoid {

class PythonExecutor::Impl : public QThread
{
public:
    PythonPlugin* pythonPlugin;
    bool isBackgroundMode;
    bool isRunningForeground;
    bool isModuleRefreshEnabled;
    PythonExecutor::NamespaceMode namespaceMode;
    bool isNamespaceClearedOnExecution;
    PyObjectHandle individualNamespace;
    std::function<PyObjectHandle(PyObject* globalNamespace)> functionToExecScript;
    PyObject* currentGlobalNamespace;
    Qt::HANDLE threadId;
    mutable QMutex stateMutex;
    QWaitCondition stateCondition;
    PyObjectHandle returnValue;
    Signal<void()> sigFinished;

    string scriptDirectory;
    PathRefMap::iterator pathRefIter;

    bool hasException;
    string exceptionTypeName;
    string exceptionText;
    PyObjectHandle exceptionType;
    PyObjectHandle exceptionValue;

    PyObjectHandle lastReturnValue;
    PyObjectHandle lastExceptionType;
    PyObjectHandle lastExceptionValue;
    string lastExceptionTypeName;
    string lastExceptionText;
    bool isTerminated;

    Impl();
    Impl(const Impl& org);
    void resetLastResultObjects();
    ~Impl();
    PyObjectHandle createIndividualNamespace();
    PyObject* currentNamespace();
    PythonExecutor::State state() const;
    bool exec(std::function<PyObjectHandle(PyObject*)> execScript, const string& filename);
    bool execMain(std::function<PyObjectHandle(PyObject*)> execScript);
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
    impl = new Impl();
}


PythonExecutor::Impl::Impl()
    : pythonPlugin(PythonPlugin::instance())
{
    isBackgroundMode = false;
    isRunningForeground = false;
    isModuleRefreshEnabled = isDefaultModuleRefreshEnabled;
    namespaceMode = PythonExecutor::SharedNamespace;
    isNamespaceClearedOnExecution = false;
    hasException = false;
    isTerminated = false;

    resetLastResultObjects();
}


PythonExecutor::PythonExecutor(const PythonExecutor& org)
{
    impl = new Impl(*org.impl);
}


PythonExecutor::Impl::Impl(const Impl& org)
    : pythonPlugin(PythonPlugin::instance())
{
    isBackgroundMode = org.isBackgroundMode;
    isRunningForeground = false;
    isModuleRefreshEnabled = isDefaultModuleRefreshEnabled;
    namespaceMode = org.namespaceMode;
    isNamespaceClearedOnExecution = org.isNamespaceClearedOnExecution;
    hasException = false;
    isTerminated = false;

    resetLastResultObjects();
}


void PythonExecutor::Impl::resetLastResultObjects()
{
    lastReturnValue.reset();
    lastExceptionType.reset();
    lastExceptionValue.reset();
}


PythonExecutor::~PythonExecutor()
{
    delete impl;
}


PythonExecutor::Impl::~Impl()
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


void PythonExecutor::setNamespaceMode(NamespaceMode mode)
{
    if(mode != impl->namespaceMode){
        impl->namespaceMode = mode;
        // The cached individual namespace is discarded so that switching modes
        // does not carry over a stale namespace.
        GilScopedAcquire lock;
        impl->individualNamespace.reset();
    }
}


PythonExecutor::NamespaceMode PythonExecutor::namespaceMode() const
{
    return impl->namespaceMode;
}


void PythonExecutor::setNamespaceClearedOnExecution(bool on)
{
    impl->isNamespaceClearedOnExecution = on;
}


bool PythonExecutor::isNamespaceClearedOnExecution() const
{
    return impl->isNamespaceClearedOnExecution;
}


PyObjectHandle PythonExecutor::Impl::createIndividualNamespace()
{
    PyObjectHandle ns = PyObjectHandle::steal(PyDict_New());
    if(ns){
        // Put the minimum entries so that a script runs in the individual
        // namespace just as it does in the shared namespace on the first run:
        // the builtins (print, len, etc.) and __name__ set to "__main__" so
        // that the "if __name__ == '__main__':" idiom works.
        PyObject* shared = pythonPlugin->globalNamespace();
        PyObject* builtins = PyDict_GetItemString(shared, "__builtins__");
        if(builtins){
            PyDict_SetItemString(ns.get(), "__builtins__", builtins);
        }
        PyObjectHandle mainName = PyObjectHandle::steal(PyUnicode_FromString("__main__"));
        PyDict_SetItemString(ns.get(), "__name__", mainName.get());
    }
    return ns;
}


PyObject* PythonExecutor::Impl::currentNamespace()
{
    if(namespaceMode == PythonExecutor::SharedNamespace){
        return pythonPlugin->globalNamespace();
    }
    if(!individualNamespace || isNamespaceClearedOnExecution){
        individualNamespace = createIndividualNamespace();
    }
    return individualNamespace.get();
}


PythonExecutor::State PythonExecutor::Impl::state() const
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


bool PythonExecutor::eval(const std::string& code)
{
    return impl->exec(
        [code](PyObject* g){
            return PyObjectHandle::steal(PyRun_String(code.c_str(), Py_eval_input, g, g)); },
        "");
}


bool PythonExecutor::execCode(const std::string& code)
{
    return impl->exec(
        [code](PyObject* g){
            return PyObjectHandle::steal(PyRun_String(code.c_str(), Py_file_input, g, g)); },
        "");
}


bool PythonExecutor::execFile(const std::string& filename)
{
    return impl->exec(
        [filename](PyObject* g){
            PyObjectHandle result;
            FILE* fp = fopen(filename.c_str(), "r");
            if(!fp){
                PyErr_SetFromErrnoWithFilename(PyExc_IOError, filename.c_str());
            } else {
                result = PyObjectHandle::steal(
                    PyRun_FileEx(fp, filename.c_str(), Py_file_input, g, g, 1));
            }
            return result; },
        filename);
}


bool PythonExecutor::Impl::exec(std::function<PyObjectHandle(PyObject*)> execScript, const string& filename)
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
        filepath = filesystem::absolute(fromUTF8(filename));
        scriptDirectory = toUTF8(filepath.parent_path().string());
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
        GilScopedAcquire lock;

        // clear exception variables
        hasException = false;
        exceptionTypeName.clear();
        exceptionText.clear();
        exceptionType.reset();
        exceptionValue.reset();

        isTerminated = false;

        functionToExecScript = execScript;

        // Determine the global namespace once for this execution so that the
        // same namespace is used for setting __file__ and for running the
        // script. When the individual namespace is cleared on every execution,
        // calling currentNamespace() more than once would create a different
        // namespace each time.
        currentGlobalNamespace = currentNamespace();

        if(doAddPythonPath){
            PyObject* path = PyObject_GetAttrString(pythonPlugin->sysModule(), "path");
            if(path){
                PyObject* dir = PyUnicode_FromString(scriptDirectory.c_str());
                PyObject* r = PyObject_CallMethod(path, "insert", "iO", 0, dir);
                Py_XDECREF(r);
                Py_XDECREF(dir);
                Py_DECREF(path);
            }
        }

        if(isModuleRefreshEnabled){
            PyObject* r = PyObject_CallMethod(
                pythonPlugin->rollbackImporterModule(), "refresh", "s", scriptDirectory.c_str());
            Py_XDECREF(r);
        }

        if(!filename.empty()){
            if(auto relativePath = getRelativePath(filepath, filesystem::current_path())){
                PyObject* g = currentGlobalNamespace;
                PyObject* file = PyUnicode_FromString(toUTF8(relativePath->string()).c_str());
                PyDict_SetItemString(g, "__file__", file);
                Py_XDECREF(file);
            }
        }

        if(!isBackgroundMode){
            stateMutex.lock();
            threadId = currentThreadId();
            isRunningForeground = true;
            stateMutex.unlock();

            result = execMain(functionToExecScript);
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


bool PythonExecutor::Impl::execMain(std::function<PyObjectHandle(PyObject*)> execScript)
{
    bool completed = false;
    returnValue.reset();

    returnValue = execScript(currentGlobalNamespace);
    if(returnValue){
        completed = true;
    } else if(PyErr_Occurred()){
        bool isExitRequested = PyErr_ExceptionMatches(pythonPlugin->exitException());
        PyObjectHandle traceback;
        exceptionText = fetchPythonExceptionText(exceptionType, exceptionValue, traceback);
        hasException = true;
        if(isExitRequested){
            isTerminated = true;
        }
    }

    releasePythonPathRef();

    stateMutex.lock();
    isRunningForeground = false;
    lastReturnValue = returnValue;
    lastExceptionType = exceptionType;
    lastExceptionValue = exceptionValue;
    lastExceptionTypeName = exceptionTypeName;
    lastExceptionText = exceptionText;
    stateCondition.wakeAll();
    stateMutex.unlock();

    if(QThread::isRunning()){
        callLater([this](){ onBackgroundExecutionFinished(); });
    } else {
        sigFinished();
    }

    return completed;
}


void PythonExecutor::Impl::run()
{
    stateMutex.lock();
    threadId = currentThreadId();
    stateCondition.wakeAll();
    stateMutex.unlock();
    GilScopedAcquire lock;
    execMain(functionToExecScript);
}


bool PythonExecutor::waitToFinish(double timeout)
{
    return impl->waitToFinish(timeout);
}


bool PythonExecutor::Impl::waitToFinish(double timeout)
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


void PythonExecutor::Impl::onBackgroundExecutionFinished()
{
    sigFinished();
}


void PythonExecutor::Impl::releasePythonPathRef()
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
            GilScopedAcquire lock;
            PyObject* path = PyObject_GetAttrString(pythonPlugin->sysModule(), "path");
            if(path){
                PyObject* dir = PyUnicode_FromString(scriptDirectory.c_str());
                PyObject* r = PyObject_CallMethod(path, "remove", "O", dir);
                Py_XDECREF(r);
                Py_XDECREF(dir);
                Py_DECREF(path);
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


bool PythonExecutor::Impl::terminateScript()
{
    bool terminated = true;

    if(QThread::isRunning()){
        terminated = false;

        for(int i=0; i < 400; ++i){
            {
                GilScopedAcquire lock;

                /**
                   Set the exception class itself instead of an instance of the exception class
                   because the following function only accepts a single parameter with regard to the
                   exception object in constrast to PyErr_SetObject that takes both the type and value
                   of the exeption, and if the instance is given to the following function, the
                   exception type will be unknown in the exception handler, which makes it impossible
                   for the handler to check if the termination is requested. By giving the class object,
                   the handler can detect the exception type even in this case.
                */
                PyThreadState_SetAsyncExc(
                    reinterpret_cast<uintptr_t>(threadId), pythonPlugin->exitException());
            }
            if(wait(20)){
                terminated = true;
                break;
            }
        }
        releasePythonPathRef();

    } else if(isRunningForeground){
        GilScopedAcquire lock;
        PyErr_SetObject(pythonPlugin->exitException(), 0);
        releasePythonPathRef();
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


bool PythonExecutor::isTerminated() const
{
    return impl->isTerminated;
}
