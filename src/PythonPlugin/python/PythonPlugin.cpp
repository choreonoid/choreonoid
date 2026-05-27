#include "PythonPlugin.h"
#include "PythonConsoleView.h"
#include "PythonScriptItem.h"
#include "PythonExecutor.h"
#include "PyCApiUtil.h"
#include <cnoid/Plugin>
#include <cnoid/App>
#include <cnoid/AppConfig>
#include <cnoid/MenuManager>
#include <cnoid/MainMenu>
#include <cnoid/ViewManager>
#include <cnoid/ItemManager>
#include <cnoid/RootItem>
#include <cnoid/ExecutablePath>
#include <cnoid/UTF8>
#include <cnoid/MessageView>
#include <cnoid/OptionManager>
#include <cnoid/Archive>
#include <cnoid/Format>
#include <cnoid/PythonInterpreter>
#include <iostream>

#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

PythonPlugin* pythonPlugin = nullptr;

vector<string> scriptFilesToExecute;
vector<string> scriptFilesToLoad;

MappingPtr pythonConfig;
Action* redirectionCheck;
Action* refreshModulesCheck;
list<string> additionalSearchPathList;


// The stdout/stderr and stdin replacements are implemented as Python types
// created with PyType_FromSpec because nanobind does not provide a runtime
// class registration API equivalent to pybind11::class_.

struct SimplePyObject {
    PyObject_HEAD
};

PyObject* MessageViewOut_write(PyObject* /* self */, PyObject* args)
{
    const char* text = nullptr;
    if(!PyArg_ParseTuple(args, "s", &text)){
        return nullptr;
    }
    auto mv = MessageView::instance();
    if(redirectionCheck->isChecked()){
        mv->put(text);
        mv->flush();
    } else {
        cout << text; cout.flush();
    }
    Py_RETURN_NONE;
}

PyObject* MessageViewOut_flush(PyObject* /* self */, PyObject* /* args */)
{
    if(redirectionCheck->isChecked()){
        MessageView::instance()->flush();
    } else {
        cout.flush();
    }
    Py_RETURN_NONE;
}

PyMethodDef MessageViewOut_methods[] = {
    { "write", MessageViewOut_write, METH_VARARGS, nullptr },
    { "flush", MessageViewOut_flush, METH_NOARGS, nullptr },
    { nullptr, nullptr, 0, nullptr }
};

PyType_Slot MessageViewOut_slots[] = {
    { Py_tp_methods, MessageViewOut_methods },
    { 0, nullptr }
};

PyType_Spec MessageViewOut_spec = {
    "cnoid.PythonPlugin.MessageViewOut",
    sizeof(SimplePyObject),
    0,
    Py_TPFLAGS_DEFAULT,
    MessageViewOut_slots
};

PyObject* MessageViewIn_readline(PyObject* /* self */, PyObject* /* args */)
{
    return PyUnicode_FromString("\n");
}

PyMethodDef MessageViewIn_methods[] = {
    { "readline", MessageViewIn_readline, METH_NOARGS, nullptr },
    { nullptr, nullptr, 0, nullptr }
};

PyType_Slot MessageViewIn_slots[] = {
    { Py_tp_methods, MessageViewIn_methods },
    { 0, nullptr }
};

PyType_Spec MessageViewIn_spec = {
    "cnoid.PythonPlugin.MessageViewIn",
    sizeof(SimplePyObject),
    0,
    Py_TPFLAGS_DEFAULT,
    MessageViewIn_slots
};

PyObject* exitExceptionTypeForExitFunc = nullptr;

PyObject* exitFunc(PyObject* /* self */, PyObject* /* args */)
{
    PyErr_SetObject(exitExceptionTypeForExitFunc, nullptr);
    return nullptr;
}

PyMethodDef exitFunc_def = { "exit", exitFunc, METH_NOARGS, nullptr };

}

namespace cnoid {


class PythonPlugin::Impl
{
public:
    PythonPlugin* self;
    std::unique_ptr<PythonExecutor> executor_;
    PyObjectHandle exitExceptionType;
    PyObjectHandle messageViewOut;
    PyObjectHandle messageViewIn;
    PyObjectHandle rollbackImporterModule;

    Impl(PythonPlugin* self);
    ~Impl();

    bool initialize();

    // Sets up the Choreonoid-specific integration (MessageView redirection and
    // the exit/quit override) on top of the interpreter that has already been
    // initialized by the CnoidPythonInterpreter library.
    bool setupInterpreter();

    bool finalize();

    void onInputFileOptionsParsed(std::vector<std::string>& inputFiles);
    void onSigOptionsParsed();
    void executeScriptFileOnStartup(const string& scriptFile);
    bool storeProperties(Archive& archive);
    void restoreProperties(const Archive& archive);

    PythonExecutor& executor() {
        if(!executor_){
            executor_.reset(new PythonExecutor);
        }
        return *executor_;
    }
};

}


PythonPlugin* PythonPlugin::instance()
{
    return pythonPlugin;
}


PythonPlugin::PythonPlugin()
    : Plugin("Python")
{
    impl = new Impl(this);
    pythonPlugin = this;
}


PythonPlugin::Impl::Impl(PythonPlugin* self)
    : self(self)
{

}


PythonPlugin::~PythonPlugin()
{
    delete impl;
}


PythonPlugin::Impl::~Impl()
{
    // The interpreter is finalized by the CnoidPythonInterpreter library as the very last
    // step of the App shutdown, not here. See src/Python/PythonInterpreter.cpp
    // and App::Impl::exec() in src/Base/App.cpp.
}


bool PythonPlugin::initialize()
{
    return impl->initialize();
}


bool PythonPlugin::Impl::initialize()
{
    if(!isPythonAvailable()){
        MessageView::instance()->putln(
            _("The embedded Python interpreter is not available, so the Python plugin is disabled."),
            MessageView::Error);
        return false;
    }

    pythonConfig = AppConfig::archive()->openMapping("Python");

    if(auto optionsMenu = MainMenu::instance()->get_Options_Menu()){
        MenuManager& mm = self->menuManager();
        mm.setCurrent(optionsMenu).setPath(N_("Python"));
        redirectionCheck = mm.addCheckItem(_("Redirectiton to MessageView"));
        refreshModulesCheck = mm.addCheckItem(_("Refresh modules in the script directory"));
    } else {
        redirectionCheck = new Action;
        refreshModulesCheck = new Action;
    }
    redirectionCheck->setChecked(pythonConfig->get("redirection_to_message_view", true));
    refreshModulesCheck->sigToggled().connect(&PythonExecutor::setModuleRefreshEnabled);

    if(pythonConfig->get("refresh_modules", false)){
        refreshModulesCheck->setChecked(true);
    }

    if(!setupInterpreter()){
        return false;
    }

    PythonScriptItem::initializeClass(self);
    PythonConsoleView::initializeClass(self);

    auto om = OptionManager::instance();
    om->add_option("--python,-p", scriptFilesToExecute, "execute a python script");
    om->add_option("--python-item", scriptFilesToLoad, "load a python script as an item");

    om->sigInputFileOptionsParsed(1).connect(
        [&](std::vector<std::string>& inputFiles){ onInputFileOptionsParsed(inputFiles); });

    om->sigOptionsParsed(1).connect(
        [&](OptionManager*){ onSigOptionsParsed(); });

    self->setProjectArchiver(
        [&](Archive& archive){ return storeProperties(archive); },
        [&](const Archive& archive){ restoreProperties(archive); });

    return true;
}


void PythonPlugin::Impl::onInputFileOptionsParsed(std::vector<std::string>& inputFiles)
{
    auto iter = inputFiles.begin();
    while(iter != inputFiles.end()){
        if(filesystem::path(fromUTF8(*iter)).extension().string() == ".py"){
            executeScriptFileOnStartup(*iter);
            iter = inputFiles.erase(iter);
        } else {
            ++iter;
        }
    }
}


void PythonPlugin::Impl::onSigOptionsParsed()
{
    for(auto& script : scriptFilesToExecute){
        executeScriptFileOnStartup(script);
    }
    for(auto& script : scriptFilesToLoad){
        PythonScriptItemPtr item = new PythonScriptItem;
        auto rootItem = RootItem::instance();
        if(item->load(script, rootItem)){
            rootItem->addChildItem(item);
        }
        item->setChecked(true);
    }
}


void PythonPlugin::Impl::executeScriptFileOnStartup(const string& scriptFile)
{
    MessageView::instance()->putln(formatR(_("Executing python script \"{}\" ..."), scriptFile));

    auto& exec = executor();
    exec.execFile(scriptFile);
    if(!exec.hasException() || exec.isTerminated()){
        MessageView::instance()->putln(_("The script finished."));
    } else {
        MessageView::instance()->putln(_("Failed to run the python script."), MessageView::Error);
        GilScopedAcquire lock;
        MessageView::instance()->put(executor().exceptionText());
    }

    App::checkErrorAndExitIfTestMode();
}


bool PythonPlugin::Impl::setupInterpreter()
{
    // The interpreter itself has already been initialized by the
    // CnoidPythonInterpreter library. Here we only add the Choreonoid-specific
    // integration. That library released the GIL after initializing the
    // interpreter, so it must be re-acquired while these objects are created.
    GilScopedAcquire lock;

    PyObject* main = cnoid::pythonMainModule();
    PyObject* g = cnoid::pythonGlobalNamespace();
    PyObject* sys = cnoid::pythonSysModule();
    if(!main || !g || !sys){
        return false;
    }

    // Redirect the stdout and stderr to the message view
    PyObject* messageViewOutType = PyType_FromSpec(&MessageViewOut_spec);
    if(!messageViewOutType){
        return false;
    }
    messageViewOut = PyObjectHandle::steal(PyObject_CallObject(messageViewOutType, nullptr));
    Py_DECREF(messageViewOutType);
    PyObject_SetAttrString(sys, "stdout", messageViewOut.get());
    PyObject_SetAttrString(sys, "stderr", messageViewOut.get());

    // Disable waiting for input
    PyObject* messageViewInType = PyType_FromSpec(&MessageViewIn_spec);
    if(!messageViewInType){
        return false;
    }
    messageViewIn = PyObjectHandle::steal(PyObject_CallObject(messageViewInType, nullptr));
    Py_DECREF(messageViewInType);
    PyObject_SetAttrString(sys, "stdin", messageViewIn.get());

    {
        PyObjectHandle r = PyObjectHandle::steal(
            PyRun_String("class ExitException (Exception): pass\n", Py_single_input, g, g));
        exitExceptionType = PyObjectHandle::steal(PyObject_GetAttrString(main, "ExitException"));
        PyObjectHandle r2 = PyObjectHandle::steal(
            PyRun_String("del ExitException\n", Py_single_input, g, g));
    }

    exitExceptionTypeForExitFunc = exitExceptionType.get();
    PyObject* exitFuncObj = PyCFunction_NewEx(&exitFunc_def, nullptr, nullptr);

    // Override exit and quit
    PyObject* builtins = PyDict_GetItemString(g, "__builtins__");
    if(builtins){
        // __builtins__ may be a module or its dict depending on the context.
        if(PyModule_Check(builtins)){
            PyObject_SetAttrString(builtins, "exit", exitFuncObj);
            PyObject_SetAttrString(builtins, "quit", exitFuncObj);
        } else {
            PyDict_SetItemString(builtins, "exit", exitFuncObj);
            PyDict_SetItemString(builtins, "quit", exitFuncObj);
        }
    }
    PyObject_SetAttrString(sys, "exit", exitFuncObj);
    Py_XDECREF(exitFuncObj);

    return true;
}


bool PythonPlugin::finalize()
{
    pythonPlugin = nullptr;

    pythonConfig->write("redirection_to_message_view", redirectionCheck->isChecked());
    pythonConfig->write("refresh_modules", refreshModulesCheck->isChecked());

    // Views and items defined in this plugin must be deleted before finalizing the Python interpreter
    // because the views and items have their own python objects
    viewManager().deleteView(ViewManager::findView<PythonConsoleView>());
    itemManager().detachAllManagedTypeItemsFromRoot();

    // Objects managed by this plugin (e.g. tool bars mounted from a Python script) may hold
    // Python callbacks connected to Qt signals. They must be deleted here while the Python
    // interpreter is still valid; otherwise they would be deleted in the base ExtensionManager
    // destructor after the interpreter has been finalized, and destroying the held callbacks
    // would access the finalized interpreter and crash.
    deleteManagedObjects();

    return true;
}


bool PythonPlugin::Impl::storeProperties(Archive& archive)
{
    if(!additionalSearchPathList.empty()){
        Listing& pathListing = *archive.openListing("moduleSearchPath");
        for(auto& path : additionalSearchPathList){
            pathListing.append(archive.getRelocatablePath(path));
        }
        return true;
    }
    return false;
}


void PythonPlugin::Impl::restoreProperties(const Archive& archive)
{
    Listing& pathListing = *archive.findListing("moduleSearchPath");
    if(pathListing.isValid()){
        MessageView* mv = MessageView::instance();
        GilScopedAcquire lock;
        string newPath;
        for(auto& path : pathListing){
            newPath = archive.resolveRelocatablePath(path->toString());
            if(!newPath.empty()){
                bool isExisting = false;
                for(auto& existingPath : additionalSearchPathList){
                    if(newPath == existingPath){
                        isExisting = true;
                        break;
                    }
                }
                if(!isExisting){
                    filesystem::path nativePath(fromUTF8(newPath));
                    PyObject* sysPath = PyObject_GetAttrString(cnoid::pythonSysModule(), "path");
                    if(sysPath){
                        PyObject* dir = PyUnicode_FromString(nativePath.make_preferred().string().c_str());
                        PyObject* r = PyObject_CallMethod(sysPath, "insert", "iO", 0, dir);
                        Py_XDECREF(r);
                        Py_XDECREF(dir);
                        Py_DECREF(sysPath);
                    }
                    additionalSearchPathList.push_back(newPath);
                    mv->putln(
                        formatR(_("PythonPlugin: \"{}\" has been added to the Python module search path list."),
                                newPath));
                }
            }
        }
    }
}


PyObject* PythonPlugin::mainModule()
{
    return cnoid::pythonMainModule();
}

PyObject* PythonPlugin::globalNamespace()
{
    return cnoid::pythonGlobalNamespace();
}

PyObject* PythonPlugin::sysModule()
{
    return cnoid::pythonSysModule();
}

PyObject* PythonPlugin::exitException()
{
    return impl->exitExceptionType.get();
}

PyObject* PythonPlugin::rollbackImporterModule()
{
    if(!impl->rollbackImporterModule){
        impl->rollbackImporterModule = PyObjectHandle::steal(PyImport_ImportModule("cnoid.rbimporter"));
    }
    return impl->rollbackImporterModule.get();
}


CNOID_IMPLEMENT_PLUGIN_ENTRY(PythonPlugin);
