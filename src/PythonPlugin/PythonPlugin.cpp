/**
   @author Shin'ichiro Nakaoka
nnn*/

#include "PythonConsoleView.h"
#include "PythonScriptItem.h"
#include "PythonExecutor.h"
#include <cnoid/PyUtil>
#include <cnoid/Plugin>
#include <cnoid/AppConfig>
#include <cnoid/MenuManager>
#include <cnoid/ViewManager>
#include <cnoid/ItemManager>
#include <cnoid/ExecutablePath>
#include <cnoid/FileUtil>
#include <cnoid/MessageView>
#include <cnoid/OptionManager>
#include <cnoid/Archive>
#include <pybind11/embed.h>
#include <fmt/format.h>
#include <iostream>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;
namespace filesystem = cnoid::stdx::filesystem;

namespace {

MappingPtr pythonConfig;
Action* redirectionCheck;
Action* refreshModulesCheck;

list<string> additionalSearchPathList;

class MessageViewOut
{
    MessageView* mv;
public:
    MessageViewOut() : mv(MessageView::instance()) { }
    
    void write(std::string const& text) {
        if(redirectionCheck->isChecked()){
            mv->put(text);
            mv->flush();
        } else {
            cout << text; cout.flush();
        }
    }

    void flush(){
        if(redirectionCheck->isChecked()){
            mv->flush();
        } else {
            cout.flush();
        }
    }
};

class MessageViewIn
{
public:
    python::object readline() {
        return python::str("\n");
    }
};
            

class PythonPlugin : public Plugin
{
public:
    unique_ptr<pybind11::scoped_interpreter> interpreter;
    unique_ptr<pybind11::gil_scoped_release> gil_scoped_release;
    std::unique_ptr<PythonExecutor> executor_;
    python::module mainModule;
    python::object globalNamespace;
    python::object cnoidModule;
    python::module sysModule;
    python::object exitExceptionType;
    python::object messageViewOut;
    python::object messageViewIn;
    python::module rollbackImporterModule;

    PythonPlugin();
    virtual bool initialize();
    bool initializeInterpreter();
    virtual bool finalize();

    void onInputFileOptionsParsed(std::vector<std::string>& inputFiles);
    void onSigOptionsParsed(boost::program_options::variables_map& v);
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


PythonPlugin* pythonPlugin = 0;

python::object pythonExit()
{
    PyErr_SetObject(pythonPlugin->exitExceptionType.ptr(), 0);
    
    if(PyErr_Occurred()){
        throw pybind11::error_already_set();
    }
    
    return python::object();
}

}


PythonPlugin::PythonPlugin()
    : Plugin("Python")
{
    pythonPlugin = this;
}


bool PythonPlugin::initialize()
{
    pythonConfig = AppConfig::archive()->openMapping("Python");

    MenuManager& mm = menuManager();
    mm.setPath("/Options").setPath("Python");
    redirectionCheck = mm.addCheckItem(_("Redirectiton to MessageView"));
    redirectionCheck->setChecked(pythonConfig->get("redirectionToMessageView", true));

    refreshModulesCheck = mm.addCheckItem(_("Refresh modules in the script directory"));
    refreshModulesCheck->sigToggled().connect(&PythonExecutor::setModuleRefreshEnabled);
    if(pythonConfig->get("refreshModules", false)){
        refreshModulesCheck->setChecked(true);
    }

    if(!initializeInterpreter()){
        return false;
    }

    PythonScriptItem::initializeClass(this);
    PythonConsoleView::initializeClass(this);
    
    OptionManager& opm = optionManager();
    opm.addOption("python,p", boost::program_options::value< vector<string> >(), "load a python script file");
    opm.sigInputFileOptionsParsed(1).connect(
        [&](std::vector<std::string>& inputFiles){ onInputFileOptionsParsed(inputFiles); });
    opm.sigOptionsParsed(1).connect(
        [&](boost::program_options::variables_map& v){ onSigOptionsParsed(v); });

    setProjectArchiver(
        [&](Archive& archive){ return storeProperties(archive); },
        [&](const Archive& archive){ restoreProperties(archive); });

    return true;
}


void PythonPlugin::onInputFileOptionsParsed(std::vector<std::string>& inputFiles)
{
    auto iter = inputFiles.begin();
    while(iter != inputFiles.end()){
        if(getExtension(*iter) == "py"){
            executeScriptFileOnStartup(*iter);
            iter = inputFiles.erase(iter);
        } else {
            ++iter;
        }
    }
}


void PythonPlugin::onSigOptionsParsed(boost::program_options::variables_map& v)
{
    if(v.count("python")){
        vector<string> pythonScriptFileNames = v["python"].as<vector<string>>();
        for(unsigned int i = 0; i < pythonScriptFileNames.size(); i++){
            executeScriptFileOnStartup(pythonScriptFileNames[i]);
        }
    }
}


void PythonPlugin::executeScriptFileOnStartup(const string& scriptFile)
{
    MessageView::instance()->putln(format(_("Executing python script \"{}\" ..."), scriptFile));
    executor().execFile(scriptFile);
    if(!executor().hasException()){
        MessageView::instance()->putln(_("The script finished."));
    } else {
        MessageView::instance()->putln(MessageView::WARNING, _("Failed to run the python script."));
        python::gil_scoped_acquire lock;
        MessageView::instance()->put(executor().exceptionText());
    }
}


bool PythonPlugin::initializeInterpreter()
{
    interpreter.reset(new pybind11::scoped_interpreter(false));

    /*
      Some python module requires argv and missing argv may cause AttributeError.a
      (Ex. AttributeError: 'module' object has no attribute 'argv')
      To avoid this problem, set dummy argv to python interpreter by PySys_SetArgvEx.
    */
#ifdef CNOID_USE_PYTHON2
    char dummy_str[] = "choreonoid"; // avoid deprecated conversion from string constant
    char* dummy_argv[] = {dummy_str};
#else
    wchar_t dummy_str[] = L"choreonoid"; // avoid deprecated conversion from string constant
    wchar_t* dummy_argv[] = {dummy_str};
#endif
    PySys_SetArgvEx(1, dummy_argv, 0);

    mainModule = python::module::import("__main__");
    globalNamespace = mainModule.attr("__dict__");

	/*
	 In Windows, the bin directory must be added to the PATH environment variable
	 so that the DLL in the directory can be loaded in loading Python modules.
	 Note that the corresponding Python variable must be updated instead of using C functions
	 because the Python caches the environment variables and updates the OS variables when
	 the cached variable is updated and the variable values updated using C functions are
	 discarded at that time. For example, the numpy module also updates the PATH variable
	 using the Python variable, and it invalidates the updated PATH value if the value is
	 set using C functions.
	*/	
#ifdef _WIN32
    python::module env = python::module::import("os").attr("environ");
    env["PATH"] = python::str(executableDirectory() + ";" + std::string(python::str(env["PATH"])));
#endif

    sysModule = python::module::import("sys");

    sysModule.attr("dont_write_bytecode") = true;
    
    // set the choreonoid default python script path
    filesystem::path scriptPath = filesystem::path(executableTopDirectory()) / CNOID_PLUGIN_SUBDIR / "python";

    sysModule.attr("path").attr("insert")(0, getNativePathString(scriptPath));

    // Redirect the stdout and stderr to the message view
    python::object messageViewOutClass =
        pybind11::class_<MessageViewOut>(mainModule, "MessageViewOut").def(pybind11::init<>())
        .def("write", &MessageViewOut::write)
        .def("flush", &MessageViewOut::flush);
    
    messageViewOut = messageViewOutClass();
    sysModule.attr("stdout") = messageViewOut;
    sysModule.attr("stderr") = messageViewOut;

    // Disable waiting for input
    python::object messageViewInClass =
        pybind11::class_<MessageViewIn>(mainModule, "MessageViewIn").def(pybind11::init<>())
        .def("readline", &MessageViewIn::readline);
    messageViewIn = messageViewInClass();
    sysModule.attr("stdin") = messageViewIn;

    pybind11::eval<pybind11::eval_single_statement>("class ExitException (Exception): pass\n");
    exitExceptionType = mainModule.attr("ExitException");
    pybind11::eval<pybind11::eval_single_statement>("del ExitException\n");
    pybind11::function exitFunc = pybind11::cpp_function(pythonExit);

    // Override exit and quit
    python::object builtins = globalNamespace["__builtins__"];
    builtins.attr("exit") = exitFunc;
    builtins.attr("quit") = exitFunc;
    sysModule.attr("exit") = exitFunc;

    gil_scoped_release.reset(new pybind11::gil_scoped_release());

    return true;
}


bool PythonPlugin::storeProperties(Archive& archive)
{
    if(!additionalSearchPathList.empty()){
        Listing& pathListing = *archive.openListing("moduleSearchPath");
        list<string>::iterator p;
        for(p = additionalSearchPathList.begin(); p != additionalSearchPathList.end(); ++p){
            pathListing.append(archive.getRelocatablePath(*p));
        }
        return true;
    }
    return false;
}


void PythonPlugin::restoreProperties(const Archive& archive)
{
    Listing& pathListing = *archive.findListing("moduleSearchPath");
    if(pathListing.isValid()){
        MessageView* mv = MessageView::instance();
        python::gil_scoped_acquire lock;
        string newPath;
        for(int i=0; i < pathListing.size(); ++i){
            newPath = archive.resolveRelocatablePath(pathListing[i].toString());
            if(!newPath.empty()){
                bool isExisting = false;
                list<string>::iterator p;
                for(p = additionalSearchPathList.begin(); p != additionalSearchPathList.end(); ++p){
                    if(newPath == (*p)){
                        isExisting = true;
                        break;
                    }
                }
                if(!isExisting){
                    sysModule.attr("path").attr("insert")(0, getNativePathString(filesystem::path(newPath)));
                    additionalSearchPathList.push_back(newPath);
                    mv->putln(
                        format(_("PythonPlugin: \"{}\" has been added to the Python module search path list."),
                               newPath));
                }
            }
        }
    }
}
    

bool PythonPlugin::finalize()
{
    pythonConfig->write("redirectionToMessageView", redirectionCheck->isChecked());
    pythonConfig->write("refreshModules", refreshModulesCheck->isChecked());

    // Views and items defined in this plugin must be deleted before finalizing the Python interpreter
    // because the views and items have their own python objects
    viewManager().deleteView(PythonConsoleView::instance());
    itemManager().detachAllManagedTypeItemsFromRoot();
    
    return true;
}


CNOID_IMPLEMENT_PLUGIN_ENTRY(PythonPlugin);

namespace cnoid {

python::module getMainModule()
{
    return pythonPlugin->mainModule;
}

python::object getGlobalNamespace()
{
    return pythonPlugin->globalNamespace;
}

python::module getSysModule()
{
    return pythonPlugin->sysModule;
}

python::object getExitException()
{
    return pythonPlugin->exitExceptionType;
}

python::module getRollbackImporterModule()
{
    if(!pythonPlugin->rollbackImporterModule){
        pythonPlugin->rollbackImporterModule = python::module::import("cnoid.rbimporter");
    }
    return pythonPlugin->rollbackImporterModule;
}

} // namespace cnoid
