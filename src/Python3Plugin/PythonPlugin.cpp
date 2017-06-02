/**
   @author Shin'ichiro Nakaoka
*/

#include "PythonPlugin.h"
#include "PythonScriptItem.h"
#include "PythonConsoleView.h"
#include "PythonExecutor.h"
#include <cnoid/PyUtil>
#include <cnoid/Plugin>
#include <cnoid/AppConfig>
#include <cnoid/MenuManager>
#include <cnoid/ExecutablePath>
#include <cnoid/FileUtil>
#include <cnoid/MessageView>
#include <cnoid/OptionManager>
#include <cnoid/Archive>
#include <pybind11/functional.h>
#include <iostream>
#include "gettext.h"

using namespace std;
namespace stdph = std::placeholders;
using namespace cnoid;
using boost::format;
namespace py = pybind11;
namespace filesystem = boost::filesystem;

static const bool debugMode = true;

namespace {
    
py::module mainModule;
py::object mainNamespace;
py::object cnoidModule;
py::module sysModule;
py::object exitExceptionType;
MappingPtr pythonConfig;
Action* redirectionCheck;
Action* refreshModulesCheck;

list<string> additionalSearchPathList;

class MessageViewOut
{
public:
    void write(std::string const& text) {
        if(redirectionCheck->isChecked()){
            MessageView* mv = MessageView::instance();
            mv->put(text);
            mv->flush();
        } else {
            cout << text; cout.flush();
        }
    }
};

class MessageViewIn
{
public:
    py::object readline() {
        return py::str("\n");
    }
};
            

py::object pythonExit()
{
    PyErr_SetObject(exitExceptionType.ptr(), 0);
    if( PyErr_Occurred())
        throw py::error_already_set();
    return py::object();
}


class PythonPlugin : public Plugin
{
public:
    std::unique_ptr<PythonExecutor> executor_;
    py::object messageViewOut;
    py::object messageViewIn;
        
    PythonPlugin();
    virtual bool initialize();
    bool initializeInterpreter();
    virtual bool finalize();

    void onSigOptionsParsed(boost::program_options::variables_map& v);
    bool storeProperties(Archive& archive);
    void restoreProperties(const Archive& archive);

    PythonExecutor& executor() {
        if(!executor_){
            executor_.reset(new PythonExecutor);
        }
        return *executor_;
    }
};

};


namespace {
PythonPlugin* pythonPlugin = 0;
}


PythonPlugin::PythonPlugin()
    : Plugin("Python")
{
    pythonPlugin = this;
}


bool PythonPlugin::initialize()
{
    if(!initializeInterpreter()){
        return false;
    }

    pythonConfig = AppConfig::archive()->openMapping("Python");

    MenuManager& mm = menuManager();
    mm.setPath("/Options").setPath("Python");
    redirectionCheck = mm.addCheckItem(_("Redirectiton to MessageView"));
    redirectionCheck->setChecked(pythonConfig->get("redirectionToMessageView", true));
                                  
    refreshModulesCheck = mm.addCheckItem(_("Refresh modules in the script directory"));
    refreshModulesCheck->sigToggled().connect(
        std::bind(&PythonExecutor::setModuleRefreshEnabled, stdph::_1));
    if(pythonConfig->get("refreshModules", false)){
        refreshModulesCheck->setChecked(true);
    }

    PythonScriptItem::initializeClass(this);
    PythonConsoleView::initializeClass(this);
    
    OptionManager& opm = optionManager();
    opm.addOption("python,p", boost::program_options::value< vector<string> >(), "load a python script file");
    opm.sigOptionsParsed().connect(std::bind(&PythonPlugin::onSigOptionsParsed, this, stdph::_1));

    setProjectArchiver(
        std::bind(&PythonPlugin::storeProperties, this, stdph::_1),
        std::bind(&PythonPlugin::restoreProperties, this, stdph::_1));

    return true;
}


void PythonPlugin::onSigOptionsParsed(boost::program_options::variables_map& v)
{
    if (v.count("python")) {
        vector<string> pythonScriptFileNames = v["python"].as< vector<string> >();
        for(unsigned int i = 0; i < pythonScriptFileNames.size(); i++){
            MessageView::instance()->putln((format(_("Executing python script \"%1%\" ...")) % pythonScriptFileNames[i]).str());
            executor().execFile(pythonScriptFileNames[i]);
            if(!executor().hasException()){
                MessageView::instance()->putln(_("The script finished."));
            } else {
                MessageView::instance()->putln(_("Failed to run the python script."));
                py::gil_scoped_acquire lock;
                MessageView::instance()->put(executor().exceptionText());
            }
        }
    }
}


bool PythonPlugin::initializeInterpreter()
{
    Py_Initialize();

    mainModule = py::module::import("__main__");
    mainNamespace = mainModule.attr("__dict__");

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
#ifdef WIN32
    py::object env = py::module::import("os").attr("environ");
    env["PATH"] = py::str(executableDirectory() + ";") + env["PATH"];
#endif

    sysModule = py::module::import("sys");
    
    // set the choreonoid default python script path
    filesystem::path scriptPath =
        filesystem::path(executableTopDirectory()) / CNOID_PLUGIN_SUBDIR / "python3";
    sysModule.attr("path").attr("insert")(0, getNativePathString(scriptPath));

    if(debugMode)
        py::print(sysModule.attr("path"));

    // Redirect the stdout and stderr to the message view
    py::object messageViewOutClass =
        py::class_<MessageViewOut>(mainModule, "MessageViewOut")
        .def(py::init<>())
        .def("write", &MessageViewOut::write);
    messageViewOut = messageViewOutClass();
    sysModule.attr("stdout") = messageViewOut;
    sysModule.attr("stderr") = messageViewOut;

    // Disable waiting for input
    py::object messageViewInClass =
        py::class_<MessageViewIn>(mainModule, "MessageViewIn")
        .def(py::init<>())
        .def("readline", &MessageViewIn::readline);
    messageViewIn = messageViewInClass();
    sysModule.attr("stdin") = messageViewIn;

    // Override exit and quit
    py::object builtins = mainNamespace["__builtins__"];
    exitExceptionType = py::module::import("cnoid.PythonPlugin").attr("ExitException");
    py::object exitFunc = py::cpp_function(pythonExit);
    builtins.attr("exit") = exitFunc;
    builtins.attr("quit") = exitFunc;
    sysModule.attr("exit") = exitFunc;


    PyEval_InitThreads();
    PyEval_SaveThread();

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
        py::gil_scoped_acquire lock;
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
                    if(debugMode)
                        py::print(sysModule.attr("path"));
                    additionalSearchPathList.push_back(newPath);
                    mv->putln(format(_("PythonPlugin: \"%1%\" has been added to the Python module search path list."))
                              % newPath);
                }
            }
        }
    }
}
    

bool PythonPlugin::finalize()
{
    pythonConfig->write("redirectionToMessageView", redirectionCheck->isChecked());
    pythonConfig->write("refreshModules", refreshModulesCheck->isChecked());
    return true;
}


CNOID_IMPLEMENT_PLUGIN_ENTRY(PythonPlugin);


py::object cnoid::pythonMainModule()
{
    return mainModule;
}


py::object cnoid::pythonMainNamespace()
{
    return mainNamespace;
}


py::object cnoid::pythonSysModule()
{
    return sysModule;
}


bool cnoid::execPythonCode(const std::string& code)
{
    PythonExecutor& executor = pythonPlugin->executor();
    bool result = executor.execCode(code);
    if(executor.hasException()){
        py::gil_scoped_acquire lock;
        MessageView::instance()->putln(executor.exceptionText());
        result = false;
    }
    return result;
}

