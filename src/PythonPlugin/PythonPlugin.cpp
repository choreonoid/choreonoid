/**
   @author Shin'ichiro Nakaoka
*/

#include "PythonUtil.h"
#include "PythonScriptItem.h"
#include "PythonConsoleView.h"
#include "PythonExecutor.h"
#include <cnoid/Plugin>
#include <cnoid/AppConfig>
#include <cnoid/MenuManager>
#include <cnoid/ExecutablePath>
#include <cnoid/FileUtil>
#include <cnoid/MessageView>
#include <cnoid/OptionManager>
#include <cnoid/Archive>
#include <boost/python.hpp>
#include <iostream>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using namespace boost;


namespace {
    
python::object mainModule;
python::object mainNamespace;
python::object cnoidModule;
python::object sysModule;
python::object exitExceptionType;

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
    python::object readline() {
        return python::str("\n");
    }
};
            

python::object pythonExit()
{
    PyErr_SetObject(exitExceptionType.ptr(), 0);
    python::throw_error_already_set();
    return python::object();
}

class PythonPlugin : public Plugin
{
    python::object messageViewOut;
    python::object messageViewIn;
        
public:
    PythonPlugin();
    virtual bool initialize();
    bool initializeInterpreter();
    virtual bool finalize();

    void onSigOptionsParsed(boost::program_options::variables_map& v);
    bool storeProperties(Archive& archive);
    void restoreProperties(const Archive& archive);
};

};


PythonPlugin::PythonPlugin()
    : Plugin("Python")
{

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
    refreshModulesCheck->sigToggled().connect(boost::bind(&PythonExecutor::setModuleRefreshEnabled, _1));
    if(pythonConfig->get("refreshModules", false)){
        refreshModulesCheck->setChecked(true);
    }

    PythonScriptItem::initializeClass(this);
    PythonConsoleView::initializeClass(this);
    
    OptionManager& opm = optionManager();
    opm.addOption("python,p", program_options::value< vector<string> >(), "load a python script file");
    opm.sigOptionsParsed().connect(boost::bind(&PythonPlugin::onSigOptionsParsed, this, _1));

    setProjectArchiver(
        boost::bind(&PythonPlugin::storeProperties, this, _1),
        boost::bind(&PythonPlugin::restoreProperties, this, _1));

    return true;
}


void PythonPlugin::onSigOptionsParsed(boost::program_options::variables_map& v)
{
    if (v.count("python")) {
        try {
            PythonExecutor pyexec;
            vector<string> pythonScriptFileNames = v["python"].as< vector<string> >();
            for(unsigned int i = 0; i < pythonScriptFileNames.size(); i++){
                MessageView::instance()->putln((format(_("Loading python script file \"%1%\" ...")) % pythonScriptFileNames[i]).str());
                pyexec.execFile(pythonScriptFileNames[i]);
            }
        } catch (const std::exception& err) {
            MessageView::instance()->putln((format(_("%1%")) % err.what()).str());
        } catch (...) {
            MessageView::instance()->putln(format(_("Failed to run the python script file.")));
        }
    }
}


bool PythonPlugin::initializeInterpreter()
{
    Py_Initialize();

    mainModule = python::import("__main__");
    mainNamespace = mainModule.attr("__dict__");
    sysModule = python::import("sys");
    
    // set the choreonoid default python script path
    python::list syspath = python::extract<python::list>(sysModule.attr("path"));
    filesystem::path scriptPath =
        filesystem::path(executableTopDirectory()) / CNOID_PLUGIN_SUBDIR / "python";
    syspath.insert(0, getNativePathString(scriptPath));

    // Redirect the stdout and stderr to the message view
    python::object messageViewOutClass =
        python::class_<MessageViewOut>("MessageViewOut", python::init<>())
        .def("write", &MessageViewOut::write);
    messageViewOut = messageViewOutClass();
    sysModule.attr("stdout") = messageViewOut;
    sysModule.attr("stderr") = messageViewOut;

    // Disable waiting for input
    python::object messageViewInClass =
        python::class_<MessageViewIn>("MessageViewIn", python::init<>())
        .def("readline", &MessageViewIn::readline);
    messageViewIn = messageViewInClass();
    sysModule.attr("stdin") = messageViewIn;

    // Override exit and quit
    python::object builtins = mainNamespace["__builtins__"];
    exitExceptionType = python::import("cnoid.Python").attr("ExitException");
    python::object exitFunc = python::make_function(pythonExit);
    builtins.attr("exit") = exitFunc;
    builtins.attr("quit") = exitFunc;
    sysModule.attr("exit") = exitFunc;

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
        PyGILock lock;
        python::list syspath = python::extract<python::list>(sysModule.attr("path"));
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
                    syspath.insert(0, getNativePathString(filesystem::path(newPath)));
                    additionalSearchPathList.push_back(newPath);
                    mv->putln(format(_("PythonPlugin: \"%1%\" has been added to the python module search path list."))
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


boost::python::object cnoid::pythonMainModule()
{
    return mainModule;
}


boost::python::object cnoid::pythonMainNamespace()
{
    return mainNamespace;
}


boost::python::object cnoid::pythonSysModule()
{
    return sysModule;
}


boost::python::object cnoid::evalPythonExpression(const char* expression)
{
    return python::eval(expression, mainNamespace);
}


boost::python::object cnoid::execPythonFile(const std::string& filename)
{
    /*
      python::dict global;
      python::dict local;
      global["__builtins__"] = mainNamespace["__builtins__"];
      return python::exec_file(filename.c_str(), global, global);
    */
    return python::exec_file(filename.c_str(), mainNamespace);
}


boost::python::object cnoid::execPythonCode(const char* code)
{
    return python::exec(code, mainNamespace);
}


boost::python::object cnoid::execPythonCode(const std::string& code)
{
    return python::exec(code.c_str(), mainNamespace);
}
