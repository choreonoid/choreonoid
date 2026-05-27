#include "PythonInterpreter.h"
#include <cnoid/ExecutablePath>
#include <cnoid/UTF8>
#include <cnoid/MessageOut>
#include <cnoid/Format>
#include <filesystem>
#include <string>

#ifdef __linux__
#include <dlfcn.h>
#include <link.h>
#include <regex>
#endif

using namespace std;
using namespace cnoid;

namespace {

namespace filesystem = std::filesystem;

bool isAvailable = false;
PyThreadState* mainThreadState = nullptr;

PyObject* mainModule_ = nullptr;
PyObject* globalNamespace_ = nullptr; // borrowed from mainModule
PyObject* sysModule_ = nullptr;


#ifdef __linux__

/*
  The link_map structure instance obtained by dlinfo with RTLD_DI_LINKMAP
  has both the prev and next instances, and the instance obtained by the function
  is a particular position in the global link_map chain. The position is determined
  by the handle given to the function. Therefore, to find the information on a
  particular shared library, you have to search in both the directions from the obtained
  link_map instance.
*/
char* findLibPython(const string& filePath)
{
    auto handle = dlopen(filePath.c_str(), RTLD_LAZY);
    if(handle == nullptr){
        return nullptr;
    }
    struct link_map* linkMap;
    if(dlinfo(handle, RTLD_DI_LINKMAP, &linkMap) != 0){
        return nullptr;
    }

    char* libPythonName = nullptr;
    struct link_map* formerLinkMap = linkMap;
    struct link_map* latterLinkMap = linkMap->l_next;
    regex pattern(".*libpython.+\\.so.*");
    std::cmatch match;

    while(true){
        if(linkMap == formerLinkMap){
            if(formerLinkMap){
                formerLinkMap = formerLinkMap->l_prev;
            }
            if(latterLinkMap){
                linkMap = latterLinkMap;
            } else {
                linkMap = formerLinkMap;
            }
        } else {
            if(latterLinkMap){
                latterLinkMap = latterLinkMap->l_next;
            }
            if(formerLinkMap){
                linkMap = formerLinkMap;
            } else {
                linkMap = latterLinkMap;
            }
        }
        if(!linkMap){
            break;
        }
        if(regex_match(linkMap->l_name, match, pattern)){
            libPythonName = linkMap->l_name;
            break;
        }
    }

    return libPythonName;
}

/*
  The symbols of shared library "libpython" must be exported so that Python modules written in the C API
  can be imported because usually C-API Python modules are not explicitly linked with a particular
  libpython file. This is probably because the modules should not depend on a particular minor version of
  Python. Symbols can be exported to use the dlopen function with the RTLD_GLOBAL option in Linux.
*/
void exportLibPythonSymbols()
{
    // Determine the path of this shared library (CnoidPythonInterpreter) itself, which is
    // linked with libpython, so that findLibPython can locate libpython through
    // its link map.
    Dl_info info;
    if(dladdr(reinterpret_cast<void*>(&exportLibPythonSymbols), &info) && info.dli_fname){
        if(auto libPython = findLibPython(info.dli_fname)){
            dlopen(libPython, RTLD_LAZY | RTLD_GLOBAL);
            return;
        }
    }
    MessageOut::master()->putln(
        "Failed to export the libpython symbols. The system may not be able to load binary Python modules.",
        MessageOut::Warning);
}

#endif


bool initializeInterpreter()
{
    PyConfig config;
    PyConfig_InitPythonConfig(&config);
    config.parse_argv = 0;
    config.install_signal_handlers = 0;

    /*
      Make a venv activated by the user effective for the embedded interpreter.
      Since Python is embedded by linking to libpython (not by running a python
      executable), CPython's normal venv detection, which looks for a pyvenv.cfg
      next to sys.executable, fails and the venv's site-packages are ignored.
      Setting config.executable to the python in the venv lets CPython detect the
      venv via its pyvenv.cfg and add the site-packages to the module search paths.

      The venv is adopted only when its Python version matches the linked
      libpython; otherwise loading native extension modules (e.g. numpy's .so)
      built for a different ABI could crash the process. The match is checked by
      the per-version library directory name (e.g. "python3.13t"), which encodes
      the major.minor version and the "t" suffix for a free-threaded build.
    */
#ifndef _WIN32
    if(const char* virtualEnv = getenv("VIRTUAL_ENV")){
        if(virtualEnv[0] != '\0'){
#ifdef Py_GIL_DISABLED
            const char* abiSuffix = "t";
#else
            const char* abiSuffix = "";
#endif
            string libDirName = formatC("python{0}.{1}{2}", PY_MAJOR_VERSION, PY_MINOR_VERSION, abiSuffix);
            filesystem::path libPath = filesystem::path(virtualEnv) / "lib" / libDirName;
            if(filesystem::exists(libPath)){
                string venvPython = string(virtualEnv) + "/bin/python";
                wchar_t* w = Py_DecodeLocale(venvPython.c_str(), nullptr);
                if(w){
                    PyConfig_SetString(&config, &config.executable, w);
                    PyMem_RawFree(w);
                }
            } else {
                MessageOut::master()->putln(
                    formatC("The activated Python virtual environment \"{0}\" is not for Python {1}.{2}{3} "
                            "to which Choreonoid is linked, so it is not used by the embedded Python interpreter.",
                            virtualEnv, PY_MAJOR_VERSION, PY_MINOR_VERSION, abiSuffix),
                    MessageOut::Warning);
            }
        }
    }
#endif

    /*
      Some python modules require argv and missing argv may cause AttributeError.
      (Ex. AttributeError: 'module' object has no attribute 'argv')
      To avoid this problem, set dummy argv to python interpreter.
    */
    wchar_t* dummy_argv[] = { const_cast<wchar_t*>(L"choreonoid") };
    PyConfig_SetArgv(&config, 1, dummy_argv);

    PyStatus status = Py_InitializeFromConfig(&config);
    PyConfig_Clear(&config);
    if(PyStatus_Exception(status)){
        MessageOut::master()->putln(
            "Failed to initialize the embedded Python interpreter.", MessageOut::Error);
        return false;
    }

    mainModule_ = PyImport_ImportModule("__main__");
    if(!mainModule_){
        return false;
    }
    globalNamespace_ = PyModule_GetDict(mainModule_); // borrowed

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
    {
        PyObject* osModule = PyImport_ImportModule("os");
        if(osModule){
            // Note: "environ" is a macro in the MSVC CRT (expands to _environ),
            // so the local variable cannot be named "environ".
            PyObject* osEnviron = PyObject_GetAttrString(osModule, "environ");
            if(osEnviron){
                PyObject* oldPath = PyObject_GetItem(osEnviron, PyUnicode_FromString("PATH"));
                string newPath = executableDir() + ";";
                if(oldPath){
                    const char* s = PyUnicode_AsUTF8(oldPath);
                    if(s){
                        newPath += s;
                    }
                    Py_DECREF(oldPath);
                }
                PyObject* key = PyUnicode_FromString("PATH");
                PyObject* value = PyUnicode_FromString(newPath.c_str());
                PyObject_SetItem(osEnviron, key, value);
                Py_XDECREF(key);
                Py_XDECREF(value);
                Py_DECREF(osEnviron);
            }
            Py_DECREF(osModule);
        }
    }
#endif

    sysModule_ = PyImport_ImportModule("sys");
    if(!sysModule_){
        return false;
    }

    PyObject_SetAttrString(sysModule_, "dont_write_bytecode", Py_True);

    // set the choreonoid default python module path
    filesystem::path modulePath = pluginDirPath() / "python";
    auto moduleDir = toUTF8(modulePath.make_preferred().string());
    {
        PyObject* path = PyObject_GetAttrString(sysModule_, "path");
        if(path){
            PyObject* dir = PyUnicode_FromString(moduleDir.c_str());
            PyObject* r = PyObject_CallMethod(path, "insert", "iO", 0, dir);
            Py_XDECREF(r);
            Py_XDECREF(dir);
            Py_DECREF(path);
        }
    }

#ifndef _WIN32
    auto pythonPath0 = getenv("PYTHONPATH");
    if(pythonPath0){
        setenv("PYTHONPATH", formatC("{0}:{1}", moduleDir, pythonPath0).c_str(), 1);
    } else {
        setenv("PYTHONPATH", moduleDir.c_str(), 1);
    }
#endif

    return true;
}

}


bool cnoid::isPythonAvailable()
{
    return isAvailable;
}


PyObject* cnoid::pythonMainModule()
{
    return mainModule_;
}


PyObject* cnoid::pythonGlobalNamespace()
{
    return globalNamespace_;
}


PyObject* cnoid::pythonSysModule()
{
    return sysModule_;
}


int cnoid_initializePythonInterpreter()
{
    if(isAvailable){
        return 1;
    }

    if(!initializeInterpreter()){
        return 0;
    }

    // Release the GIL acquired by the interpreter initialization. The saved
    // thread state is restored when the interpreter is finalized.
    mainThreadState = PyEval_SaveThread();

#ifdef __linux__
    exportLibPythonSymbols();
#endif

    isAvailable = true;
    return 1;
}


void cnoid_finalizePythonInterpreter()
{
    if(!isAvailable){
        return;
    }
    isAvailable = false;

    if(mainThreadState){
        // Re-acquire the GIL that was released after the interpreter was
        // initialized so that the interpreter can be finalized safely.
        PyEval_RestoreThread(mainThreadState);
        mainThreadState = nullptr;
    }

    mainModule_ = nullptr;
    globalNamespace_ = nullptr;
    sysModule_ = nullptr;

    // This must be the very last step of the application shutdown. Under the
    // nanobind backend the Python wrappers own the Referenced-derived C++
    // objects, so finalizing the interpreter runs their C++ destructors here.
    // By calling this after every plugin has been finalized and the item tree
    // has been released, no other shutdown code runs afterwards, so destroying
    // the remaining wrappers cannot interfere with anything else.
    Py_FinalizeEx();
}
