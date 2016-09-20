/**
   \file
   \brief The implementation for the BodyCustomizer class
   \author Shin'ichiro Nakaoka
*/

#ifdef _WIN64
#define NOT_USE_BOOST_ATOMIC
#endif

#include "BodyCustomizerInterface.h"
#include "Body.h"
#include <cnoid/FileUtil>
#include <map>
#include <set>
#include <cstdlib>
#include <iostream>
#include <boost/tokenizer.hpp>

using namespace cnoid;
using namespace std;
using namespace boost;

#ifdef _WIN32
# include <windows.h>
#endif

namespace {

#ifdef _WIN32
const char* DLLSFX = ".dll";
const char* PATH_DELIMITER = ";";
const char* CNOID_BODY_SHARE_DIR="";
typedef HINSTANCE DllHandle;
inline DllHandle loadDll(const char* filename) { return LoadLibrary(filename); }
inline void* resolveDllSymbol(DllHandle handle, const char* symbol) { return GetProcAddress(handle, symbol); }
inline void unloadDll(DllHandle handle) { FreeLibrary(handle); }
#else
# include <dlfcn.h>
#ifdef __darwin__
const char* DLLSFX = ".dylib";
#else
const char* DLLSFX = ".so";
#endif
const char* PATH_DELIMITER = ":";
typedef void* DllHandle;
inline DllHandle loadDll(const char* filename) { return dlopen(filename, RTLD_LAZY); }
inline void* resolveDllSymbol(DllHandle handle, const char* symbol) { return dlsym(handle, symbol); }
inline void unloadDll(DllHandle handle) { dlclose(handle); }
#endif

typedef std::map<std::string, BodyCustomizerInterface*> NameToInterfaceMap;
NameToInterfaceMap customizerRepository;
bool pluginLoadingFunctionsCalled = false;

set<string> customizerDirectories;
}


static bool checkInterface(BodyCustomizerInterface* customizerInterface)
{
    bool qualified = true
        && (customizerInterface->version == BODY_CUSTOMIZER_INTERFACE_VERSION)
        && customizerInterface->getTargetModelNames
        && customizerInterface->create
        && customizerInterface->destroy;
    //&& customizerInterface->initializeAnalyticIk
    //&& customizerInterface->calcAnalyticIk
    //&& customizerInterface->setVirtualJointForces;

    return qualified;
}


static bool loadCustomizerDll(BodyInterface* bodyInterface, const std::string filename)
{
    BodyCustomizerInterface* customizerInterface = 0;

    DllHandle dll = loadDll(filename.c_str());
	
    if(dll){
		
        GetBodyCustomizerInterfaceFunc getCustomizerInterface =
            (GetBodyCustomizerInterfaceFunc)resolveDllSymbol(dll, "getHrpBodyCustomizerInterface");
		
        if(!getCustomizerInterface){
            unloadDll(dll);
        } else {
            customizerInterface = getCustomizerInterface(bodyInterface);
			
            if(customizerInterface){
				
                if(!checkInterface(customizerInterface)){
                    cerr << "Body customizer \"" << filename << "\" is incomatible and cannot be loaded.";
                } else {
                    cerr << "Loading body customizer \"" << filename << "\" for ";
					
                    const char** names = customizerInterface->getTargetModelNames();
					
                    for(int i=0; names[i]; ++i){
                        if(i > 0){
                            cerr << ", ";
                        }
                        string name(names[i]);
                        if(!name.empty()){
                            customizerRepository[name] = customizerInterface;
                        }
                        cerr << names[i];
                    }
                    cerr << endl;
                }
            }
        }
    }
	
    return (customizerInterface != 0);
}


/**
   DLLs of body customizer in the path are loaded and
   they are registered to the customizer repository.

   The loaded customizers can be obtained by using
   findBodyCustomizer() function.

   \param pathString the path to a DLL file or a directory that contains DLLs
*/
int cnoid::loadBodyCustomizers(const std::string pathString, BodyInterface* bodyInterface)
{
    pluginLoadingFunctionsCalled = true;
	
    int numLoaded = 0;

    filesystem::path pluginPath(pathString);
	
    if(filesystem::exists(pluginPath)){
        if(!filesystem::is_directory(pluginPath)){
            if(loadCustomizerDll(bodyInterface, pathString)){
                numLoaded++;
            }
        } else {
            static const string pluginNamePattern(string("Customizer") + DLLSFX);
            filesystem::directory_iterator end;
            for(filesystem::directory_iterator it(pluginPath); it != end; ++it){
                const filesystem::path& filepath = *it;
                if(!filesystem::is_directory(filepath)){
                    string filename(getFilename(filepath));
                    size_t pos = filename.rfind(pluginNamePattern);
                    if(pos == (filename.size() - pluginNamePattern.size())){
                        if(loadCustomizerDll(bodyInterface, getNativePathString(filepath))){
                            numLoaded++;
                        }
                    }
                }
            }
        }
    }

    return numLoaded;
}


int cnoid::loadBodyCustomizers(const std::string pathString)
{
    return loadBodyCustomizers(pathString, Body::bodyInterface());
}


/**
   The function loads the customizers in the directories specified
   by the environmental variable CNOID_CUSTOMIZER_PATH
*/
int cnoid::loadBodyCustomizers(BodyInterface* bodyInterface)
{
    int numLoaded = 0;

    if(!pluginLoadingFunctionsCalled){

        pluginLoadingFunctionsCalled = true;

        char* pathListEnv = getenv("CNOID_CUSTOMIZER_PATH");

        if(pathListEnv){
            char_separator<char> sep(PATH_DELIMITER);
            string pathList(pathListEnv);
            tokenizer< char_separator<char> > paths(pathList, sep);
            tokenizer< char_separator<char> >::iterator p;
            for(p = paths.begin(); p != paths.end(); ++p){
                numLoaded = loadBodyCustomizers(*p, bodyInterface);
            }
        }

#ifndef _WIN32
        Dl_info info;
        if(dladdr((void*)&findBodyCustomizer, &info)){
            filesystem::path customizerPath =
                filesystem::path(info.dli_fname).parent_path().parent_path() / CNOID_PLUGIN_SUBDIR / "customizer";
            customizerDirectories.insert(getNativePathString(customizerPath));
        }
#else
        string customizerPath(CNOID_BODY_SHARE_DIR);
        customizerPath.append("/customizer");
        customizerDirectories.insert(string(CNOID_BODY_SHARE_DIR) + "/customzier");
#endif

        for(std::set<string>::iterator p = customizerDirectories.begin(); p != customizerDirectories.end(); ++p){
            numLoaded += loadBodyCustomizers(*p, bodyInterface);
        }
    }

    return numLoaded;
}


int cnoid::loadBodyCustomizers()
{
    return loadBodyCustomizers(Body::bodyInterface());
}


BodyCustomizerInterface* cnoid::findBodyCustomizer(std::string modelName)
{
    BodyCustomizerInterface* customizerInterface = 0;

    NameToInterfaceMap::iterator p = customizerRepository.find(modelName);
    if(p != customizerRepository.end()){
        customizerInterface = p->second;
    }

    return customizerInterface;
}


void Body::addCustomizerDirectory(const std::string& path)
{
    customizerDirectories.insert(path);
}
