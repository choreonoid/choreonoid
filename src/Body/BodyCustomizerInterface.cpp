/**
   \file
   \brief The implementation for the BodyCustomizer class
   \author Shin'ichiro Nakaoka
*/

#include "BodyCustomizerInterface.h"
#include "Body.h"
#include <cnoid/FileUtil>
#include <cnoid/Tokenizer>
#include <map>
#include <set>
#include <cstdlib>
#include <iostream>
#include <fmt/format.h>

#ifdef _WIN32
#include <windows.h>
#endif

#include "gettext.h"

using namespace std;
using namespace cnoid;
namespace filesystem = cnoid::stdx::filesystem;
using fmt::format;

namespace {

#ifdef _WIN32
const char* CNOID_BODY_SHARE_DIR="";
typedef HINSTANCE DllHandle;
inline DllHandle loadDll(const char* filename) { return LoadLibrary(filename); }
inline void* resolveDllSymbol(DllHandle handle, const char* symbol) { return GetProcAddress(handle, symbol); }
inline void unloadDll(DllHandle handle) { FreeLibrary(handle); }
#else
# include <dlfcn.h>
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
    return (customizerInterface->version <= BODY_CUSTOMIZER_INTERFACE_VERSION) &&
        customizerInterface->getTargetModelNames &&
        customizerInterface->create &&
        customizerInterface->destroy;
}


static bool loadCustomizerDll(BodyInterface* bodyInterface, const std::string filename, std::ostream& os)
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
                    os << format(_("Body customizer \"{}\" is incomatible and cannot be loaded."), filename) << endl;
                } else {
                
                    const char** names = customizerInterface->getTargetModelNames();
                    string modelNames;
		    for(int i=0; names[i]; ++i){
                        if(i > 0){
                            modelNames += ", ";
                        }
                        string name(names[i]);
                        if(!name.empty()){
                            customizerRepository[name] = customizerInterface;
                        }
                        modelNames += name;
                    }
                    os << format(_("Body customizer \"{0}\" for {1} has been loaded."), filename, modelNames) << endl;
                }
            }
        }
    }
	
    return (customizerInterface != 0);
}


static int loadBodyCustomizers(BodyInterface* bodyInterface, const std::string pathString, std::ostream& os)
{
    pluginLoadingFunctionsCalled = true;
	
    int numLoaded = 0;

    filesystem::path pluginPath(pathString);
	
    if(filesystem::exists(pluginPath)){
        if(!filesystem::is_directory(pluginPath)){
            if(loadCustomizerDll(bodyInterface, pathString, os)){
                numLoaded++;
            }
        } else {
            static const string pluginNamePattern(string("Customizer") + DLL_SUFFIX);
            filesystem::directory_iterator end;
            for(filesystem::directory_iterator it(pluginPath); it != end; ++it){
                const filesystem::path& filepath = *it;
                if(!filesystem::is_directory(filepath)){
                    string filename(getFilename(filepath));
                    size_t pos = filename.rfind(pluginNamePattern);
                    if(pos == (filename.size() - pluginNamePattern.size())){
                        if(loadCustomizerDll(bodyInterface, getNativePathString(filepath), os)){
                            numLoaded++;
                        }
                    }
                }
            }
        }
    }

    return numLoaded;
}


/**
   DLLs of body customizer in the path are loaded and
   they are registered to the customizer repository.

   The loaded customizers can be obtained by using
   findBodyCustomizer() function.

   \param pathString the path to a DLL file or a directory that contains DLLs
*/
int cnoid::loadBodyCustomizers(const std::string pathString, std::ostream& os)
{
    return ::loadBodyCustomizers(Body::bodyInterface(), pathString, os);
}


static int loadBodyCustomizers(BodyInterface* bodyInterface, std::ostream& os)
{
    int numLoaded = 0;

    if(!pluginLoadingFunctionsCalled){

        pluginLoadingFunctionsCalled = true;
        
        char* pathListEnv = getenv("CNOID_CUSTOMIZER_PATH");
        if(pathListEnv){
            string pathList = pathListEnv;
            for(auto& path : Tokenizer<CharSeparator<char>>(pathList, CharSeparator<char>(PATH_DELIMITER))){
                numLoaded = ::loadBodyCustomizers(bodyInterface, path, os);
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
            numLoaded += ::loadBodyCustomizers(bodyInterface, *p, os);
        }
    }

    return numLoaded;
}


/**
   The function loads the customizers in the directories specified
   by the environmental variable CNOID_CUSTOMIZER_PATH
*/
int cnoid::loadDefaultBodyCustomizers(std::ostream& os)
{
    static bool loaded = false;
    int numLoaded = 0;
    if(!loaded){
        numLoaded = ::loadBodyCustomizers(Body::bodyInterface(), os);
        loaded = true;
    }
    return numLoaded;
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
