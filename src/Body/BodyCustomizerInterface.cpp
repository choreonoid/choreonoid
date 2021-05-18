/**
   \file
   \brief The implementation for the BodyCustomizer class
   \author Shin'ichiro Nakaoka
*/

#include "BodyCustomizerInterface.h"
#include "Body.h"
#include <cnoid/ExecutablePath>
#include <cnoid/Tokenizer>
#include <cnoid/FileUtil>
#include <cnoid/UTF8>
#include <cnoid/stdx/filesystem>
#include <map>
#include <set>
#include <cstdlib>
#include <iostream>
#include <fmt/format.h>

#ifdef _WIN32
#include <windows.h>
#else
#include <dlfcn.h>
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


static bool loadCustomizerDll
(BodyInterface* bodyInterface, const std::string nativeFilename, std::ostream& os)
{
    BodyCustomizerInterface* customizerInterface = 0;

    DllHandle dll = loadDll(nativeFilename.c_str());
	
    if(dll){
		
        GetBodyCustomizerInterfaceFunc getCustomizerInterface =
            (GetBodyCustomizerInterfaceFunc)resolveDllSymbol(dll, "getHrpBodyCustomizerInterface");
		
        if(!getCustomizerInterface){
            unloadDll(dll);
        } else {
            customizerInterface = getCustomizerInterface(bodyInterface);
			
            if(customizerInterface){
				
                if(!checkInterface(customizerInterface)){
                    os << format(_("Body customizer \"{}\" is incomatible and cannot be loaded."),
                                 toUTF8(nativeFilename)) << endl;
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
                    os << format(_("Body customizer \"{0}\" for {1} has been loaded."),
                                 toUTF8(nativeFilename), modelNames) << endl;
                }
            }
        }
    }
	
    return (customizerInterface != 0);
}


static int loadBodyCustomizers
(BodyInterface* bodyInterface, const std::string pathString, std::ostream& os)
{
    pluginLoadingFunctionsCalled = true;
	
    int numLoaded = 0;

    filesystem::path pluginPath(fromUTF8(pathString));
	
    if(filesystem::exists(pluginPath)){
        if(!filesystem::is_directory(pluginPath)){
            if(loadCustomizerDll(bodyInterface, pluginPath.string(), os)){
                numLoaded++;
            }
        } else {
            static const string pluginNamePattern(string("Customizer") + DLL_SUFFIX);
            filesystem::directory_iterator end;
            for(filesystem::directory_iterator it(pluginPath); it != end; ++it){
                auto path = it->path();
                if(!filesystem::is_directory(path)){
                    string filename(toUTF8(path.filename().string()));
                    size_t pos = filename.rfind(pluginNamePattern);
                    if(pos == (filename.size() - pluginNamePattern.size())){
                        if(loadCustomizerDll(bodyInterface, path.make_preferred().string(), os)){
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
                numLoaded = ::loadBodyCustomizers(bodyInterface, toUTF8(path), os);
            }
        }

        for(auto& dir : customizerDirectories){
            numLoaded += ::loadBodyCustomizers(bodyInterface, dir, os);
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
