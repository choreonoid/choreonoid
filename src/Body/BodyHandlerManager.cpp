#include "BodyHandlerManager.h"
#include "BodyHandler.h"
#include "Body.h"
#include <cnoid/ExecutablePath>
#include <cnoid/FileUtil>
#include <fmt/format.h>
#include <unordered_map>
#include <iostream>
#ifdef _WIN32
# include <windows.h>
#endif
#include "gettext.h"

using namespace std;
using namespace cnoid;
using namespace fmt;
namespace filesystem = cnoid::stdx::filesystem;

namespace {

typedef BodyHandler* (*CreateCnoidBodyHandlerFunc)(std::ostream& os);

#ifdef _WIN32
const char* CNOID_BODY_SHARE_DIR="";
typedef HINSTANCE DllHandle;
DllHandle loadDll(const char* filename) { return LoadLibrary(filename); }
void* resolveDllSymbol(DllHandle handle, const char* symbol) { return GetProcAddress(handle, symbol); }
void unloadDll(DllHandle handle) { FreeLibrary(handle); }
#else
# include <dlfcn.h>
typedef void* DllHandle;
DllHandle loadDll(const char* filename) { return dlopen(filename, RTLD_LAZY); }
void* resolveDllSymbol(DllHandle handle, const char* symbol) { return dlsym(handle, symbol); }
void unloadDll(DllHandle handle) { dlclose(handle); }
#endif

}

namespace cnoid {

class BodyHandlerManagerImpl
{
public:
    typedef unordered_map<string, CreateCnoidBodyHandlerFunc> FileToHandlerFactoryMap;
    FileToHandlerFactoryMap handlerFactories;
    vector<filesystem::path> handlerPaths;
    ostream* os;

    BodyHandlerManagerImpl();
    bool loadBodyHandler(Body* body, const string& filename);
    CreateCnoidBodyHandlerFunc loadHandlerFactory(filesystem::path& path, const string& handlerName);
    CreateCnoidBodyHandlerFunc loadHandlerFactoryWithFullPath(filesystem::path& path, const string& handlerName);
};

}


BodyHandlerManager::BodyHandlerManager()
{
    impl = new BodyHandlerManagerImpl;
}


BodyHandlerManagerImpl::BodyHandlerManagerImpl()
{
    handlerPaths.push_back(filesystem::path(pluginDirectory()) / "bodyhandler");
    os = &cout;
}


void BodyHandlerManager::setMessageSink(std::ostream& os)
{
    impl->os = &os;
}


bool BodyHandlerManager::loadBodyHandler(Body* body, const std::string& filename)
{
    return impl->loadBodyHandler(body, filename);
}


bool BodyHandlerManagerImpl::loadBodyHandler(Body* body, const string& filename)
{
    bool loaded = false;

    filesystem::path path(filename);
    string handlerName = path.stem().string();
    
    CreateCnoidBodyHandlerFunc createHandler = nullptr;
    auto iter = handlerFactories.find(filename);
    if(iter != handlerFactories.end()){
        createHandler = iter->second;
    } else {
        createHandler = loadHandlerFactory(path, handlerName);
    }

    if(createHandler){
        auto handler = createHandler(*os);
        if(handler){
            if(handler->initialize(body, *os)){
                loaded = body->addHandler(handler);
            }
        }
    }

    if(loaded){
        *os << format(_("Body handler {0} has been loaded to {1}."),
                      handlerName, body->name()) << endl;
    } else {
        *os << format(_("Body handler {0} cannot be loaded to {1}."),
                      handlerName, body->name()) << endl;
    }
    
    return loaded;
}


CreateCnoidBodyHandlerFunc BodyHandlerManagerImpl::loadHandlerFactory
(filesystem::path& path, const string& handlerName)
{
    CreateCnoidBodyHandlerFunc factory = nullptr;
    
    if(path.is_absolute()){
        factory = loadHandlerFactoryWithFullPath(path, handlerName);
    } else {
        for(auto& handlerPath : handlerPaths){
            filesystem::path fullPath = handlerPath / path;
            factory = loadHandlerFactoryWithFullPath(fullPath, handlerName);
            if(factory){
                break;
            }
        }
    }

    if(!factory){
        *os << format(_("Body handler {0} was not found."), handlerName) << endl;
    }

    return factory;
}


CreateCnoidBodyHandlerFunc BodyHandlerManagerImpl::loadHandlerFactoryWithFullPath
(filesystem::path& path, const string& handlerName)
{
    CreateCnoidBodyHandlerFunc factory = nullptr;

    auto filename = path.string();
    auto extension = getExtension(path);
    if(extension != DLL_SUFFIX){
        filename += DLL_SUFFIX;
    }
    
    DllHandle dll = loadDll(filename.c_str());
    if(dll){
        factory = (CreateCnoidBodyHandlerFunc)resolveDllSymbol(dll, "createCnoidBodyHandler");
        if(!factory){
            unloadDll(dll);
            *os << format(_("Body handler {0} found at \"{1}\" does not have the factory function \"createCnoidBodyHandler\"."),
                          handlerName, filename) << endl;
        }
    }

    if(factory){
        *os << format(_("Body handler {0} was found at \"{1}\"."),
                      handlerName, filename) << endl;
    }

    return factory;
}
