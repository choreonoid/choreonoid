#include "BodyHandlerManager.h"
#include "BodyHandler.h"
#include "Body.h"
#include <cnoid/ExecutablePath>
#include <cnoid/UTF8>
#include <cnoid/FileUtil>
#include <cnoid/stdx/filesystem>
#include <fmt/format.h>
#include <unordered_map>
#include <iostream>
#ifdef _WIN32
#include <windows.h>
#else
#include <dlfcn.h>
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
typedef void* DllHandle;
DllHandle loadDll(const char* filename) { return dlopen(filename, RTLD_LAZY); }
void* resolveDllSymbol(DllHandle handle, const char* symbol) { return dlsym(handle, symbol); }
void unloadDll(DllHandle handle) { dlclose(handle); }
#endif

}

namespace cnoid {

class BodyHandlerManager::Impl
{
public:
    typedef unordered_map<string, CreateCnoidBodyHandlerFunc> FileToHandlerFactoryMap;
    FileToHandlerFactoryMap handlerFactories;
    vector<filesystem::path> handlerPaths;
    ostream* os;

    Impl();
    bool loadBodyHandler(Body* body, const string& filename);
    CreateCnoidBodyHandlerFunc loadHandlerFactory(filesystem::path& path, const string& handlerName);
    CreateCnoidBodyHandlerFunc loadHandlerFactoryWithFullPath(filesystem::path& path, const string& handlerName);
};

}


BodyHandlerManager::BodyHandlerManager()
{
    impl = new Impl;
}


BodyHandlerManager::Impl::Impl()
{
    handlerPaths.push_back(pluginDirPath() / "bodyhandler");
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


bool BodyHandlerManager::Impl::loadBodyHandler(Body* body, const string& filename)
{
    bool loaded = false;

    filesystem::path path(fromUTF8(filename));
    string handlerName = toUTF8(path.stem().string());
    
    CreateCnoidBodyHandlerFunc createHandler = nullptr;
    auto iter = handlerFactories.find(filename);
    if(iter != handlerFactories.end()){
        createHandler = iter->second;
    } else {
        createHandler = loadHandlerFactory(path, handlerName);
        handlerFactories[filename] = createHandler;
    }

    if(createHandler){
        auto handler = createHandler(*os);
        if(handler){
            handler->filename_ = filename;
            if(handler->initialize(body, *os)){
                loaded = body->addHandler(handler);
            }
        }
    }

    if(loaded){
        *os << format(_("Body handler {0} has been loaded to {1}."),
                      handlerName, body->modelName()) << endl;
    } else {
        *os << format(_("Body handler {0} cannot be loaded to {1}."),
                      handlerName, body->modelName()) << endl;
    }
    
    return loaded;
}


CreateCnoidBodyHandlerFunc BodyHandlerManager::Impl::loadHandlerFactory
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


CreateCnoidBodyHandlerFunc BodyHandlerManager::Impl::loadHandlerFactoryWithFullPath
(filesystem::path& path, const string& handlerName)
{
    CreateCnoidBodyHandlerFunc factory = nullptr;

    if(path.extension().string() != DLL_SUFFIX){
        path += DLL_SUFFIX;
    }
    
    DllHandle dll = loadDll(path.string().c_str());
    if(dll){
        factory = (CreateCnoidBodyHandlerFunc)resolveDllSymbol(dll, "createCnoidBodyHandler");
        if(!factory){
            unloadDll(dll);
            *os << format(_("Body handler {0} found at \"{1}\" does not have the factory function \"createCnoidBodyHandler\"."),
                          handlerName, toUTF8(path.string())) << endl;
        }
    }

    if(factory){
        *os << format(_("Body handler {0} was found at \"{1}\"."),
                      handlerName, toUTF8(path.string())) << endl;
    }

    return factory;
}
