#include "BodyHandlerManager.h"
#include "BodyHandler.h"
#include "Body.h"
#include <cnoid/ExecutablePath>
#include <cnoid/FileUtil>
#include <fmt/format.h>
#include <unordered_map>
#include <iostream>
#include "gettext.h"

using namespace std;
using namespace cnoid;
namespace filesystem = boost::filesystem;

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
    CreateCnoidBodyHandlerFunc loadHandlerFactory(const string& filename);
    CreateCnoidBodyHandlerFunc loadHandlerFactoryWithFullPath(const string& filename);
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

    CreateCnoidBodyHandlerFunc createHandler = nullptr;
    auto iter = handlerFactories.find(filename);
    if(iter != handlerFactories.end()){
        createHandler = iter->second;
    } else {
        createHandler = loadHandlerFactory(filename);
    }

    if(createHandler){
        auto handler = createHandler(*os);
        if(handler){
            loaded = handler->initialize(body, *os);
            if(loaded){
                body->addHandler(handler);
            }
        }
    }
    
    return loaded;
}


CreateCnoidBodyHandlerFunc BodyHandlerManagerImpl::loadHandlerFactory(const string& filename)
{
    CreateCnoidBodyHandlerFunc factory = nullptr;
    
    filesystem::path path(filename);
    if(path.is_absolute()){
        factory = loadHandlerFactoryWithFullPath(filename);
    } else {
        for(auto& handlerPath : handlerPaths){
            filesystem::path fullPath = handlerPath / filename;
            factory = loadHandlerFactoryWithFullPath(fullPath.string());
            if(factory){
                break;
            }
        }
    }

    return factory;
}


CreateCnoidBodyHandlerFunc BodyHandlerManagerImpl::loadHandlerFactoryWithFullPath(const string& filename)
{
    CreateCnoidBodyHandlerFunc factory = nullptr;
    
    DllHandle dll = loadDll(filename.c_str());
    if(dll){
        factory = (CreateCnoidBodyHandlerFunc)resolveDllSymbol(dll, "createCnoidBodyHandler");
        if(!factory){
            unloadDll(dll);
        }
    }

    return factory;
}
