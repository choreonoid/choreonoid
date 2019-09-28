/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include "BodyLoader.h"
#include "YAMLBodyLoader.h"
#include "VRMLBodyLoader.h"
#include "Body.h"
#include <cnoid/SceneLoader>
#include <cnoid/STLSceneLoader>
#include <cnoid/YAMLSceneLoader>
#include <cnoid/ValueTree>
#include <cnoid/Exception>
#include <cnoid/FileUtil>
#include <cnoid/NullOut>
#include <fmt/format.h>
#include <mutex>
#include "gettext.h"

using namespace std;
using namespace cnoid;
namespace filesystem = cnoid::stdx::filesystem;

namespace {

map<string, string> unifiedExtensionMap;
typedef function<AbstractBodyLoaderPtr()> LoaderFactory;
map<string, LoaderFactory> loaderFactoryMap;
mutex loaderMapMutex;

class SceneLoaderAdapter : public AbstractBodyLoader
{
    SceneLoader loader;
    ostream* os;

public:
    SceneLoaderAdapter(){
        os = &nullout();
    }

    virtual void setMessageSink(std::ostream& os_) override {
        os = &os_;
    }

    virtual bool load(Body* body, const std::string& filename) override {

        body->clearDevices();
        body->clearExtraJoints();

        loader.setMessageSink(*os);
        bool isSupported;
        SgNode* scene = loader.load(filename, isSupported);
        if(scene){
            Link* link = body->createLink();
            link->setName("Root");
            link->setShape(scene);
            link->setMass(1.0);
            link->setInertia(Matrix3::Identity());
            body->setRootLink(link);
            body->setModelName(filesystem::path(filename).stem().string());
        }
        if(!isSupported){
            (*os) <<
                fmt::format(_("The file format of \"{}\" is not supported by the body loader.\n"),
                            filesystem::path(filename).filename().string());
        }

        return (scene != nullptr);
    }
};

struct FactoryRegistration
{
    FactoryRegistration(){
        lock_guard<mutex> lock(loaderMapMutex);

        unifiedExtensionMap["body"] = "yaml";
        unifiedExtensionMap["yaml"] = "yaml";
        unifiedExtensionMap["yml"] = "yaml";
        unifiedExtensionMap["wrl"] = "wrl";

        loaderFactoryMap["yaml"] = [](){ return make_shared<YAMLBodyLoader>(); };
        loaderFactoryMap["wrl"] = [](){ return make_shared<VRMLBodyLoader>(); };
    }
} factoryRegistration;
    
}

bool BodyLoader::registerLoader(const std::string& extension, std::function<AbstractBodyLoaderPtr()> factory)
{
    lock_guard<mutex> lock(loaderMapMutex);
    unifiedExtensionMap[extension] = extension;
    loaderFactoryMap[extension] = factory;
    return  true;
}
   

namespace cnoid {

class BodyLoaderImpl
{
public:
    ostream* os;
    AbstractBodyLoaderPtr actualLoader;
    map<string, AbstractBodyLoaderPtr> loaderMap;
    AbstractBodyLoaderPtr generalLoader;
    bool isVerbose;
    bool isShapeLoadingEnabled;
    int defaultDivisionNumber;
    double defaultCreaseAngle;

    BodyLoaderImpl();
    ~BodyLoaderImpl();
    bool load(Body* body, const std::string& filename);
    void mergeExtraLinkInfos(Body* body, Mapping* info);
};

}


BodyLoader::BodyLoader()
{
    impl = new BodyLoaderImpl();
}


BodyLoaderImpl::BodyLoaderImpl()
{
    os = &nullout();
    isVerbose = false;
    isShapeLoadingEnabled = true;
    defaultDivisionNumber = -1;
    defaultCreaseAngle = -1.0;
}


BodyLoader::~BodyLoader()
{
    delete impl;
}


BodyLoaderImpl::~BodyLoaderImpl()
{

}


void BodyLoader::setMessageSink(std::ostream& os)
{
    impl->os = &os;
}


void BodyLoader::setVerbose(bool on)
{
    impl->isVerbose = on;
}


void BodyLoader::setShapeLoadingEnabled(bool on)
{
    impl->isShapeLoadingEnabled = on;
}
    

void BodyLoader::setDefaultDivisionNumber(int n)
{
    impl->defaultDivisionNumber = n;
}


void BodyLoader::setDefaultCreaseAngle(double theta)
{
    impl->defaultCreaseAngle = theta;
}


bool BodyLoader::load(Body* body, const std::string& filename)
{
    body->info()->clear();    
    return impl->load(body, filename);
}


Body* BodyLoader::load(const std::string& filename)
{
    Body* body = new Body();
    if(load(body, filename)){
        return body;
    } else {
        delete body;
        return 0;
    }
}


bool BodyLoaderImpl::load(Body* body, const std::string& filename)
{
    filesystem::path path(filename);
    string ext = getExtension(path);
    actualLoader = nullptr;

    {
        std::lock_guard<std::mutex> lock(loaderMapMutex);
        auto p = unifiedExtensionMap.find(ext);
        if(p != unifiedExtensionMap.end()){
            auto& uext = p->second;
            auto q = loaderMap.find(uext);
            if(q != loaderMap.end()){
                actualLoader = q->second;
            } else {
                actualLoader = loaderFactoryMap[uext]();
                loaderMap[uext] = actualLoader;
            }
        }
    }
    
    if(!actualLoader){
        if(!generalLoader){
            generalLoader = make_shared<SceneLoaderAdapter>();
        }
        actualLoader = generalLoader;
    }
    
    actualLoader->setMessageSink(*os);
    actualLoader->setVerbose(isVerbose);
    actualLoader->setShapeLoadingEnabled(isShapeLoadingEnabled);
    actualLoader->setDefaultDivisionNumber(defaultDivisionNumber);
    actualLoader->setDefaultCreaseAngle(defaultCreaseAngle);

    bool result = false;
    try {
        result = actualLoader->load(body, filename);

    } catch(const ValueNode::Exception& ex){
        (*os) << ex.message();
    } catch(const nonexistent_key_error& error){
        if(const std::string* message = boost::get_error_info<error_info_message>(error)){
            (*os) << *message;
        }
    } catch(const std::exception& ex){
        (*os) << ex.what();
    }
    os->flush();
    
    return result;
}


AbstractBodyLoaderPtr BodyLoader::lastActualBodyLoader() const
{
    return impl->actualLoader;
}
