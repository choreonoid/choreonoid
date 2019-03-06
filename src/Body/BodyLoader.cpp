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
namespace filesystem = boost::filesystem;

namespace {

typedef std::function<AbstractBodyLoaderPtr()> LoaderFactory;
typedef map<string, LoaderFactory> LoaderFactoryMap;
LoaderFactoryMap loaderFactoryMap;
std::mutex loaderFactoryMapMutex;

class SceneLoaderAdapter : public AbstractBodyLoader
{
    AbstractSceneLoader* loader;
    ostream* os;

public:
    SceneLoaderAdapter(AbstractSceneLoader* loader) : loader(loader) {
        os = &nullout();
    }
    ~SceneLoaderAdapter() { delete loader; }

    virtual void setMessageSink(std::ostream& os_)
    {
        os = &os_;
    }

    virtual bool load(Body* body, const std::string& filename) {

        body->clearDevices();
        body->clearExtraJoints();

        loader->setMessageSink(*os);
        SgNode* scene = loader->load(filename);
        if(scene){
            Link* link = body->createLink();
            link->setName("Root");
            link->setShape(scene);
            link->setMass(1.0);
            link->setInertia(Matrix3::Identity());
            body->setRootLink(link);
            body->setModelName(filesystem::path(filename).stem().string());
        }

        return (scene != 0);
    }
};

struct FactoryRegistration
{
    FactoryRegistration(){
        BodyLoader::registerLoader(
            "body", [](){ return std::make_shared<YAMLBodyLoader>(); });
        BodyLoader::registerLoader(
            "yaml", [](){ return std::make_shared<YAMLBodyLoader>(); });
        BodyLoader::registerLoader(
            "yml", [](){ return std::make_shared<YAMLBodyLoader>(); });
        BodyLoader::registerLoader(
            "wrl", [](){ return std::make_shared<VRMLBodyLoader>(); });
        BodyLoader::registerLoader(
            "scen", [](){ return std::make_shared<SceneLoaderAdapter>(new YAMLSceneLoader); });
        BodyLoader::registerLoader(
            "stl", [](){ return std::make_shared<SceneLoaderAdapter>(new STLSceneLoader); });
        BodyLoader::registerLoader(
            "dae", [](){ return std::make_shared<SceneLoaderAdapter>(new SceneLoader); });
    }
} factoryRegistration;
    
}

bool BodyLoader::registerLoader(const std::string& extension, std::function<AbstractBodyLoaderPtr()> factory)
{
    std::lock_guard<std::mutex> lock(loaderFactoryMapMutex);
    loaderFactoryMap[extension] = factory;
    return  true;
}
   

namespace cnoid {

class BodyLoaderImpl
{
public:
    ostream* os;
    AbstractBodyLoaderPtr actualLoader;
    bool isVerbose;
    bool isShapeLoadingEnabled;
    int defaultDivisionNumber;
    double defaultCreaseAngle;

    typedef map<string, AbstractBodyLoaderPtr> BodyLoaderMap;
    BodyLoaderMap bodyLoaderMap;
        
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
    bool result = false;

    filesystem::path path(filename);
    string ext = getExtension(path);

    try {
        auto p = bodyLoaderMap.find(ext);
        if(p != bodyLoaderMap.end()){
            actualLoader = p->second;
        } else {
            std::lock_guard<std::mutex> lock(loaderFactoryMapMutex);
            auto q = loaderFactoryMap.find(ext);
            if(q != loaderFactoryMap.end()){
                LoaderFactory factory = q->second;
                actualLoader = factory();
                bodyLoaderMap[ext] = actualLoader;
            }
        }

        if(!actualLoader){
            (*os) <<
                fmt::format(_("The file format of \"{}\" is not supported by the body loader.\n"),
                            path.filename().string());
        } else {
            actualLoader->setMessageSink(*os);
            actualLoader->setVerbose(isVerbose);
            actualLoader->setShapeLoadingEnabled(isShapeLoadingEnabled);
            actualLoader->setDefaultDivisionNumber(defaultDivisionNumber);
            actualLoader->setDefaultCreaseAngle(defaultCreaseAngle);

            result = actualLoader->load(body, filename);
        }
        
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
