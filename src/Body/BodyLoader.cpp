/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include "BodyLoader.h"
#include "StdBodyLoader.h"
#include "VRMLBodyLoader.h"
#include "Body.h"
#include <cnoid/SceneLoader>
#include <cnoid/ValueTree>
#include <cnoid/Exception>
#include <cnoid/NullOut>
#include <cnoid/UTF8>
#include <cnoid/stdx/filesystem>
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
    BodyLoader::LengthUnit lengthUnitHint;
    BodyLoader::UpperAxis upperAxisHint;
    ostream* os;

public:
    SceneLoaderAdapter()
    {
        os = &nullout();
    }

    void setMeshImportHint(BodyLoader::LengthUnit unit, BodyLoader::UpperAxis axis)
    {
        lengthUnitHint = unit;
        upperAxisHint = axis;
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
            link->setMass(1.0);
            link->setInertia(Matrix3::Identity());
            body->setRootLink(link);
            body->setModelName(toUTF8(filesystem::path(fromUTF8(filename)).stem().string()));

            SgNodePtr topNode = scene;
            /**
               \note Modifying the vertex positions might be better than
               inserting the transform nodes.
               \note This should be implemented in SceneLoader.
            */
            if(lengthUnitHint != BodyLoader::Meter){
                auto scale = new SgScaleTransform;
                if(lengthUnitHint == BodyLoader::Millimeter){
                    scale->setScale(1.0 / 1000.0);
                } else if(lengthUnitHint == BodyLoader::Inch){
                    scale->setScale(0.0254);
                }
                scale->addChild(topNode);
                topNode = scale;
            }
            if(upperAxisHint == BodyLoader::Y){
                auto transform = new SgPosTransform;
                Matrix3 R;
                R << 0, 0, 1,
                    1, 0, 0,
                    0, 1, 0;
                transform->setRotation(R);
                transform->addChild(topNode);
                topNode = transform;
            }
            link->addShapeNode(topNode);
        }
        if(!isSupported){
            (*os) <<
                fmt::format(_("The file format of \"{}\" is not supported by the body loader.\n"),
                            toUTF8(filesystem::path(fromUTF8(filename)).filename().string()));
        }

        return (scene != nullptr);
    }
};

struct FactoryRegistration
{
    FactoryRegistration(){
        lock_guard<mutex> lock(loaderMapMutex);

        unifiedExtensionMap["body"] = "body";
        unifiedExtensionMap["yaml"] = "body";
        unifiedExtensionMap["yml"] = "body";
        unifiedExtensionMap["wrl"] = "wrl";

        loaderFactoryMap["body"] = [](){ return make_shared<StdBodyLoader>(); };
        loaderFactoryMap["wrl"] = [](){ return make_shared<VRMLBodyLoader>(); };
    }
} factoryRegistration;
    
}

void BodyLoader::registerLoader(const std::string& extension, std::function<AbstractBodyLoaderPtr()> factory)
{
    lock_guard<mutex> lock(loaderMapMutex);
    unifiedExtensionMap[extension] = extension;
    loaderFactoryMap[extension] = factory;
}


void BodyLoader::registerLoader
(std::initializer_list<const char*> extensions, std::function<AbstractBodyLoaderPtr()> factory)
{
    auto iter = extensions.begin();
    if(iter != extensions.end()){
        string ext(*iter++);
        registerLoader(ext, factory);
        if(iter != extensions.end()){
            lock_guard<mutex> lock(loaderMapMutex);
            while(iter != extensions.end()){
                unifiedExtensionMap[*iter++] = ext;
            }
        }
    }
}


namespace cnoid {

class BodyLoader::Impl
{
public:
    ostream* os;
    AbstractBodyLoaderPtr actualLoader;
    map<string, AbstractBodyLoaderPtr> loaderMap;
    shared_ptr<SceneLoaderAdapter> loaderAdapter;
    bool isVerbose;
    bool isShapeLoadingEnabled;
    int defaultDivisionNumber;
    double defaultCreaseAngle;
    BodyLoader::LengthUnit lengthUnitHint;
    BodyLoader::UpperAxis upperAxisHint;

    Impl();
    ~Impl();
    bool load(Body* body, const std::string& filename);
    void mergeExtraLinkInfos(Body* body, Mapping* info);
};

}


BodyLoader::BodyLoader()
{
    impl = new Impl;
}


BodyLoader::Impl::Impl()
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


BodyLoader::Impl::~Impl()
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


void BodyLoader::setMeshImportHint(LengthUnit unit, UpperAxis upperAxis)
{
    impl->lengthUnitHint = unit;
    impl->upperAxisHint = upperAxis;
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
        return nullptr;
    }
}


bool BodyLoader::Impl::load(Body* body, const std::string& filename)
{
    filesystem::path path(fromUTF8(filename));
    actualLoader = nullptr;
    string ext = path.extension().string();

    if(!ext.empty()) {
        ext = ext.substr(1); // remove the dot
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
        if(!loaderAdapter){
            loaderAdapter = make_shared<SceneLoaderAdapter>();
        }
        loaderAdapter->setMeshImportHint(lengthUnitHint, upperAxisHint);
        actualLoader = loaderAdapter;
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
