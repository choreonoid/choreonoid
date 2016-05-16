/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include "BodyLoader.h"
#include "YAMLBodyLoader.h"
#include "VRMLBodyLoader.h"
#include "ColladaBodyLoader.h"
#include "Body.h"
#include <cnoid/STLSceneLoader>
#include <cnoid/Exception>
#include <cnoid/YAMLReader>
#include <cnoid/FileUtil>
#include <cnoid/NullOut>
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>
#include <boost/make_shared.hpp>
#include "gettext.h"

using namespace std;
using namespace cnoid;
namespace filesystem = boost::filesystem;

namespace {

typedef boost::function<AbstractBodyLoaderPtr()> LoaderFactory;
typedef map<string, LoaderFactory> LoaderFactoryMap;
LoaderFactoryMap loaderFactoryMap;
boost::mutex loaderFactoryMapMutex;


AbstractBodyLoaderPtr yamlBodyLoaderFactory()
{
    return boost::make_shared<YAMLBodyLoader>();
}

AbstractBodyLoaderPtr vrmlBodyLoaderFactory()
{
    return boost::make_shared<VRMLBodyLoader>();
}

AbstractBodyLoaderPtr colladaBodyLoaderFactory()
{
    return boost::make_shared<ColladaBodyLoader>();
}

class SceneLoaderAdapter : public AbstractBodyLoader
{
    AbstractSceneLoader* loader;
public:
    SceneLoaderAdapter(AbstractSceneLoader* loader) : loader(loader) { }
    ~SceneLoaderAdapter() { delete loader; }
    virtual const char* format() const { return loader->format(); }

    virtual bool load(Body* body, const std::string& filename) {

        body->clearDevices();
        body->clearExtraJoints();

        SgNode* scene = loader->load(filename);
        if(scene){
            Link* link = body->createLink();
            link->setName("Root");
            link->setShape(scene);
            link->setMass(1.0);
            link->setInertia(Matrix3::Identity());
            body->setRootLink(link);
            body->setModelName(getBasename(filename));
        }

        return (scene != 0);
    }
};


AbstractBodyLoaderPtr stlBodyLoaderFactory()
{
    return boost::make_shared<SceneLoaderAdapter>(new STLSceneLoader);
}

    
struct FactoryRegistration
{
    FactoryRegistration(){
        BodyLoader::registerLoader("body", yamlBodyLoaderFactory);
        BodyLoader::registerLoader("yaml", yamlBodyLoaderFactory);
        BodyLoader::registerLoader("yml", yamlBodyLoaderFactory);
        BodyLoader::registerLoader("wrl", vrmlBodyLoaderFactory);
        BodyLoader::registerLoader("dae", colladaBodyLoaderFactory);
        BodyLoader::registerLoader("stl", stlBodyLoaderFactory);
    }
} factoryRegistration;
    
}


bool BodyLoader::registerLoader(const std::string& extension, boost::function<AbstractBodyLoaderPtr()> factory)
{
    boost::lock_guard<boost::mutex> lock(loaderFactoryMapMutex);
    loaderFactoryMap[extension] = factory;
    return  true;
}
    

namespace cnoid {

class BodyLoaderImpl
{
public:
    ostream* os;
    AbstractBodyLoaderPtr loader;
    bool isVerbose;
    bool isShapeLoadingEnabled;
    int defaultDivisionNumber;
    double defaultCreaseAngle;

    typedef map<string, AbstractBodyLoaderPtr> LoaderMap;
    LoaderMap loaderMap;
        
    BodyLoaderImpl();
    ~BodyLoaderImpl();
    bool load(Body* body, const std::string& filename);
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


const char* BodyLoader::format() const
{
    return "General";
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

    filesystem::path orgpath(filename);
    string ext = getExtension(orgpath);
    string modelFilename;
    MappingPtr yamlDoc;

    try {
        if(ext != "yaml"){
            modelFilename = filename;
        } else {
            YAMLReader parser;
            yamlDoc = parser.loadDocument(filename)->toMapping();

            ValueNode* modelFileNode = yamlDoc->find("modelFile");
            if(modelFileNode->isValid()){
                filesystem::path mpath(modelFileNode->toString());
                if(mpath.has_root_path()){
                    modelFilename = getNativePathString(mpath);
                } else {
                    modelFilename = getNativePathString(orgpath.parent_path() / mpath);
                }
                ext = getExtension(mpath);
            }
        }

        LoaderMap::iterator p = loaderMap.find(ext);
        if(p != loaderMap.end()){
            loader = p->second;
        } else {
            boost::lock_guard<boost::mutex> lock(loaderFactoryMapMutex);
            LoaderFactoryMap::iterator q = loaderFactoryMap.find(ext);
            if(q != loaderFactoryMap.end()){
                LoaderFactory factory = q->second;
                loader = factory();
                loaderMap[ext] = loader;
            }
        }

        if(!loader){
            (*os) << str(boost::format(_("The file format of \"%1%\" is not supported by the body loader.\n"))
                         % getFilename(filesystem::path(modelFilename)));

        } else {
            loader->setMessageSink(*os);
            loader->setVerbose(isVerbose);
            loader->setShapeLoadingEnabled(isShapeLoadingEnabled);

            YAMLBodyLoader* yamlBodyLoader = 0;
            if(yamlDoc){
                YAMLBodyLoader* yamlBodyLoader = dynamic_cast<YAMLBodyLoader*>(loader.get());
            }

            if(yamlBodyLoader){
                result = yamlBodyLoader->read(body, yamlDoc);

            } else {
                int dn = defaultDivisionNumber;
                if(yamlDoc){
                    Mapping& geometryInfo = *yamlDoc->findMapping("geometry");
                    if(geometryInfo.isValid()){
                        geometryInfo.read("divisionNumber", dn);
                    }
                }
                if(dn > 0){
                    loader->setDefaultDivisionNumber(dn);
                }

                if(defaultCreaseAngle >= 0.0){
                    loader->setDefaultCreaseAngle(defaultCreaseAngle);
                }
            
                if(yamlDoc){
                    body->resetInfo(yamlDoc);
                } else {
                    body->info()->clear();
                }

                result = loader->load(body, modelFilename);
            }
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
    return impl->loader;
}
